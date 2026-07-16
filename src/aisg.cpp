#include "atc/aisg.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace atc::aisg {
namespace {

constexpr std::uint8_t xidFormatIdentifier = 0x81;
constexpr std::uint8_t xidUserGroup = 0xF0;

struct XidParameterView {
    std::uint8_t identifier{};
    std::span<const std::uint8_t> value;
};

std::optional<std::vector<XidParameterView>> parseXid(
    const std::span<const std::uint8_t> information) {
    if (information.size() < 3 || information[0] != xidFormatIdentifier ||
        information[1] != xidUserGroup || information[2] != information.size() - 3) {
        return std::nullopt;
    }
    std::vector<XidParameterView> result;
    std::size_t offset = 3;
    while (offset < information.size()) {
        if (information.size() - offset < 2) return std::nullopt;
        const auto identifier = information[offset++];
        const auto length = static_cast<std::size_t>(information[offset++]);
        if (length > information.size() - offset) return std::nullopt;
        result.push_back({identifier, information.subspan(offset, length)});
        offset += length;
    }
    return result;
}

const XidParameterView* findParameter(const std::vector<XidParameterView>& parameters,
                                      const std::uint8_t identifier) {
    const auto found = std::find_if(parameters.begin(), parameters.end(),
        [identifier](const auto& parameter) { return parameter.identifier == identifier; });
    return found == parameters.end() ? nullptr : &*found;
}

hdlc::Bytes encodeXid(
    const std::initializer_list<std::pair<std::uint8_t, hdlc::Bytes>> parameters) {
    hdlc::Bytes body;
    for (const auto& [identifier, value] : parameters) {
        if (value.size() > std::numeric_limits<std::uint8_t>::max()) {
            throw std::invalid_argument("AISG 2.0 XID parameter is too long");
        }
        body.push_back(identifier);
        body.push_back(static_cast<std::uint8_t>(value.size()));
        body.insert(body.end(), value.begin(), value.end());
    }
    if (body.size() > std::numeric_limits<std::uint8_t>::max()) {
        throw std::invalid_argument("AISG 2.0 XID group is too long");
    }
    hdlc::Bytes result{xidFormatIdentifier, xidUserGroup,
                       static_cast<std::uint8_t>(body.size())};
    result.insert(result.end(), body.begin(), body.end());
    return result;
}

constexpr std::array<std::uint8_t, 8> requestControls{
    0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE};
constexpr std::array<std::uint8_t, 8> keepAliveControls{
    0x11, 0x31, 0x51, 0x71, 0x91, 0xB1, 0xD1, 0xF1};

hdlc::Frame commandFrame(const std::uint8_t address,
                         const std::uint8_t control,
                         const Command command,
                         hdlc::Bytes tail = {0x00, 0x00}) {
    hdlc::Bytes information;
    information.reserve(tail.size() + 1);
    information.push_back(static_cast<std::uint8_t>(command));
    information.insert(information.end(), tail.begin(), tail.end());
    return {address, control, std::move(information)};
}

std::string readLengthPrefixed(const hdlc::Bytes& bytes, std::size_t& offset, bool& valid) {
    if (!valid || offset >= bytes.size()) {
        valid = false;
        return {};
    }
    const auto length = static_cast<std::size_t>(bytes[offset++]);
    if (length > bytes.size() - offset) {
        valid = false;
        return {};
    }
    std::string value(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                      bytes.begin() + static_cast<std::ptrdiff_t>(offset + length));
    offset += length;
    while (!value.empty() && value.back() == '\0') {
        value.pop_back();
    }
    return value;
}

std::string statusMessage(const std::uint8_t code) {
    switch (code) {
    case 0x00: return "success";
    case 0x02: return "motor jam";
    case 0x03: return "actuator jam";
    case 0x05: return "device busy";
    case 0x06: return "checksum error";
    case 0x0B: return "operation failed";
    case 0x0E: return "device is not calibrated";
    case 0x0F: return "device is not configured";
    case 0x11: return "hardware error";
    case 0x13: return "value is out of range";
    case 0x19: return "unknown procedure";
    case 0x1D: return "read-only parameter";
    case 0x1E: return "unknown parameter";
    case 0x24: return "format error";
    case 0x25: return "unsupported procedure";
    default:
        return "device returned status 0x" +
               hdlc::toHex(std::span<const std::uint8_t>(&code, 1));
    }
}

} // namespace

std::uint8_t ControlSequence::nextRequest() noexcept {
    const auto result = requestControls[requestIndex_];
    requestIndex_ = (requestIndex_ + 1) % requestControls.size();
    return result;
}

std::uint8_t ControlSequence::nextKeepAlive() noexcept {
    const auto result = keepAliveControls[keepAliveIndex_];
    keepAliveIndex_ = (keepAliveIndex_ + 1) % keepAliveControls.size();
    return result;
}

void ControlSequence::reset() noexcept {
    requestIndex_ = 0;
    keepAliveIndex_ = 0;
}

std::uint8_t expectedResponseControl(const std::uint8_t requestControl) noexcept {
    return static_cast<std::uint8_t>(requestControl + 0x20U);
}

bool isResponseFor(const hdlc::Frame& response, const hdlc::Frame& request) noexcept {
    if (response.address != request.address) {
        return false;
    }
    if (request.control == 0x93) {
        return response.control == 0x73;
    }
    if (request.information.empty()) {
        return response.control == request.control;
    }
    return response.control == expectedResponseControl(request.control) &&
           !response.information.empty() &&
           response.information.front() == request.information.front();
}

hdlc::Frame makeDeviceScanRequest(const std::span<const std::uint8_t> uniqueId,
                                  const std::span<const std::uint8_t> uniqueIdMask) {
    if (uniqueId.size() != uniqueIdMask.size() || uniqueId.size() > 19) {
        throw std::invalid_argument(
            "AISG 2.0 scan UID and mask must have equal lengths up to 19 octets");
    }
    return {allStationsAddress, xidControl,
            encodeXid({{1, hdlc::Bytes(uniqueId.begin(), uniqueId.end())},
                       {3, hdlc::Bytes(uniqueIdMask.begin(), uniqueIdMask.end())}})};
}

std::optional<XidIdentity> parseDeviceScanResponse(const hdlc::Frame& frame) {
    if (frame.address != noStationAddress || frame.control != xidControl) {
        return std::nullopt;
    }
    const auto parameters = parseXid(frame.information);
    if (!parameters) return std::nullopt;
    const auto* uid = findParameter(*parameters, 1);
    const auto* type = findParameter(*parameters, 4);
    const auto* vendor = findParameter(*parameters, 6);
    if (!uid || !type || !vendor || uid->value.size() < 3 || uid->value.size() > 19 ||
        type->value.size() != 1 || vendor->value.size() != 2 ||
        uid->value[0] != vendor->value[0] || uid->value[1] != vendor->value[1]) {
        return std::nullopt;
    }
    XidIdentity result;
    result.uniqueId.assign(uid->value.begin(), uid->value.end());
    result.deviceType = type->value[0];
    result.vendorCode = {static_cast<char>(vendor->value[0]),
                         static_cast<char>(vendor->value[1])};
    return result;
}

hdlc::Frame makeAddressAssignmentRequest(const XidIdentity& identity,
                                         const std::uint8_t address) {
    if (address == noStationAddress || address == allStationsAddress ||
        identity.uniqueId.size() < 3 || identity.uniqueId.size() > 19 ||
        static_cast<std::uint8_t>(identity.vendorCode[0]) != identity.uniqueId[0] ||
        static_cast<std::uint8_t>(identity.vendorCode[1]) != identity.uniqueId[1]) {
        throw std::invalid_argument("invalid AISG 2.0 address assignment");
    }
    return {allStationsAddress, xidControl,
            encodeXid({{1, identity.uniqueId}, {2, {address}}})};
}

bool isAddressAssignmentResponse(const hdlc::Frame& frame,
                                 const XidIdentity& identity,
                                 const std::uint8_t address) {
    if (frame.address != address || frame.control != xidControl) return false;
    const auto parameters = parseXid(frame.information);
    if (!parameters) return false;
    const auto* uid = findParameter(*parameters, 1);
    const auto* type = findParameter(*parameters, 4);
    return uid && type && type->value.size() == 1 &&
           std::ranges::equal(uid->value, identity.uniqueId) &&
           type->value[0] == identity.deviceType;
}

hdlc::Frame makeReleaseNegotiationRequest(const std::uint8_t address,
                                          const std::uint8_t release) {
    if (address == noStationAddress || address == allStationsAddress) {
        throw std::invalid_argument("AISG 2.0 release negotiation needs an assigned address");
    }
    return {address, xidControl, encodeXid({{5, {release}}})};
}

bool isReleaseNegotiationResponse(const hdlc::Frame& frame,
                                  const std::uint8_t address) {
    if (frame.address != address || frame.control != xidControl) return false;
    const auto parameters = parseXid(frame.information);
    if (!parameters) return false;
    const auto* release = findParameter(*parameters, 5);
    return release && release->value.size() == 1;
}

hdlc::Frame makeResetDeviceRequest(const std::uint8_t address) {
    return {address, xidControl, encodeXid({{7, {}}})};
}

hdlc::Frame makeSnrm(const std::uint8_t address) { return {address, 0x93, {}}; }

hdlc::Frame makeKeepAlive(const std::uint8_t address, const std::uint8_t control) {
    return {address, control, {}};
}

hdlc::Frame makeInitialDataRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::initialData);
}

hdlc::Frame makeGetTiltRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::getTilt);
}

hdlc::Frame makeSetTiltRequest(const std::uint8_t address,
                               const std::uint8_t control,
                               const double degrees) {
    if (!std::isfinite(degrees) || degrees < 0.0 || degrees > 6553.5) {
        throw std::invalid_argument("tilt cannot be represented in AISG deci-degrees");
    }
    const auto deciDegrees = static_cast<std::uint16_t>(std::lround(degrees * 10.0));
    return commandFrame(address, control, Command::setTilt,
                        {0x02, 0x00,
                         static_cast<std::uint8_t>(deciDegrees & 0xFFU),
                         static_cast<std::uint8_t>((deciDegrees >> 8U) & 0xFFU)});
}

hdlc::Frame makeGetAlarmsRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::getAlarms);
}

hdlc::Frame makeClearAlarmsRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::clearAlarms);
}

hdlc::Frame makeSubscribeAlarmsRequest(const std::uint8_t address,
                                       const std::uint8_t control) {
    return commandFrame(address, control, Command::subscribeAlarms);
}

hdlc::Frame makeSelfTestRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::selfTest);
}

hdlc::Frame makeCalibrateRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::calibrate);
}

hdlc::Frame makeSetTmaModeRequest(const std::uint8_t address,
                                  const std::uint8_t control,
                                  const TmaMode mode) {
    const auto value = mode == TmaMode::bypass ? std::uint8_t{1} : std::uint8_t{0};
    return commandFrame(address, control, Command::setTmaMode, {0x01, 0x00, value});
}

hdlc::Frame makeGetTmaModeRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::getTmaMode);
}

hdlc::Frame makeSetTmaGainRequest(const std::uint8_t address,
                                  const std::uint8_t control,
                                  const double gainDb) {
    if (!std::isfinite(gainDb) || gainDb < 0.0 || gainDb > 63.75) {
        throw std::invalid_argument("TMA gain cannot be represented in quarter-decibels");
    }
    const auto quarterDb = static_cast<std::uint8_t>(std::lround(gainDb * 4.0));
    return commandFrame(address, control, Command::setTmaGain, {0x01, 0x00, quarterDb});
}

hdlc::Frame makeGetTmaGainRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::getTmaGain);
}

hdlc::Frame makeGetDataRequest(const std::uint8_t address,
                               const std::uint8_t control,
                               const Field field) {
    return commandFrame(address, control, Command::getDeviceData,
                        {0x01, 0x00, static_cast<std::uint8_t>(field)});
}

hdlc::Frame makeSetDataRequest(const std::uint8_t address,
                               const std::uint8_t control,
                               const Field field,
                               const std::span<const std::uint8_t> data) {
    if (data.size() > 252) {
        throw std::invalid_argument("AISG device-data value exceeds 252 bytes");
    }
    hdlc::Bytes tail;
    tail.reserve(data.size() + 3);
    tail.push_back(static_cast<std::uint8_t>(data.size() + 1));
    tail.push_back(0x00);
    tail.push_back(static_cast<std::uint8_t>(field));
    tail.insert(tail.end(), data.begin(), data.end());
    return commandFrame(address, control, Command::setDeviceData, std::move(tail));
}

ProtocolStatus parseStatus(const hdlc::Frame& response, const Command expectedCommand) {
    if (response.information.empty() ||
        response.information.front() != static_cast<std::uint8_t>(expectedCommand)) {
        return {false, 0xFF, "unexpected or missing AISG endpoint"};
    }
    if (response.information.size() < 4) {
        return {false, 0xFF, "AISG response is truncated"};
    }
    const auto code = response.information[3];
    auto message = statusMessage(code);
    if (code == 0x0B && response.information.size() >= 5) {
        message += ": " + statusMessage(response.information[4]);
    }
    return {code == 0, code, std::move(message)};
}

std::optional<InitialData> parseInitialData(const hdlc::Frame& response) {
    const auto status = parseStatus(response, Command::initialData);
    if (response.information.size() < 4) {
        return std::nullopt;
    }
    std::size_t offset = 4;
    bool valid = true;
    InitialData result;
    result.status = status;
    result.product = readLengthPrefixed(response.information, offset, valid);
    result.serialNumber = readLengthPrefixed(response.information, offset, valid);
    result.hardwareVersion = readLengthPrefixed(response.information, offset, valid);
    result.softwareVersion = readLengthPrefixed(response.information, offset, valid);
    return valid ? std::optional<InitialData>(std::move(result)) : std::nullopt;
}

std::optional<double> parseTilt(const hdlc::Frame& response) {
    const auto status = parseStatus(response, Command::getTilt);
    if (!status.success || response.information.size() < 6) {
        return std::nullopt;
    }
    const auto deciDegrees = static_cast<std::uint16_t>(response.information[4]) |
                             (static_cast<std::uint16_t>(response.information[5]) << 8U);
    return static_cast<double>(deciDegrees) / 10.0;
}

std::optional<TmaMode> parseTmaMode(const hdlc::Frame& response) {
    const auto status = parseStatus(response, Command::getTmaMode);
    if (!status.success || response.information.size() < 5) {
        return std::nullopt;
    }
    return response.information[4] == 0 ? TmaMode::normal : TmaMode::bypass;
}

std::optional<double> parseTmaGain(const hdlc::Frame& response) {
    const auto status = parseStatus(response, Command::getTmaGain);
    if (!status.success || response.information.size() < 5) {
        return std::nullopt;
    }
    return static_cast<double>(response.information[4]) / 4.0;
}

std::optional<DataValue> parseData(const hdlc::Frame& response) {
    const auto status = parseStatus(response, Command::getDeviceData);
    if (response.information.size() < 4) {
        return std::nullopt;
    }
    DataValue result;
    result.status = status;
    result.value.assign(response.information.begin() + 4, response.information.end());
    return result;
}

std::optional<AlarmData> parseAlarms(const hdlc::Frame& response) {
    const auto status = parseStatus(response, Command::getAlarms);
    if (response.information.size() < 4) {
        return std::nullopt;
    }
    AlarmData result;
    result.status = status;
    for (std::size_t index = 3; index < response.information.size(); ++index) {
        const auto code = response.information[index];
        if (code == 0) {
            continue;
        }
        result.alarms.push_back({code,
                                 code >= 0x20 ? AlarmSeverity::critical : AlarmSeverity::warning,
                                 "AISG alarm 0x" + hdlc::toHex(std::span(&code, 1)),
                                 true,
                                 std::chrono::system_clock::now()});
    }
    return result;
}

} // namespace atc::aisg
