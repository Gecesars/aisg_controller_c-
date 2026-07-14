#include "atc/aisg.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace atc::aisg {
namespace {

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
    case 0x01: return "command rejected";
    case 0x02: return "busy";
    case 0x03: return "invalid data";
    default: return "device returned status " + std::to_string(code);
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
                        {0x03, 0x00, 0x00,
                         static_cast<std::uint8_t>(deciDegrees & 0xFFU),
                         static_cast<std::uint8_t>((deciDegrees >> 8U) & 0xFFU)});
}

hdlc::Frame makeGetAlarmsRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::getAlarms);
}

hdlc::Frame makeClearAlarmsRequest(const std::uint8_t address, const std::uint8_t control) {
    return commandFrame(address, control, Command::clearAlarms);
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
    return {code == 0, code, statusMessage(code)};
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
