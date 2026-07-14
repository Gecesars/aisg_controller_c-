#include "atc/aisg3.hpp"

#include <algorithm>
#include <bit>
#include <limits>
#include <stdexcept>

namespace atc::aisg3 {
namespace {

void appendLe16(hdlc::Bytes& target, const std::uint16_t value) {
    target.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void appendBe16(hdlc::Bytes& target, const std::uint16_t value) {
    target.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    target.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void appendBe32(hdlc::Bytes& target, const std::uint32_t value) {
    target.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    target.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

std::optional<std::uint16_t> readLe16(const std::span<const std::uint8_t> data,
                                      const std::size_t offset) {
    if (offset > data.size() || data.size() - offset < 2) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(data[offset]) |
           (static_cast<std::uint16_t>(data[offset + 1]) << 8U);
}

std::optional<std::uint16_t> readBe16(const std::span<const std::uint8_t> data) {
    if (data.empty() || data.size() > 2) {
        return std::nullopt;
    }
    std::uint16_t result{};
    for (const auto value : data) {
        result = static_cast<std::uint16_t>((result << 8U) | value);
    }
    return result;
}

bool validAldType(const std::uint8_t value) {
    return value == static_cast<std::uint8_t>(AldType::sald) ||
           value == static_cast<std::uint8_t>(AldType::mald);
}

bool validSubunitType(const std::uint8_t value) {
    return value >= static_cast<std::uint8_t>(SubunitType::ret) &&
           value <= static_cast<std::uint8_t>(SubunitType::als);
}

bool validUtf8(const std::string_view value) {
    const auto* data = reinterpret_cast<const unsigned char*>(value.data());
    std::size_t index{};
    while (index < value.size()) {
        const auto first = data[index++];
        if (first <= 0x7FU) continue;
        std::size_t continuation{};
        std::uint32_t codepoint{};
        if (first >= 0xC2U && first <= 0xDFU) {
            continuation = 1;
            codepoint = first & 0x1FU;
        } else if (first >= 0xE0U && first <= 0xEFU) {
            continuation = 2;
            codepoint = first & 0x0FU;
        } else if (first >= 0xF0U && first <= 0xF4U) {
            continuation = 3;
            codepoint = first & 0x07U;
        } else {
            return false;
        }
        if (continuation > value.size() - index) return false;
        for (std::size_t count = 0; count < continuation; ++count) {
            const auto next = data[index++];
            if ((next & 0xC0U) != 0x80U) return false;
            codepoint = (codepoint << 6U) | (next & 0x3FU);
        }
        if ((continuation == 2 && codepoint < 0x800U) ||
            (continuation == 3 && codepoint < 0x10000U) ||
            (codepoint >= 0xD800U && codepoint <= 0xDFFFU) || codepoint > 0x10FFFFU) {
            return false;
        }
    }
    return true;
}

bool validUniqueId(const std::span<const std::uint8_t> uniqueId) {
    if (uniqueId.size() != 19 || uniqueId[0] < 0x21U || uniqueId[0] > 0x7EU ||
        uniqueId[1] < 0x21U || uniqueId[1] > 0x7EU) {
        return false;
    }
    bool unitCodeStarted = false;
    for (std::size_t index = 2; index < uniqueId.size(); ++index) {
        const auto value = uniqueId[index];
        if (value == 0) {
            if (unitCodeStarted) return false;
            continue;
        }
        if (value < 0x21U || value > 0x7EU) return false;
        unitCodeStarted = true;
    }
    return unitCodeStarted;
}

const XidParameter* findAfter(const XidPayload& payload,
                              const std::uint8_t identifier,
                              std::size_t& cursor) {
    while (cursor < payload.parameters.size()) {
        const auto& parameter = payload.parameters[cursor++];
        if (parameter.identifier == identifier) {
            return &parameter;
        }
    }
    return nullptr;
}

void validateScanPattern(const DeviceScanPattern& pattern) {
    if (pattern.uniqueId.size() != pattern.uniqueIdMask.size() || pattern.uniqueId.size() > 19) {
        throw std::invalid_argument("AISG v3 UniqueID pattern and mask must have equal lengths up to 19");
    }
    if (pattern.portNumber.size() != pattern.portNumberMask.size() ||
        pattern.portNumber.size() > 2) {
        throw std::invalid_argument("AISG v3 port pattern and mask must have equal lengths up to 2");
    }
}

hdlc::Bytes versionBytes(const Version version) {
    return {version.release, version.major, version.minor};
}

std::optional<std::string> readString(const hdlc::Bytes& data,
                                      std::size_t& offset,
                                      const std::size_t maximum = 255) {
    if (offset >= data.size()) {
        return std::nullopt;
    }
    const auto length = static_cast<std::size_t>(data[offset++]);
    if (length > maximum || length > data.size() - offset) {
        return std::nullopt;
    }
    std::string result(data.begin() + static_cast<std::ptrdiff_t>(offset),
                       data.begin() + static_cast<std::ptrdiff_t>(offset + length));
    offset += length;
    return validUtf8(result) ? std::optional<std::string>(std::move(result)) : std::nullopt;
}

} // namespace

std::uint32_t primaryIdForNodeName(const std::string_view nodeName) {
    if (nodeName.empty()) {
        throw std::invalid_argument("AISG primary node name must not be empty");
    }
    if (!validUtf8(nodeName)) {
        throw std::invalid_argument("AISG primary node name must be valid UTF-8");
    }

    hdlc::Bytes message(nodeName.begin(), nodeName.end());
    const auto bitLength = static_cast<std::uint64_t>(message.size()) * 8U;
    message.push_back(0x80U);
    while (message.size() % 64U != 56U) message.push_back(0);
    for (int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<std::uint8_t>((bitLength >> shift) & 0xFFU));
    }

    std::uint32_t h0 = 0x67452301U;
    std::uint32_t h1 = 0xEFCDAB89U;
    std::uint32_t h2 = 0x98BADCFEU;
    std::uint32_t h3 = 0x10325476U;
    std::uint32_t h4 = 0xC3D2E1F0U;
    for (std::size_t block = 0; block < message.size(); block += 64U) {
        std::array<std::uint32_t, 80> words{};
        for (std::size_t index = 0; index < 16; ++index) {
            const auto offset = block + index * 4U;
            words[index] = (static_cast<std::uint32_t>(message[offset]) << 24U) |
                           (static_cast<std::uint32_t>(message[offset + 1]) << 16U) |
                           (static_cast<std::uint32_t>(message[offset + 2]) << 8U) |
                           static_cast<std::uint32_t>(message[offset + 3]);
        }
        for (std::size_t index = 16; index < words.size(); ++index) {
            words[index] = std::rotl(words[index - 3] ^ words[index - 8] ^
                                     words[index - 14] ^ words[index - 16], 1);
        }

        auto a = h0;
        auto b = h1;
        auto c = h2;
        auto d = h3;
        auto e = h4;
        for (std::size_t index = 0; index < words.size(); ++index) {
            std::uint32_t function{};
            std::uint32_t constant{};
            if (index < 20) {
                function = (b & c) | ((~b) & d);
                constant = 0x5A827999U;
            } else if (index < 40) {
                function = b ^ c ^ d;
                constant = 0x6ED9EBA1U;
            } else if (index < 60) {
                function = (b & c) | (b & d) | (c & d);
                constant = 0x8F1BBCDCU;
            } else {
                function = b ^ c ^ d;
                constant = 0xCA62C1D6U;
            }
            const auto temporary = std::rotl(a, 5) + function + e + constant + words[index];
            e = d;
            d = c;
            c = std::rotl(b, 30);
            b = a;
            a = temporary;
        }
        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }
    return h0 == 0 ? 1U : h0;
}

hdlc::Bytes encodeXid(const XidPayload& payload) {
    hdlc::Bytes parameters;
    for (const auto& parameter : payload.parameters) {
        if (parameter.value.size() > std::numeric_limits<std::uint8_t>::max()) {
            throw std::invalid_argument("AISG XID parameter exceeds 255 octets");
        }
        parameters.push_back(parameter.identifier);
        parameters.push_back(static_cast<std::uint8_t>(parameter.value.size()));
        parameters.insert(parameters.end(), parameter.value.begin(), parameter.value.end());
    }
    if (parameters.size() > std::numeric_limits<std::uint8_t>::max()) {
        throw std::invalid_argument("AISG XID group exceeds 255 octets");
    }
    hdlc::Bytes result{xidFormatIdentifier, xidGroupIdentifier,
                       static_cast<std::uint8_t>(parameters.size())};
    result.insert(result.end(), parameters.begin(), parameters.end());
    return result;
}

std::optional<XidPayload> parseXid(const std::span<const std::uint8_t> information) {
    if (information.size() < 3 || information[0] != xidFormatIdentifier ||
        information[1] != xidGroupIdentifier ||
        static_cast<std::size_t>(information[2]) != information.size() - 3) {
        return std::nullopt;
    }
    XidPayload result;
    std::size_t offset = 3;
    while (offset < information.size()) {
        if (information.size() - offset < 2) {
            return std::nullopt;
        }
        const auto identifier = information[offset++];
        const auto length = static_cast<std::size_t>(information[offset++]);
        if (length > information.size() - offset) {
            return std::nullopt;
        }
        result.parameters.push_back({identifier,
            hdlc::Bytes(information.begin() + static_cast<std::ptrdiff_t>(offset),
                        information.begin() + static_cast<std::ptrdiff_t>(offset + length))});
        offset += length;
    }
    return result;
}

hdlc::Frame makeDeviceScanCommand(const DeviceScanPattern& pattern) {
    validateScanPattern(pattern);
    XidPayload payload{{
        {1, pattern.uniqueId},
        {8, pattern.uniqueIdMask},
        {10, pattern.portNumber},
        {11, pattern.portNumberMask},
        {19, {1}},
    }};
    return {allStationsAddress, xidControl, encodeXid(payload)};
}

std::optional<DeviceScanResponse> parseDeviceScanResponse(const hdlc::Frame& frame) {
    if (frame.address != noStationAddress || frame.control != xidControl) {
        return std::nullopt;
    }
    const auto payload = parseXid(frame.information);
    if (!payload) {
        return std::nullopt;
    }
    std::size_t cursor{};
    const auto* uid = findAfter(*payload, 1, cursor);
    const auto* type = findAfter(*payload, 4, cursor);
    const auto* vendor = findAfter(*payload, 6, cursor);
    const auto* port = findAfter(*payload, 10, cursor);
    const auto* versions = findAfter(*payload, 22, cursor);
    const auto* subunits = findAfter(*payload, 27, cursor);
    if (!uid || !type || !vendor || !port || !versions || !subunits ||
        !validUniqueId(uid->value) || type->value.size() != 1 ||
        !validAldType(type->value[0]) || vendor->value.size() != 2 ||
        vendor->value[0] != uid->value[0] || vendor->value[1] != uid->value[1] ||
        port->value.empty() || port->value.size() > 2 || versions->value.empty() ||
        versions->value.size() % 3 != 0 || subunits->value.empty()) {
        return std::nullopt;
    }
    const auto portNumber = readBe16(port->value);
    if (!portNumber || *portNumber == 0) {
        return std::nullopt;
    }

    DeviceScanResponse result;
    std::copy(uid->value.begin(), uid->value.end(), result.uniqueId.begin());
    result.aldType = static_cast<AldType>(type->value[0]);
    result.vendorCode = {static_cast<char>(vendor->value[0]), static_cast<char>(vendor->value[1])};
    result.portNumber = *portNumber;
    for (std::size_t index = 0; index < versions->value.size(); index += 3) {
        result.baseVersions.push_back(
            {versions->value[index], versions->value[index + 1], versions->value[index + 2]});
    }
    for (const auto value : subunits->value) {
        if (!validSubunitType(value)) {
            return std::nullopt;
        }
        result.subunitTypes.push_back(static_cast<SubunitType>(value));
    }
    return result;
}

hdlc::Frame makeAddressAssignmentCommand(const AddressAssignment& assignment) {
    if (assignment.address == noStationAddress || assignment.address == allStationsAddress ||
        assignment.primaryId == 0 || assignment.portNumber == 0 ||
        !validUniqueId(assignment.uniqueId) ||
        static_cast<std::uint8_t>(assignment.vendorCode[0]) != assignment.uniqueId[0] ||
        static_cast<std::uint8_t>(assignment.vendorCode[1]) != assignment.uniqueId[1]) {
        throw std::invalid_argument("AISG v3 address assignment contains a reserved value");
    }
    hdlc::Bytes address{assignment.address};
    hdlc::Bytes primaryId;
    appendBe32(primaryId, assignment.primaryId);
    hdlc::Bytes uniqueId(assignment.uniqueId.begin(), assignment.uniqueId.end());
    hdlc::Bytes vendor{static_cast<std::uint8_t>(assignment.vendorCode[0]),
                       static_cast<std::uint8_t>(assignment.vendorCode[1])};
    hdlc::Bytes port;
    appendBe16(port, assignment.portNumber);
    XidPayload payload{{
        {2, std::move(address)},
        {22, versionBytes(assignment.baseVersion)},
        {26, std::move(primaryId)},
        {1, std::move(uniqueId)},
        {4, {static_cast<std::uint8_t>(assignment.aldType)}},
        {6, std::move(vendor)},
        {10, std::move(port)},
    }};
    return {allStationsAddress, xidControl, encodeXid(payload)};
}

std::optional<AddressAssignmentResponse> parseAddressAssignmentResponse(const hdlc::Frame& frame) {
    if (frame.address == noStationAddress || frame.address == allStationsAddress ||
        frame.control != xidControl) {
        return std::nullopt;
    }
    const auto payload = parseXid(frame.information);
    if (!payload) {
        return std::nullopt;
    }
    std::size_t cursor{};
    const auto* uid = findAfter(*payload, 1, cursor);
    const auto* type = findAfter(*payload, 4, cursor);
    const auto* port = findAfter(*payload, 10, cursor);
    if (!uid || !type || !port || !validUniqueId(uid->value) || type->value.size() != 1 ||
        !validAldType(type->value[0])) {
        return std::nullopt;
    }
    const auto portNumber = readBe16(port->value);
    if (!portNumber || *portNumber == 0) {
        return std::nullopt;
    }
    AddressAssignmentResponse result;
    result.address = frame.address;
    std::copy(uid->value.begin(), uid->value.end(), result.uniqueId.begin());
    result.aldType = static_cast<AldType>(type->value[0]);
    result.portNumber = *portNumber;
    return result;
}

hdlc::Frame makeResetPortCommand(const std::uint8_t address) {
    if (address == noStationAddress) {
        throw std::invalid_argument("AISG reset-port cannot target the no-station address");
    }
    return {address, xidControl, encodeXid({{{7, {}}}})};
}

bool isResetPortResponse(const hdlc::Frame& frame, const std::uint8_t expectedAddress) {
    if (frame.address != expectedAddress || frame.control != xidControl) {
        return false;
    }
    const auto payload = parseXid(frame.information);
    return payload && payload->parameters.size() == 1 &&
           payload->parameters[0].identifier == 7 && payload->parameters[0].value.empty();
}

hdlc::Bytes encodeCommand(const PrimaryCommand& command) {
    if (command.data.size() > maximumLayer7Data) {
        throw std::invalid_argument("AISG v3 layer-7 data exceeds 256 octets");
    }
    hdlc::Bytes result;
    result.reserve(8 + command.data.size());
    appendLe16(result, static_cast<std::uint16_t>(command.command));
    appendLe16(result, command.sequence);
    appendLe16(result, command.subunit);
    appendLe16(result, static_cast<std::uint16_t>(command.data.size()));
    result.insert(result.end(), command.data.begin(), command.data.end());
    return result;
}

hdlc::Frame makeCommandFrame(const std::uint8_t address,
                             const std::uint8_t control,
                             const PrimaryCommand& command) {
    if (address == noStationAddress || address == allStationsAddress) {
        throw std::invalid_argument("AISG v3 layer-7 command needs an assigned ALD address");
    }
    return {address, control, encodeCommand(command)};
}

std::optional<Response> parseResponse(const hdlc::Frame& frame,
                                      const Command expectedCommand,
                                      const std::uint16_t expectedSequence) {
    const auto& information = frame.information;
    if (information.size() < 8 || information.size() > maximumLayer2Payload) {
        return std::nullopt;
    }
    const auto command = readLe16(information, 0);
    const auto sequence = readLe16(information, 2);
    const auto returnCode = readLe16(information, 4);
    const auto length = readLe16(information, 6);
    if (!command || !sequence || !returnCode || !length ||
        *command != static_cast<std::uint16_t>(expectedCommand) ||
        *sequence != expectedSequence || static_cast<std::size_t>(*length) != information.size() - 8) {
        return std::nullopt;
    }
    Response result;
    result.command = expectedCommand;
    result.sequence = *sequence;
    result.returnCode = static_cast<ReturnCode>(*returnCode);
    result.data.assign(information.begin() + 8, information.end());
    if (!result.success() && result.data.size() >= 2) {
        result.aldState = result.data[0];
        result.connectionState = result.data[1];
    }
    return result;
}

bool isResponseFor(const hdlc::Frame& frame,
                   const hdlc::Frame& request,
                   const Command command,
                   const std::uint16_t sequence) {
    if (frame.address != request.address ||
        frame.control != static_cast<std::uint8_t>(request.control + 0x20U)) {
        return false;
    }
    return parseResponse(frame, command, sequence).has_value();
}

PrimaryCommand makeGetInformation(const std::uint16_t sequence) {
    return {Command::getInformation, sequence, 0, {}};
}

std::optional<Information> parseInformation(const Response& response) {
    if (!response.success() || response.command != Command::getInformation) {
        return std::nullopt;
    }
    std::size_t offset{};
    const auto product = readString(response.data, offset);
    const auto serial = readString(response.data, offset);
    const auto hardware = readString(response.data, offset);
    const auto software = readString(response.data, offset);
    if (!product || !serial || !hardware || !software || offset != response.data.size()) {
        return std::nullopt;
    }
    return Information{*product, *serial, *hardware, *software};
}

PrimaryCommand makeGetSubunitList(const std::uint16_t sequence) {
    return {Command::getSubunitList, sequence, 0, {}};
}

std::optional<std::vector<Subunit>> parseSubunitList(const Response& response) {
    if (!response.success() || response.command != Command::getSubunitList ||
        response.data.size() < 2) {
        return std::nullopt;
    }
    const auto count = readLe16(response.data, 0);
    if (!count || response.data.size() != 2 + static_cast<std::size_t>(*count) * 3) {
        return std::nullopt;
    }
    std::vector<Subunit> result;
    result.reserve(*count);
    std::size_t offset = 2;
    for (std::size_t index = 0; index < *count; ++index) {
        const auto number = readLe16(response.data, offset);
        if (!number || *number == 0 || !validSubunitType(response.data[offset + 2])) {
            return std::nullopt;
        }
        result.push_back({*number, static_cast<SubunitType>(response.data[offset + 2])});
        offset += 3;
    }
    return result;
}

PrimaryCommand makeGetSubunitVersions(const std::uint16_t sequence, const SubunitType type) {
    return {Command::getSubunitTypeStandardVersions, sequence, 0,
            {static_cast<std::uint8_t>(type)}};
}

std::optional<SubunitVersions> parseSubunitVersions(const Response& response) {
    if (!response.success() || response.command != Command::getSubunitTypeStandardVersions ||
        response.data.size() < 4) {
        return std::nullopt;
    }
    const auto count = static_cast<std::size_t>(response.data[3]);
    if (response.data.size() != 4 + count * 3) {
        return std::nullopt;
    }
    SubunitVersions result{{response.data[0], response.data[1], response.data[2]}, {}};
    result.supported.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto offset = 4 + index * 3;
        result.supported.push_back(
            {response.data[offset], response.data[offset + 1], response.data[offset + 2]});
    }
    return result;
}

PrimaryCommand makeSetSubunitVersion(const std::uint16_t sequence,
                                     const SubunitType type,
                                     const Version version) {
    return {Command::setSubunitTypeStandardVersion, sequence, 0,
            {static_cast<std::uint8_t>(type), version.release, version.major, version.minor}};
}

PrimaryCommand makeGetAlarmStatus(const std::uint16_t sequence, const std::uint16_t subunit) {
    return {Command::getAlarmStatus, sequence, subunit, {}};
}

std::optional<std::vector<AlarmState>> parseAlarmStatus(const Response& response) {
    if (!response.success() || response.command != Command::getAlarmStatus || response.data.empty()) {
        return std::nullopt;
    }
    const auto count = static_cast<std::size_t>(response.data[0]);
    if (response.data.size() != 1 + count * 3) {
        return std::nullopt;
    }
    std::vector<AlarmState> result;
    result.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto offset = 1 + index * 3;
        const auto code = readLe16(response.data, offset);
        if (!code || response.data[offset + 2] == 0 || response.data[offset + 2] > 4) {
            return std::nullopt;
        }
        result.push_back({*code, response.data[offset + 2]});
    }
    return result;
}

PrimaryCommand makeClearActiveAlarms(const std::uint16_t sequence,
                                     const std::uint16_t subunit) {
    return {Command::clearActiveAlarms, sequence, subunit, {}};
}

std::string uniqueIdString(const std::array<std::uint8_t, 19>& uniqueId) {
    std::string result;
    result.reserve(uniqueId.size());
    for (const auto value : uniqueId) {
        if (value != 0) {
            result.push_back(static_cast<char>(value));
        }
    }
    return result;
}

const char* returnCodeMessage(const ReturnCode code) noexcept {
    switch (code) {
    case ReturnCode::ok: return "OK";
    case ReturnCode::busy: return "Busy";
    case ReturnCode::generalError: return "GeneralError";
    case ReturnCode::outOfRange: return "OutOfRange";
    case ReturnCode::inUseByAnotherPrimary: return "InUseByAnotherPrimary";
    case ReturnCode::notAuthorised: return "NotAuthorised";
    case ReturnCode::invalidSubunitNumber: return "InvalidSubunitNumber";
    case ReturnCode::invalidPortNumber: return "InvalidPortNumber";
    case ReturnCode::formatError: return "FormatError";
    case ReturnCode::unknownCommand: return "UnknownCommand";
    case ReturnCode::invalidSubunitType: return "InvalidSubunitType";
    case ReturnCode::incorrectState: return "IncorrectState";
    case ReturnCode::tooManyArguments: return "TooManyArguments";
    case ReturnCode::invalidArrayElementNumber: return "InvalidArrayElementNumber";
    case ReturnCode::invalidProvenance: return "InvalidProvenance";
    case ReturnCode::unsupportedProtocolVersion: return "UnsupportedProtocolVersion";
    case ReturnCode::subunitTypeNotAccessible: return "SubunitTypeNotAccessible";
    case ReturnCode::protocolVersionNotNegotiated: return "ProtocolVersionNotNegotiated";
    case ReturnCode::rfPathIdsNotInitialised: return "RFPathIDsNotInitialised";
    case ReturnCode::invalidPrimaryId: return "InvalidPrimaryID";
    case ReturnCode::notAControlPort: return "NotAControlPort";
    case ReturnCode::adbNotAntennaPort: return "ADBNotAntennaPort";
    }
    return "UnknownReturnCode";
}

} // namespace atc::aisg3
