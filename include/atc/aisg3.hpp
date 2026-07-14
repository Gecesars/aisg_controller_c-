#pragma once

#include "atc/domain.hpp"
#include "atc/hdlc.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace atc::aisg3 {

inline constexpr std::uint8_t allStationsAddress = 0xFF;
inline constexpr std::uint8_t noStationAddress = 0x00;
inline constexpr std::uint8_t xidControl = 0xBF;
inline constexpr std::uint8_t xidFormatIdentifier = 0x81;
inline constexpr std::uint8_t xidGroupIdentifier = 0xF0;
inline constexpr std::size_t maximumLayer2Payload = 264;
inline constexpr std::size_t maximumLayer7Data = 256;

struct Version {
    std::uint8_t release{};
    std::uint8_t major{};
    std::uint8_t minor{};

    friend bool operator==(const Version&, const Version&) = default;
};

inline constexpr Version supportedBaseVersion{3, 0, 8};
inline constexpr Version supportedAdbVersion{3, 1, 7};

// AISG Base 7.1/8.10: left-most 8 hexadecimal digits of SHA-1(node name),
// interpreted as a four-octet integer; zero is replaced by one.
[[nodiscard]] std::uint32_t primaryIdForNodeName(std::string_view nodeName);

enum class AldType : std::uint8_t {
    sald = 64,
    mald = 65,
};

enum class SubunitType : std::uint8_t {
    ret = 0x01,
    tma = 0x02,
    adb = 0x03,
    als = 0x04,
};

enum class Provenance : std::uint8_t {
    notSet = 0,
    factory = 1,
    file = 2,
    automatic = 3,
    manual = 4,
};

enum class Command : std::uint16_t {
    getAlarmStatus = 0x0004,
    getInformation = 0x0005,
    clearActiveAlarms = 0x0006,
    alarmIndication = 0x0007,
    getSubunitList = 0x0008,
    getAldResetCause = 0x0009,
    getDiagnosticInformation = 0x000B,
    setSubunitTypeStandardVersion = 0x000C,
    getSubunitTypeStandardVersions = 0x000D,
    aldSetInstallationInfo = 0x0010,
    aldGetInstallationInfo = 0x0011,
    alarmSubscribe = 0x0012,
    getNumberOfPorts = 0x001E,
    getPortInfo = 0x001F,
    getPortInterconnections = 0x0020,
    getRfPortFrequencyInfo = 0x0025,
    getConnectorPlateMarkingInfo = 0x0029,
    getAldConfigurationChecksum = 0x002B,
    setAldCurrentTime = 0x002E,
    getAldCurrentTime = 0x002F,
    adbGetAntennaInfo = 0x0300,
    adbGetAntennaPortInfo = 0x0301,
    adbGetAntennaArrayElementInfo = 0x0302,
    adbSetAntennaInstallationInfo = 0x0303,
    adbGetAntennaInstallationInfo = 0x0304,
    adbSetRfPathIdToArrayElement = 0x0305,
    adbGetRfPathIdOfArrayElement = 0x0306,
};

enum class ReturnCode : std::uint16_t {
    ok = 0x0000,
    busy = 0x0005,
    generalError = 0x0011,
    outOfRange = 0x0013,
    inUseByAnotherPrimary = 0x0023,
    notAuthorised = 0x002C,
    invalidSubunitNumber = 0x002D,
    invalidPortNumber = 0x002E,
    formatError = 0x0024,
    unknownCommand = 0x0019,
    invalidSubunitType = 0x003E,
    incorrectState = 0x0040,
    tooManyArguments = 0x0043,
    invalidArrayElementNumber = 0x0047,
    invalidProvenance = 0x0051,
    unsupportedProtocolVersion = 0x0054,
    subunitTypeNotAccessible = 0x0058,
    protocolVersionNotNegotiated = 0x0059,
    rfPathIdsNotInitialised = 0x005A,
    invalidPrimaryId = 0x005B,
    notAControlPort = 0x005C,
    adbNotAntennaPort = 0x0300,
};

struct XidParameter {
    std::uint8_t identifier{};
    hdlc::Bytes value;

    friend bool operator==(const XidParameter&, const XidParameter&) = default;
};

struct XidPayload {
    std::vector<XidParameter> parameters;
};

struct DeviceScanPattern {
    hdlc::Bytes uniqueId;
    hdlc::Bytes uniqueIdMask;
    hdlc::Bytes portNumber;
    hdlc::Bytes portNumberMask;
};

struct DeviceScanResponse {
    std::array<std::uint8_t, 19> uniqueId{};
    AldType aldType{AldType::sald};
    std::array<char, 2> vendorCode{};
    std::uint16_t portNumber{};
    std::vector<Version> baseVersions;
    std::vector<SubunitType> subunitTypes;
};

struct AddressAssignment {
    std::uint8_t address{};
    Version baseVersion{supportedBaseVersion};
    std::uint32_t primaryId{};
    std::array<std::uint8_t, 19> uniqueId{};
    AldType aldType{AldType::sald};
    std::array<char, 2> vendorCode{};
    std::uint16_t portNumber{};
};

struct AddressAssignmentResponse {
    std::uint8_t address{};
    std::array<std::uint8_t, 19> uniqueId{};
    AldType aldType{AldType::sald};
    std::uint16_t portNumber{};
};

[[nodiscard]] hdlc::Bytes encodeXid(const XidPayload& payload);
[[nodiscard]] std::optional<XidPayload> parseXid(std::span<const std::uint8_t> information);
[[nodiscard]] hdlc::Frame makeDeviceScanCommand(const DeviceScanPattern& pattern = {});
[[nodiscard]] std::optional<DeviceScanResponse> parseDeviceScanResponse(const hdlc::Frame& frame);
[[nodiscard]] hdlc::Frame makeAddressAssignmentCommand(const AddressAssignment& assignment);
[[nodiscard]] std::optional<AddressAssignmentResponse> parseAddressAssignmentResponse(
    const hdlc::Frame& frame);
[[nodiscard]] hdlc::Frame makeResetPortCommand(std::uint8_t address = allStationsAddress);
[[nodiscard]] bool isResetPortResponse(const hdlc::Frame& frame, std::uint8_t expectedAddress);

struct PrimaryCommand {
    Command command{Command::getInformation};
    std::uint16_t sequence{};
    std::uint16_t subunit{};
    hdlc::Bytes data;
};

struct Response {
    Command command{Command::getInformation};
    std::uint16_t sequence{};
    ReturnCode returnCode{ReturnCode::ok};
    hdlc::Bytes data;
    std::optional<std::uint8_t> aldState;
    std::optional<std::uint8_t> connectionState;

    [[nodiscard]] bool success() const noexcept { return returnCode == ReturnCode::ok; }
};

struct Information {
    std::string productNumber;
    std::string serialNumber;
    std::string hardwareVersion;
    std::string softwareVersion;
};

struct Subunit {
    std::uint16_t number{};
    SubunitType type{SubunitType::ret};
};

struct SubunitVersions {
    Version negotiated;
    std::vector<Version> supported;
};

struct AlarmState {
    std::uint16_t code{};
    std::uint8_t severity{};
};

[[nodiscard]] hdlc::Bytes encodeCommand(const PrimaryCommand& command);
[[nodiscard]] hdlc::Frame makeCommandFrame(std::uint8_t address,
                                           std::uint8_t control,
                                           const PrimaryCommand& command);
[[nodiscard]] std::optional<Response> parseResponse(const hdlc::Frame& frame,
                                                    Command expectedCommand,
                                                    std::uint16_t expectedSequence);
[[nodiscard]] bool isResponseFor(const hdlc::Frame& frame,
                                 const hdlc::Frame& request,
                                 Command command,
                                 std::uint16_t sequence);

[[nodiscard]] PrimaryCommand makeGetInformation(std::uint16_t sequence);
[[nodiscard]] std::optional<Information> parseInformation(const Response& response);
[[nodiscard]] PrimaryCommand makeGetSubunitList(std::uint16_t sequence);
[[nodiscard]] std::optional<std::vector<Subunit>> parseSubunitList(const Response& response);
[[nodiscard]] PrimaryCommand makeGetSubunitVersions(std::uint16_t sequence, SubunitType type);
[[nodiscard]] std::optional<SubunitVersions> parseSubunitVersions(const Response& response);
[[nodiscard]] PrimaryCommand makeSetSubunitVersion(std::uint16_t sequence,
                                                   SubunitType type,
                                                   Version version);
[[nodiscard]] PrimaryCommand makeGetAlarmStatus(std::uint16_t sequence, std::uint16_t subunit);
[[nodiscard]] std::optional<std::vector<AlarmState>> parseAlarmStatus(const Response& response);
[[nodiscard]] PrimaryCommand makeClearActiveAlarms(std::uint16_t sequence,
                                                   std::uint16_t subunit);

[[nodiscard]] std::string uniqueIdString(const std::array<std::uint8_t, 19>& uniqueId);
[[nodiscard]] const char* returnCodeMessage(ReturnCode code) noexcept;

} // namespace atc::aisg3
