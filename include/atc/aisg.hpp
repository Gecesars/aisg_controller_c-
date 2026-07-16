#pragma once

#include "atc/domain.hpp"
#include "atc/hdlc.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace atc::aisg {

inline constexpr std::uint8_t allStationsAddress = 0xFF;
inline constexpr std::uint8_t noStationAddress = 0x00;
inline constexpr std::uint8_t xidControl = 0xBF;

struct XidIdentity {
    hdlc::Bytes uniqueId;
    std::uint8_t deviceType{};
    std::array<char, 2> vendorCode{};

    friend bool operator==(const XidIdentity&, const XidIdentity&) = default;
};

// These endpoint identifiers are inferred from captured AISG 2.0 traffic.
// They remain experimental until checked against licensed specifications and
// real hardware from each vendor.
enum class Command : std::uint8_t {
    getAlarms = 0x04,
    initialData = 0x05,
    clearAlarms = 0x06,
    selfTest = 0x0A,
    setDeviceData = 0x0E,
    getDeviceData = 0x0F,
    subscribeAlarms = 0x12,
    calibrate = 0x31,
    setTilt = 0x33,
    getTilt = 0x34,
    setTmaMode = 0x70,
    getTmaMode = 0x71,
    setTmaGain = 0x72,
    getTmaGain = 0x73,
    antennaCount = 0x88,
};

enum class Field : std::uint8_t {
    antennaModel = 0x01,
    uniqueId = 0x02,
    operatingBands = 0x03,
    beamwidth = 0x04,
    gain = 0x05,
    maximumTilt = 0x06,
    minimumTilt = 0x07,
    installationDate = 0x21,
    installerId = 0x22,
    baseStationId = 0x23,
    sectorId = 0x24,
    bearing = 0x25,
    mechanicalTilt = 0x26,
};

class ControlSequence {
public:
    [[nodiscard]] std::uint8_t nextRequest() noexcept;
    [[nodiscard]] std::uint8_t nextKeepAlive() noexcept;
    void reset() noexcept;

private:
    std::size_t requestIndex_{};
    std::size_t keepAliveIndex_{};
};

[[nodiscard]] std::uint8_t expectedResponseControl(std::uint8_t requestControl) noexcept;
[[nodiscard]] bool isResponseFor(const hdlc::Frame& response, const hdlc::Frame& request) noexcept;

[[nodiscard]] hdlc::Frame makeDeviceScanRequest(
    std::span<const std::uint8_t> uniqueId = {},
    std::span<const std::uint8_t> uniqueIdMask = {});
[[nodiscard]] std::optional<XidIdentity> parseDeviceScanResponse(const hdlc::Frame& frame);
[[nodiscard]] hdlc::Frame makeAddressAssignmentRequest(const XidIdentity& identity,
                                                       std::uint8_t address);
[[nodiscard]] bool isAddressAssignmentResponse(const hdlc::Frame& frame,
                                               const XidIdentity& identity,
                                               std::uint8_t address);
[[nodiscard]] hdlc::Frame makeReleaseNegotiationRequest(std::uint8_t address,
                                                        std::uint8_t release = 14);
[[nodiscard]] bool isReleaseNegotiationResponse(const hdlc::Frame& frame,
                                                std::uint8_t address);
[[nodiscard]] hdlc::Frame makeResetDeviceRequest(
    std::uint8_t address = allStationsAddress);

[[nodiscard]] hdlc::Frame makeSnrm(std::uint8_t address);
[[nodiscard]] hdlc::Frame makeKeepAlive(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeInitialDataRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeGetTiltRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeSetTiltRequest(std::uint8_t address, std::uint8_t control, double degrees);
[[nodiscard]] hdlc::Frame makeGetAlarmsRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeClearAlarmsRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeSubscribeAlarmsRequest(std::uint8_t address,
                                                     std::uint8_t control);
[[nodiscard]] hdlc::Frame makeSelfTestRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeCalibrateRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeSetTmaModeRequest(std::uint8_t address,
                                                std::uint8_t control,
                                                TmaMode mode);
[[nodiscard]] hdlc::Frame makeGetTmaModeRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeSetTmaGainRequest(std::uint8_t address,
                                                std::uint8_t control,
                                                double gainDb);
[[nodiscard]] hdlc::Frame makeGetTmaGainRequest(std::uint8_t address, std::uint8_t control);
[[nodiscard]] hdlc::Frame makeGetDataRequest(std::uint8_t address, std::uint8_t control, Field field);
[[nodiscard]] hdlc::Frame makeSetDataRequest(std::uint8_t address,
                                             std::uint8_t control,
                                             Field field,
                                             std::span<const std::uint8_t> data);

struct ProtocolStatus {
    bool success{};
    std::uint8_t code{};
    std::string message;
};

struct InitialData {
    ProtocolStatus status;
    std::string product;
    std::string serialNumber;
    std::string hardwareVersion;
    std::string softwareVersion;
};

struct DataValue {
    ProtocolStatus status;
    hdlc::Bytes value;
};

struct AlarmData {
    ProtocolStatus status;
    std::vector<Alarm> alarms;
};

[[nodiscard]] std::optional<InitialData> parseInitialData(const hdlc::Frame& response);
[[nodiscard]] std::optional<double> parseTilt(const hdlc::Frame& response);
[[nodiscard]] std::optional<TmaMode> parseTmaMode(const hdlc::Frame& response);
[[nodiscard]] std::optional<double> parseTmaGain(const hdlc::Frame& response);
[[nodiscard]] std::optional<DataValue> parseData(const hdlc::Frame& response);
[[nodiscard]] std::optional<AlarmData> parseAlarms(const hdlc::Frame& response);
[[nodiscard]] ProtocolStatus parseStatus(const hdlc::Frame& response, Command expectedCommand);

} // namespace atc::aisg
