#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace atc {

enum class DeviceKind { unknown, ret, tma };
enum class DeviceStatus { disconnected, discovered, initializing, ready, busy, alarm, fault };
enum class AlarmSeverity { information, warning, critical };

struct Alarm {
    std::uint16_t code{};
    AlarmSeverity severity{AlarmSeverity::warning};
    std::string description;
    bool active{true};
    std::chrono::system_clock::time_point occurredAt{std::chrono::system_clock::now()};

    friend bool operator==(const Alarm&, const Alarm&) = default;
};

struct InstallationData {
    std::string antennaModel;
    std::string antennaSerial;
    std::string baseStationId;
    std::string sectorId;
    std::string installationDate;
    std::string installerId;
    std::string technology;
    std::string location;
    double mechanicalTiltDegrees{};
    double bearingDegrees{};
    double heightMeters{};

    [[nodiscard]] std::optional<std::string> validationError() const;
};

struct Ret {
    double electricalTiltDegrees{};
    double minimumTiltDegrees{};
    double maximumTiltDegrees{15.0};
    bool calibrated{true};
    bool moving{};

    [[nodiscard]] bool acceptsTilt(double degrees) const noexcept;
};

enum class TmaMode { normal, bypass };

struct Tma {
    double gainDb{};
    double minimumGainDb{};
    double maximumGainDb{30.0};
    TmaMode mode{TmaMode::normal};
};

using DeviceDetails = std::variant<std::monostate, Ret, Tma>;

struct Device {
    std::uint8_t address{};
    std::string uid;
    std::string vendor;
    std::string product;
    std::string serialNumber;
    std::string hardwareVersion;
    std::string softwareVersion;
    DeviceKind kind{DeviceKind::unknown};
    DeviceStatus status{DeviceStatus::disconnected};
    InstallationData installation;
    DeviceDetails details;
    std::vector<Alarm> alarms;
    std::uint64_t revision{};

    [[nodiscard]] Ret* ret() noexcept;
    [[nodiscard]] const Ret* ret() const noexcept;
    [[nodiscard]] Tma* tma() noexcept;
    [[nodiscard]] const Tma* tma() const noexcept;
};

[[nodiscard]] const char* toString(DeviceKind value) noexcept;
[[nodiscard]] const char* toString(DeviceStatus value) noexcept;
[[nodiscard]] const char* toString(AlarmSeverity value) noexcept;

} // namespace atc
