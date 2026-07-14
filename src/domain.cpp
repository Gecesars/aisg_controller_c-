#include "atc/domain.hpp"

#include <cmath>

namespace atc {

std::optional<std::string> InstallationData::validationError() const {
    if (antennaSerial.size() > 17) {
        return "antenna serial must contain at most 17 characters";
    }
    if (installerId.size() > 5) {
        return "installer id must contain at most 5 characters";
    }
    if (!installationDate.empty() && installationDate.size() != 6) {
        return "installation date must use MMDDYY (6 characters)";
    }
    if (!std::isfinite(mechanicalTiltDegrees)) {
        return "mechanical tilt must be finite";
    }
    if (!std::isfinite(bearingDegrees) || bearingDegrees < 0.0 || bearingDegrees > 359.9) {
        return "bearing must be between 0 and 359.9 degrees";
    }
    if (!std::isfinite(heightMeters) || heightMeters < 0.0 || heightMeters > 999.0) {
        return "height must be between 0 and 999 metres";
    }
    return std::nullopt;
}

bool Ret::acceptsTilt(const double degrees) const noexcept {
    return std::isfinite(degrees) && degrees >= minimumTiltDegrees && degrees <= maximumTiltDegrees;
}

Ret* Device::ret() noexcept { return std::get_if<Ret>(&details); }
const Ret* Device::ret() const noexcept { return std::get_if<Ret>(&details); }
Tma* Device::tma() noexcept { return std::get_if<Tma>(&details); }
const Tma* Device::tma() const noexcept { return std::get_if<Tma>(&details); }

const char* toString(const DeviceKind value) noexcept {
    switch (value) {
    case DeviceKind::ret: return "RET";
    case DeviceKind::tma: return "TMA";
    case DeviceKind::adb: return "ADB";
    case DeviceKind::unknown: return "Unknown";
    }
    return "Unknown";
}

const char* toString(const ProtocolProfile value) noexcept {
    switch (value) {
    case ProtocolProfile::legacyAisg2: return "AISG 2.0 experimental";
    case ProtocolProfile::aisg3: return "AISG 3.0.8";
    }
    return "Unknown";
}

const char* toString(const DeviceStatus value) noexcept {
    switch (value) {
    case DeviceStatus::disconnected: return "Disconnected";
    case DeviceStatus::discovered: return "Discovered";
    case DeviceStatus::initializing: return "Initializing";
    case DeviceStatus::ready: return "Ready";
    case DeviceStatus::busy: return "Busy";
    case DeviceStatus::alarm: return "Alarm";
    case DeviceStatus::fault: return "Fault";
    }
    return "Unknown";
}

const char* toString(const AlarmSeverity value) noexcept {
    switch (value) {
    case AlarmSeverity::information: return "Information";
    case AlarmSeverity::warning: return "Warning";
    case AlarmSeverity::critical: return "Critical";
    }
    return "Unknown";
}

} // namespace atc
