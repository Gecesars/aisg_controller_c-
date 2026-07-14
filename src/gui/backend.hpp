#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace atc::gui {

enum class DeviceKind { Ret, Tma, Sensor, Unknown };
enum class DeviceState { Offline, Discovered, Operational, Busy, Alarm, Fault };

struct DeviceRecord {
    std::uint8_t address{0};
    DeviceKind kind{DeviceKind::Unknown};
    DeviceState state{DeviceState::Offline};
    std::string uid;
    std::string vendor;
    std::string product;
    std::string serial;
    std::string hardwareVersion;
    std::string softwareVersion;
    std::string aisgVersion{"2.0"};
    std::string antennaModel;
    std::string baseStation;
    std::string sector;
    std::string frequencyBand;
    std::string installationDate;
    std::string installerId;
    std::string technology;
    std::string location;
    double electricalTilt{0.0};
    double mechanicalTilt{0.0};
    double minimumTilt{0.0};
    double maximumTilt{15.0};
    double bearing{0.0};
    double height{0.0};
    double gain{0.0};
    bool bypass{false};
    bool calibrated{true};
    int activeAlarms{0};
};

struct ConnectionSettings {
    bool simulator{true};
    std::string port;
    int baudRate{9600};
};

struct OperationResult {
    bool ok{false};
    std::string message;
};

class Backend {
public:
    using LogCallback = std::function<void(const std::string&, bool)>;

    virtual ~Backend() = default;
    virtual void setLogCallback(LogCallback callback) = 0;
    virtual OperationResult connect(const ConnectionSettings& settings) = 0;
    virtual void disconnect() = 0;
    [[nodiscard]] virtual bool connected() const = 0;
    [[nodiscard]] virtual bool simulator() const = 0;
    virtual std::vector<DeviceRecord> scan() = 0;
    virtual void cancelCurrent() noexcept = 0;
    virtual OperationResult refresh(DeviceRecord& device) = 0;
    virtual OperationResult setTilt(DeviceRecord& device, double targetDegrees) = 0;
    virtual OperationResult setGain(DeviceRecord& device, double targetDb) = 0;
    virtual OperationResult setBypass(DeviceRecord& device, bool bypass) = 0;
    virtual OperationResult calibrate(DeviceRecord& device) = 0;
    virtual OperationResult selfTest(DeviceRecord& device) = 0;
    virtual OperationResult clearAlarms(DeviceRecord& device) = 0;
    virtual OperationResult saveConfiguration(DeviceRecord& device) = 0;
};

// Backend seguro para desenvolvimento sem hardware. O adaptador do núcleo AISG
// implementa a mesma interface, mantendo a GUI livre de bytes de protocolo.
std::unique_ptr<Backend> makeControllerBackend();

}  // namespace atc::gui
