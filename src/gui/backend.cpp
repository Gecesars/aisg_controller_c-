#include "backend.hpp"

#include "atc/controller.hpp"
#include "atc/posix_serial_transport.hpp"
#include "atc/simulated_transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string_view>
#include <thread>
#include <utility>

namespace atc::gui {
namespace {

using namespace std::chrono_literals;

DeviceKind mapKind(const atc::DeviceKind kind) {
    switch (kind) {
    case atc::DeviceKind::ret: return DeviceKind::Ret;
    case atc::DeviceKind::tma: return DeviceKind::Tma;
    case atc::DeviceKind::adb: return DeviceKind::Adb;
    case atc::DeviceKind::unknown: return DeviceKind::Unknown;
    }
    return DeviceKind::Unknown;
}

DeviceState mapState(const atc::DeviceStatus state) {
    switch (state) {
    case atc::DeviceStatus::disconnected: return DeviceState::Offline;
    case atc::DeviceStatus::discovered:
    case atc::DeviceStatus::initializing: return DeviceState::Discovered;
    case atc::DeviceStatus::ready: return DeviceState::Operational;
    case atc::DeviceStatus::busy: return DeviceState::Busy;
    case atc::DeviceStatus::alarm: return DeviceState::Alarm;
    case atc::DeviceStatus::fault: return DeviceState::Fault;
    }
    return DeviceState::Fault;
}

hdlc::Bytes bytes(std::string_view value) { return {value.begin(), value.end()}; }

std::uint32_t stablePrimaryId() {
    std::ifstream machineId("/etc/machine-id");
    std::string identity;
    machineId >> identity;
    if (identity.empty()) {
        identity = "antenna-tilt-controller";
    }

    return atc::aisg3::primaryIdForNodeName(identity);
}

class CoreBackend final : public Backend {
public:
    CoreBackend() = default;

    void setLogCallback(LogCallback callback) override { log_ = std::move(callback); }

    OperationResult connect(const ConnectionSettings& settings) override {
        if (controller_ && controller_->isConnected()) {
            (void)wait(controller_->disconnect(), 2s);
        }
        controller_.reset();
        transport_.reset();
        simulatorTransport_.reset();
        isSimulator_ = settings.simulator;
        if (isSimulator_) {
            simulatorTransport_ = std::make_shared<atc::SimulatedTransport>();
            transport_ = simulatorTransport_;
        } else {
            transport_ = std::make_shared<atc::PosixSerialTransport>();
        }
        controller_ = std::make_unique<atc::ControllerService>(transport_);

        atc::TransportConfig config;
        config.endpoint = settings.port;
        config.baudRate = isSimulator_
                              ? static_cast<unsigned int>(std::max(settings.baudRate, 1))
                              : 9600U;
        // Most USB/RS-485 adapters control direction internally. Kernel
        // TIOCSRS485 can be enabled later for native UART endpoints.
        config.rs485 = false;
        config.readTimeout = 100ms;
        const auto result = wait(controller_->connect(std::move(config)), 2s);
        if (result.ok) emit(isSimulator_ ? "CORE Simulador AISG 2 conectado"
                                        : "CORE Serial AISG Base 3.0.8 conectado a 9600 8N1",
                            false);
        return result;
    }

    void disconnect() override {
        if (!controller_ || !controller_->isConnected()) return;
        (void)wait(controller_->disconnect(), 2s);
    }

    [[nodiscard]] bool connected() const override { return controller_ && controller_->isConnected(); }
    [[nodiscard]] bool simulator() const override { return isSimulator_; }

    std::vector<DeviceRecord> scan() override {
        if (!connected()) return {};
        atc::ScanOptions options;
        options.firstAddress = 1;
        options.lastAddress = isSimulator_ ? 4 : 254;
        options.responseTimeout = isSimulator_ ? 10ms : 250ms;
        options.protocol = isSimulator_ ? atc::ProtocolProfile::legacyAisg2
                                        : atc::ProtocolProfile::aisg3;
        options.primaryId = stablePrimaryId();
        const auto result = wait(controller_->scan(options), isSimulator_ ? 3s : 120s);
        if (!result.ok) return {};

        // Populate alarm state through the same HDLC/EP path used by hardware.
        const auto discovered = controller_->devices();
        for (const auto& device : discovered) {
            (void)wait(controller_->refreshAlarms(device.address),
                       device.protocol == atc::ProtocolProfile::aisg3 ? 30s : 1s);
        }
        return snapshot();
    }

    void cancelCurrent() noexcept override {
        if (controller_) {
            const auto operation = currentOperation_.load();
            if (operation != 0) controller_->cancel(operation);
        }
    }

    OperationResult refresh(DeviceRecord& device) override {
        const auto result = wait(controller_->refresh(device.address), device.aisg3 ? 10s : 2s);
        update(device);
        return result;
    }

    OperationResult setTilt(DeviceRecord& device, const double targetDegrees) override {
        if (device.aisg3) return unsupportedAisg3Write();
        const auto result = wait(controller_->moveRet(device.address, targetDegrees), 3s);
        update(device);
        return result;
    }

    OperationResult setGain(DeviceRecord& device, const double targetDb) override {
        if (device.aisg3) return unsupportedAisg3Write();
        if (device.kind != DeviceKind::Tma) return {false, "O dispositivo selecionado não é um TMA"};
        if (!std::isfinite(targetDb) || targetDb < 0.0 || targetDb > 31.75) {
            return {false, "Ganho fora do intervalo 0–31,75 dB"};
        }
        const auto result = wait(controller_->setTmaGain(device.address, targetDb), 2s);
        update(device);
        return result;
    }

    OperationResult setBypass(DeviceRecord& device, const bool bypass) override {
        if (device.aisg3) return unsupportedAisg3Write();
        if (device.kind != DeviceKind::Tma) return {false, "O dispositivo selecionado não é um TMA"};
        const auto result = wait(controller_->setTmaMode(
            device.address, bypass ? atc::TmaMode::bypass : atc::TmaMode::normal), 2s);
        update(device);
        return result;
    }

    OperationResult calibrate(DeviceRecord& device) override {
        if (device.aisg3) return unsupportedAisg3Write();
        const auto result = wait(controller_->calibrate(device.address), 3s);
        update(device);
        return result;
    }

    OperationResult selfTest(DeviceRecord& device) override {
        if (device.aisg3) return unsupportedAisg3Write();
        const auto result = wait(controller_->runSelfTest(device.address), 3s);
        update(device);
        return result;
    }

    OperationResult clearAlarms(DeviceRecord& device) override {
        const auto result = wait(controller_->clearAlarms(device.address), device.aisg3 ? 15s : 2s);
        update(device);
        return result;
    }

    OperationResult saveConfiguration(DeviceRecord& device) override {
        if (device.aisg3) return unsupportedAisg3Write();
        const auto installation = atc::InstallationData{
            .antennaModel = device.antennaModel,
            .antennaSerial = device.serial,
            .baseStationId = device.baseStation,
            .sectorId = device.sector,
            .installationDate = device.installationDate,
            .installerId = device.installerId,
            .technology = device.technology,
            .location = device.location,
            .mechanicalTiltDegrees = device.mechanicalTilt,
            .bearingDegrees = device.bearing,
            .heightMeters = device.height,
        };
        if (const auto validation = installation.validationError()) return {false, *validation};

        const std::pair<atc::aisg::Field, std::string_view> fields[] = {
            {atc::aisg::Field::antennaModel, device.antennaModel},
            {atc::aisg::Field::installationDate, device.installationDate},
            {atc::aisg::Field::installerId, device.installerId},
            {atc::aisg::Field::baseStationId, device.baseStation},
            {atc::aisg::Field::sectorId, device.sector},
        };
        for (const auto& [field, value] : fields) {
            const auto result = wait(controller_->setDeviceField(device.address, field, bytes(value)), 2s);
            if (!result.ok) return result;
        }
        // Fields not represented by the experimental capture profile stay in
        // the GUI session and are exported in reports, but are not sent raw.
        update(device, false);
        return {true, "Configuração validada e campos AISG suportados aplicados"};
    }

private:
    static OperationResult unsupportedAisg3Write() {
        return {false,
                "Operação não habilitada no perfil AISG 3.0.8: somente descoberta, "
                "negociação, leitura Base/ADB e alarmes foram validados"};
    }

    OperationResult wait(const atc::OperationId operation,
                         const std::chrono::milliseconds timeout) {
        currentOperation_.store(operation);
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            for (auto& event : controller_->drainEvents()) {
                logEvent(event);
                if (event.operation != operation) continue;
                if (event.kind == atc::EventKind::operationCompleted) {
                    currentOperation_.store(0);
                    return {true, event.message};
                }
                if (event.kind == atc::EventKind::operationCancelled) {
                    currentOperation_.store(0);
                    return {false, event.message};
                }
                if (event.kind == atc::EventKind::operationFailed) {
                    currentOperation_.store(0);
                    return {false, event.message};
                }
            }
            std::this_thread::sleep_for(2ms);
        }
        controller_->cancel(operation);
        currentOperation_.store(0);
        return {false, "Tempo limite excedido"};
    }

    void logEvent(const atc::ControllerEvent& event) const {
        if (event.kind == atc::EventKind::txFrame) {
            emit("TX   " + event.message, true);
        } else if (event.kind == atc::EventKind::rxFrame) {
            emit("RX   " + event.message, true);
        } else if (event.kind == atc::EventKind::operationFailed ||
                   event.kind == atc::EventKind::log) {
            emit(std::string(atc::toString(event.kind)) + "  " + event.message, false);
        }
    }

    [[nodiscard]] std::vector<DeviceRecord> snapshot() const {
        std::vector<DeviceRecord> result;
        const auto simulatorDevices = simulatorTransport_ ? simulatorTransport_->snapshot()
                                                          : std::vector<atc::SimulatedDevice>{};
        for (const auto& device : controller_->devices()) {
            const auto fallback = std::find_if(simulatorDevices.begin(), simulatorDevices.end(),
                [&](const auto& candidate) { return candidate.device.address == device.address; });
            result.push_back(map(device, fallback == simulatorDevices.end() ? nullptr : &fallback->device));
        }
        return result;
    }

    void update(DeviceRecord& destination, const bool overwriteSessionFields = true) const {
        const auto value = controller_->device(destination.address);
        if (!value) return;
        const auto simulatorDevices = simulatorTransport_ ? simulatorTransport_->snapshot()
                                                          : std::vector<atc::SimulatedDevice>{};
        const auto fallback = std::find_if(simulatorDevices.begin(), simulatorDevices.end(),
            [&](const auto& candidate) { return candidate.device.address == destination.address; });
        auto mapped = map(*value, fallback == simulatorDevices.end() ? nullptr : &fallback->device);
        if (!overwriteSessionFields) {
            mapped.antennaModel = destination.antennaModel;
            mapped.serial = destination.serial;
            mapped.baseStation = destination.baseStation;
            mapped.sector = destination.sector;
            mapped.installationDate = destination.installationDate;
            mapped.installerId = destination.installerId;
            mapped.technology = destination.technology;
            mapped.location = destination.location;
            mapped.mechanicalTilt = destination.mechanicalTilt;
            mapped.bearing = destination.bearing;
            mapped.height = destination.height;
        }
        destination = std::move(mapped);
    }

    static DeviceRecord map(const atc::Device& source, const atc::Device* fallback) {
        const auto& installation = fallback ? fallback->installation : source.installation;
        DeviceRecord destination;
        destination.address = source.address;
        destination.kind = mapKind(source.kind);
        destination.state = mapState(source.status);
        destination.uid = source.uid;
        destination.vendor = source.vendor.empty() && fallback ? fallback->vendor : source.vendor;
        destination.product = source.product;
        destination.serial = source.serialNumber;
        destination.hardwareVersion = source.hardwareVersion;
        destination.softwareVersion = source.softwareVersion;
        destination.aisgVersion = source.protocol == atc::ProtocolProfile::aisg3 ? "3.0.8" : "2.0";
        destination.aisg3 = source.protocol == atc::ProtocolProfile::aisg3;
        destination.antennaModel = installation.antennaModel;
        destination.baseStation = installation.baseStationId;
        destination.sector = installation.sectorId;
        destination.installationDate = installation.installationDate;
        destination.installerId = installation.installerId;
        destination.technology = installation.technology;
        destination.location = installation.location;
        destination.mechanicalTilt = installation.mechanicalTiltDegrees;
        destination.bearing = installation.bearingDegrees;
        destination.height = installation.heightMeters;
        if (!installation.antennaSerial.empty()) {
            destination.serial = installation.antennaSerial;
        }
        destination.activeAlarms = static_cast<int>(source.alarms.size());
        if (destination.aisg3) destination.calibrated = false;
        if (const auto* ret = source.ret()) {
            destination.electricalTilt = ret->electricalTiltDegrees;
            destination.minimumTilt = ret->minimumTiltDegrees;
            destination.maximumTilt = ret->maximumTiltDegrees;
            destination.calibrated = ret->calibrated;
        } else if (const auto* tma = source.tma()) {
            destination.gain = tma->gainDb;
            destination.bypass = tma->mode == atc::TmaMode::bypass;
        }
        if (fallback) {
            if (const auto* ret = fallback->ret()) {
                destination.minimumTilt = ret->minimumTiltDegrees;
                destination.maximumTilt = ret->maximumTiltDegrees;
                destination.calibrated = ret->calibrated;
            } else if (const auto* tma = fallback->tma()) {
                destination.gain = tma->gainDb;
                destination.bypass = tma->mode == atc::TmaMode::bypass;
            }
        }
        return destination;
    }

    void emit(const std::string& text, const bool frame) const {
        if (log_) log_(text, frame);
    }

    std::shared_ptr<atc::ITransport> transport_;
    std::shared_ptr<atc::SimulatedTransport> simulatorTransport_;
    std::unique_ptr<atc::ControllerService> controller_;
    LogCallback log_;
    bool isSimulator_{true};
    std::atomic<atc::OperationId> currentOperation_{0};
};

}  // namespace

std::unique_ptr<Backend> makeControllerBackend() {
    return std::make_unique<CoreBackend>();
}

}  // namespace atc::gui
