#include "atc/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace atc {

struct ControllerService::Work {
    OperationId id{};
    std::string name;
    Task task;
    std::shared_ptr<std::atomic_bool> cancelled;
};

ControllerService::ControllerService(std::shared_ptr<ITransport> transport)
    : transport_(std::move(transport)),
      worker_([this](const std::stop_token stopToken) { workerLoop(stopToken); }) {
    if (!transport_) {
        throw std::invalid_argument("ControllerService needs a transport");
    }
}

ControllerService::~ControllerService() {
    worker_.request_stop();
    queueCondition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    transport_->close();
}

OperationId ControllerService::connect(TransportConfig config) {
    return enqueue("Connect", [this, config = std::move(config)](
                                  const OperationId operation,
                                  const std::shared_ptr<std::atomic_bool>&) {
        if (transport_->isOpen()) {
            transport_->close();
        }
        transport_->open(config);
        decoder_.reset();
        {
            std::scoped_lock lock(stateMutex_);
            connected_ = true;
        }
        emit({EventKind::connectionChanged, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, "Connected: " + transport_->description(), {}});
    });
}

OperationId ControllerService::disconnect() {
    return enqueue("Disconnect", [this](const OperationId operation,
                                         const std::shared_ptr<std::atomic_bool>&) {
        transport_->close();
        decoder_.reset();
        {
            std::scoped_lock lock(stateMutex_);
            connected_ = false;
            devices_.clear();
            sequences_.clear();
        }
        emit({EventKind::devicesCleared, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, "Device list cleared", {}});
        emit({EventKind::connectionChanged, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, "Disconnected", {}});
    });
}

OperationId ControllerService::scan(const ScanOptions options) {
    return enqueue("Scan", [this, options](const OperationId operation,
                                            const std::shared_ptr<std::atomic_bool>& cancelled) {
        if (!isConnected()) {
            throw std::runtime_error("cannot scan while disconnected");
        }
        if (options.firstAddress == 0 || options.firstAddress > options.lastAddress) {
            throw std::invalid_argument("invalid AISG scan address range");
        }

        {
            std::scoped_lock lock(stateMutex_);
            devices_.clear();
            sequences_.clear();
        }
        emit({EventKind::devicesCleared, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, 0, "Starting address scan", {}});

        const auto total = static_cast<unsigned int>(options.lastAddress) -
                           static_cast<unsigned int>(options.firstAddress) + 1U;
        for (unsigned int rawAddress = options.firstAddress;
             rawAddress <= options.lastAddress; ++rawAddress) {
            if (cancelled->load()) {
                return;
            }
            const auto address = static_cast<std::uint8_t>(rawAddress);
            const auto snrm = aisg::makeSnrm(address);
            const auto ua = transact(operation, snrm, options.responseTimeout, cancelled);
            if (ua) {
                Device found;
                found.address = address;
                found.status = DeviceStatus::initializing;

                const auto initialRequest = aisg::makeInitialDataRequest(address, nextControl(address));
                const auto initialResponse = transact(operation, initialRequest,
                                                      options.responseTimeout, cancelled);
                if (initialResponse) {
                    if (const auto initial = aisg::parseInitialData(*initialResponse);
                        initial && initial->status.success) {
                        found.product = initial->product;
                        found.serialNumber = initial->serialNumber;
                        found.hardwareVersion = initial->hardwareVersion;
                        found.softwareVersion = initial->softwareVersion;
                        found.uid = initial->serialNumber;
                    }
                }

                const auto isTma = found.product.find("TMA") != std::string::npos;
                found.kind = isTma ? DeviceKind::tma : DeviceKind::ret;
                if (isTma) {
                    found.details = Tma{};
                    auto& tma = std::get<Tma>(found.details);
                    const auto gainRequest = aisg::makeGetTmaGainRequest(address, nextControl(address));
                    if (const auto gainResponse = transact(operation, gainRequest,
                                                           options.responseTimeout, cancelled)) {
                        if (const auto gain = aisg::parseTmaGain(*gainResponse)) {
                            tma.gainDb = *gain;
                        }
                    }
                    const auto modeRequest = aisg::makeGetTmaModeRequest(address, nextControl(address));
                    if (const auto modeResponse = transact(operation, modeRequest,
                                                           options.responseTimeout, cancelled)) {
                        if (const auto mode = aisg::parseTmaMode(*modeResponse)) {
                            tma.mode = *mode;
                        }
                    }
                } else {
                    found.details = Ret{};
                    const auto tiltRequest = aisg::makeGetTiltRequest(address, nextControl(address));
                    const auto tiltResponse = transact(operation, tiltRequest,
                                                       options.responseTimeout, cancelled);
                    if (tiltResponse) {
                        if (const auto tilt = aisg::parseTilt(*tiltResponse)) {
                            std::get<Ret>(found.details).electricalTiltDegrees = *tilt;
                        }
                    }
                }
                found.status = DeviceStatus::ready;
                storeDevice(std::move(found), operation, true);
            }

            const auto completed = rawAddress - options.firstAddress + 1U;
            const auto progress = static_cast<int>((completed * 100U) / total);
            emit({EventKind::scanProgress, operation, std::chrono::system_clock::now(),
                  address, std::nullopt, progress,
                  "Scanned AISG address " + std::to_string(rawAddress), {}});
        }
    });
}

OperationId ControllerService::refresh(const std::uint8_t address) {
    return enqueue("Refresh device", [this, address](
                                              const OperationId operation,
                                              const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        const auto timeout = std::chrono::milliseconds(250);
        const auto initialRequest = aisg::makeInitialDataRequest(address, nextControl(address));
        const auto initialResponse = transact(operation, initialRequest, timeout, cancelled);
        if (!initialResponse) {
            throw std::runtime_error("device information request timed out");
        }
        const auto initial = aisg::parseInitialData(*initialResponse);
        if (!initial || !initial->status.success) {
            throw std::runtime_error("device returned invalid initial data");
        }
        current.product = initial->product;
        current.serialNumber = initial->serialNumber;
        current.hardwareVersion = initial->hardwareVersion;
        current.softwareVersion = initial->softwareVersion;

        if (auto* ret = current.ret()) {
            const auto tiltRequest = aisg::makeGetTiltRequest(address, nextControl(address));
            const auto tiltResponse = transact(operation, tiltRequest, timeout, cancelled);
            if (!tiltResponse) {
                throw std::runtime_error("tilt request timed out");
            }
            if (const auto tilt = aisg::parseTilt(*tiltResponse)) {
                ret->electricalTiltDegrees = *tilt;
            }
        }
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::moveRet(const std::uint8_t address, const double tiltDegrees) {
    return enqueue("Move RET", [this, address, tiltDegrees](
                                         const OperationId operation,
                                         const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        auto* ret = current.ret();
        if (!ret) {
            throw std::invalid_argument("selected device is not a RET");
        }
        if (!ret->acceptsTilt(tiltDegrees)) {
            throw std::invalid_argument("requested tilt is outside RET limits");
        }

        const auto request = aisg::makeSetTiltRequest(address, nextControl(address), tiltDegrees);
        const auto response = transact(operation, request, std::chrono::milliseconds(1000), cancelled);
        if (!response) {
            throw std::runtime_error("RET move timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setTilt);
        if (!status.success) {
            throw std::runtime_error("RET rejected move: " + status.message);
        }
        ret->electricalTiltDegrees = std::round(tiltDegrees * 10.0) / 10.0;
        ret->moving = false;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::refreshAlarms(const std::uint8_t address) {
    return enqueue("Get alarms", [this, address](
                                          const OperationId operation,
                                          const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        const auto request = aisg::makeGetAlarmsRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("alarm request timed out");
        }
        const auto alarms = aisg::parseAlarms(*response);
        if (!alarms || !alarms->status.success) {
            throw std::runtime_error("device returned invalid alarm data");
        }
        current.alarms = alarms->alarms;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::clearAlarms(const std::uint8_t address) {
    return enqueue("Clear alarms", [this, address](
                                            const OperationId operation,
                                            const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        const auto request = aisg::makeClearAlarmsRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("clear alarms timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::clearAlarms);
        if (!status.success) {
            throw std::runtime_error("device rejected clear alarms: " + status.message);
        }
        current.alarms.clear();
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::runSelfTest(const std::uint8_t address) {
    return enqueue("Self test", [this, address](
                                        const OperationId operation,
                                        const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        const auto request = aisg::makeSelfTestRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(1000), cancelled);
        if (!response) {
            throw std::runtime_error("self test timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::selfTest);
        if (!status.success) {
            throw std::runtime_error("self test failed: " + status.message);
        }
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::calibrate(const std::uint8_t address) {
    return enqueue("Calibrate", [this, address](
                                        const OperationId operation,
                                        const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        auto* ret = current.ret();
        if (!ret) {
            throw std::invalid_argument("selected device is not a RET");
        }
        const auto request = aisg::makeCalibrateRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(2000), cancelled);
        if (!response) {
            throw std::runtime_error("calibration timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::calibrate);
        if (!status.success) {
            throw std::runtime_error("calibration failed: " + status.message);
        }
        ret->calibrated = true;
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::setTmaGain(const std::uint8_t address, const double gainDb) {
    return enqueue("Set TMA gain", [this, address, gainDb](
                                          const OperationId operation,
                                          const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        auto* tma = current.tma();
        if (!tma) {
            throw std::invalid_argument("selected device is not a TMA");
        }
        if (!std::isfinite(gainDb) || gainDb < tma->minimumGainDb || gainDb > tma->maximumGainDb) {
            throw std::invalid_argument("requested gain is outside TMA limits");
        }
        const auto rounded = std::round(gainDb * 4.0) / 4.0;
        const auto request = aisg::makeSetTmaGainRequest(address, nextControl(address), rounded);
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("TMA gain request timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setTmaGain);
        if (!status.success) {
            throw std::runtime_error("TMA rejected gain: " + status.message);
        }
        tma->gainDb = rounded;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::setTmaMode(const std::uint8_t address, const TmaMode mode) {
    return enqueue("Set TMA mode", [this, address, mode](
                                          const OperationId operation,
                                          const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        auto* tma = current.tma();
        if (!tma) {
            throw std::invalid_argument("selected device is not a TMA");
        }
        const auto request = aisg::makeSetTmaModeRequest(address, nextControl(address), mode);
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("TMA mode request timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setTmaMode);
        if (!status.success) {
            throw std::runtime_error("TMA rejected mode: " + status.message);
        }
        tma->mode = mode;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::setDeviceField(const std::uint8_t address,
                                               const aisg::Field field,
                                               hdlc::Bytes value) {
    return enqueue("Set device data", [this, address, field, value = std::move(value)](
                                               const OperationId operation,
                                               const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        const auto request = aisg::makeSetDataRequest(address, nextControl(address), field, value);
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("set device data timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setDeviceData);
        if (!status.success) {
            throw std::runtime_error("device rejected configuration: " + status.message);
        }
        const std::string textValue(value.begin(), value.end());
        switch (field) {
        case aisg::Field::antennaModel: current.installation.antennaModel = textValue; break;
        case aisg::Field::installationDate: current.installation.installationDate = textValue; break;
        case aisg::Field::installerId: current.installation.installerId = textValue; break;
        case aisg::Field::baseStationId: current.installation.baseStationId = textValue; break;
        case aisg::Field::sectorId: current.installation.sectorId = textValue; break;
        default: break;
        }
        storeDevice(std::move(current), operation);
    });
}

void ControllerService::cancel(const OperationId operation) noexcept {
    std::scoped_lock lock(queueMutex_);
    if (const auto iterator = cancellation_.find(operation); iterator != cancellation_.end()) {
        iterator->second->store(true);
    }
}

bool ControllerService::isConnected() const noexcept {
    std::scoped_lock lock(stateMutex_);
    return connected_;
}

bool ControllerService::isBusy() const noexcept {
    std::scoped_lock stateLock(stateMutex_);
    std::scoped_lock queueLock(queueMutex_);
    return activeWork_ || !workQueue_.empty();
}

std::vector<Device> ControllerService::devices() const {
    std::scoped_lock lock(stateMutex_);
    std::vector<Device> result;
    result.reserve(devices_.size());
    for (const auto& [address, device] : devices_) {
        (void)address;
        result.push_back(device);
    }
    std::sort(result.begin(), result.end(), [](const Device& left, const Device& right) {
        return left.address < right.address;
    });
    return result;
}

std::optional<Device> ControllerService::device(const std::uint8_t address) const {
    std::scoped_lock lock(stateMutex_);
    const auto iterator = devices_.find(address);
    return iterator == devices_.end() ? std::nullopt : std::optional<Device>(iterator->second);
}

std::vector<ControllerEvent> ControllerService::drainEvents() {
    std::scoped_lock lock(eventMutex_);
    auto result = std::move(events_);
    events_.clear();
    return result;
}

OperationId ControllerService::enqueue(std::string name, Task task) {
    const auto id = nextOperation_.fetch_add(1);
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    {
        std::scoped_lock lock(queueMutex_);
        cancellation_[id] = cancelled;
        workQueue_.push_back({id, std::move(name), std::move(task), std::move(cancelled)});
    }
    queueCondition_.notify_all();
    return id;
}

void ControllerService::workerLoop(const std::stop_token stopToken) {
    while (!stopToken.stop_requested()) {
        Work work;
        {
            std::unique_lock lock(queueMutex_);
            if (!queueCondition_.wait(lock, stopToken, [this] { return !workQueue_.empty(); })) {
                break;
            }
            work = std::move(workQueue_.front());
            workQueue_.pop_front();
        }
        {
            std::scoped_lock lock(stateMutex_);
            activeWork_ = true;
        }
        emit({EventKind::operationStarted, work.id, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, work.name, {}});
        try {
            if (!work.cancelled->load()) {
                work.task(work.id, work.cancelled);
            }
            emit({work.cancelled->load() ? EventKind::operationCancelled
                                         : EventKind::operationCompleted,
                  work.id, std::chrono::system_clock::now(), std::nullopt, std::nullopt,
                  -1, work.cancelled->load() ? work.name + " cancelled" : work.name + " completed", {}});
        } catch (const std::exception& exception) {
            emit({EventKind::operationFailed, work.id, std::chrono::system_clock::now(),
                  std::nullopt, std::nullopt, -1, exception.what(), {}});
        } catch (...) {
            emit({EventKind::operationFailed, work.id, std::chrono::system_clock::now(),
                  std::nullopt, std::nullopt, -1, "unknown controller error", {}});
        }
        {
            std::scoped_lock lock(queueMutex_);
            cancellation_.erase(work.id);
        }
        {
            std::scoped_lock lock(stateMutex_);
            activeWork_ = false;
        }
    }
}

void ControllerService::emit(ControllerEvent event) {
    std::scoped_lock lock(eventMutex_);
    events_.push_back(std::move(event));
}

void ControllerService::logFrame(const EventKind kind,
                                 const OperationId operation,
                                 const hdlc::Bytes& encoded) {
    emit({kind, operation, std::chrono::system_clock::now(), std::nullopt, std::nullopt,
          -1, hdlc::toHex(encoded), encoded});
}

std::optional<hdlc::Frame> ControllerService::transact(
    const OperationId operation,
    const hdlc::Frame& request,
    const std::chrono::milliseconds timeout,
    const std::shared_ptr<std::atomic_bool>& cancelled) {
    if (!transport_->isOpen()) {
        throw TransportError("transport is closed");
    }
    const auto encoded = hdlc::encode(request);
    logFrame(EventKind::txFrame, operation, encoded);
    transport_->write(encoded);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && !cancelled->load()) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto bytes = transport_->read(std::max(remaining, std::chrono::milliseconds(1)));
        if (bytes.empty()) {
            continue;
        }
        logFrame(EventKind::rxFrame, operation, bytes);
        for (auto& result : decoder_.push(bytes)) {
            if (!result.frame) {
                emit({EventKind::log, operation, std::chrono::system_clock::now(),
                      std::nullopt, std::nullopt, -1, "Discarded HDLC frame: " + result.message, {}});
                continue;
            }
            if (aisg::isResponseFor(*result.frame, request)) {
                return std::move(*result.frame);
            }
            emit({EventKind::log, operation, std::chrono::system_clock::now(),
                  result.frame->address, std::nullopt, -1,
                  "Received an unrelated or unsolicited AISG frame", {}});
        }
    }
    return std::nullopt;
}

Device ControllerService::requireDevice(const std::uint8_t address) const {
    if (!isConnected()) {
        throw std::runtime_error("controller is disconnected");
    }
    std::scoped_lock lock(stateMutex_);
    const auto iterator = devices_.find(address);
    if (iterator == devices_.end()) {
        throw std::out_of_range("AISG address is not present in the device list");
    }
    return iterator->second;
}

void ControllerService::storeDevice(Device device,
                                    const OperationId operation,
                                    const bool added) {
    ++device.revision;
    const auto address = device.address;
    {
        std::scoped_lock lock(stateMutex_);
        devices_[address] = device;
    }
    emit({added ? EventKind::deviceAdded : EventKind::deviceUpdated,
          operation, std::chrono::system_clock::now(), address, std::move(device),
          -1, added ? "Device discovered" : "Device updated", {}});
}

std::uint8_t ControllerService::nextControl(const std::uint8_t address) {
    std::scoped_lock lock(stateMutex_);
    return sequences_[address].nextRequest();
}

const char* toString(const EventKind value) noexcept {
    switch (value) {
    case EventKind::operationStarted: return "Operation started";
    case EventKind::operationCompleted: return "Operation completed";
    case EventKind::operationCancelled: return "Operation cancelled";
    case EventKind::operationFailed: return "Operation failed";
    case EventKind::connectionChanged: return "Connection changed";
    case EventKind::scanProgress: return "Scan progress";
    case EventKind::deviceAdded: return "Device added";
    case EventKind::deviceUpdated: return "Device updated";
    case EventKind::devicesCleared: return "Devices cleared";
    case EventKind::txFrame: return "TX";
    case EventKind::rxFrame: return "RX";
    case EventKind::log: return "Log";
    }
    return "Unknown";
}

} // namespace atc
