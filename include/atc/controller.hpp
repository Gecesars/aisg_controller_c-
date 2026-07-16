#pragma once

#include "atc/aisg.hpp"
#include "atc/aisg3.hpp"
#include "atc/domain.hpp"
#include "atc/hdlc.hpp"
#include "atc/transport.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace atc {

using OperationId = std::uint64_t;

enum class EventKind {
    operationStarted,
    operationCompleted,
    operationCancelled,
    operationFailed,
    connectionChanged,
    scanProgress,
    deviceAdded,
    deviceUpdated,
    devicesCleared,
    txFrame,
    rxFrame,
    log,
};

struct ControllerEvent {
    EventKind kind{EventKind::log};
    OperationId operation{};
    std::chrono::system_clock::time_point timestamp{std::chrono::system_clock::now()};
    std::optional<std::uint8_t> address;
    std::optional<Device> device;
    int progress{-1};
    std::string message;
    hdlc::Bytes frame;
};

struct ScanOptions {
    std::uint8_t firstAddress{1};
    std::uint8_t lastAddress{32};
    std::chrono::milliseconds responseTimeout{50};
    ProtocolProfile protocol{ProtocolProfile::legacyAisg2};
    std::uint32_t primaryId{0x41544331};
    bool discoverUnaddressed{};
    bool resetBeforeDiscovery{};
};

class ControllerService {
public:
    explicit ControllerService(std::shared_ptr<ITransport> transport);
    ~ControllerService();

    ControllerService(const ControllerService&) = delete;
    ControllerService& operator=(const ControllerService&) = delete;

    [[nodiscard]] OperationId connect(TransportConfig config = {});
    [[nodiscard]] OperationId disconnect();
    [[nodiscard]] OperationId scan(ScanOptions options = {});
    [[nodiscard]] OperationId refresh(std::uint8_t address);
    [[nodiscard]] OperationId moveRet(std::uint8_t address, double tiltDegrees);
    [[nodiscard]] OperationId refreshAlarms(std::uint8_t address);
    [[nodiscard]] OperationId clearAlarms(std::uint8_t address);
    [[nodiscard]] OperationId runSelfTest(std::uint8_t address);
    [[nodiscard]] OperationId calibrate(std::uint8_t address);
    [[nodiscard]] OperationId setTmaGain(std::uint8_t address, double gainDb);
    [[nodiscard]] OperationId setTmaMode(std::uint8_t address, TmaMode mode);
    [[nodiscard]] OperationId setDeviceField(std::uint8_t address,
                                             aisg::Field field,
                                             hdlc::Bytes value);

    void cancel(OperationId operation) noexcept;
    [[nodiscard]] bool isConnected() const noexcept;
    [[nodiscard]] bool isBusy() const noexcept;
    [[nodiscard]] std::vector<Device> devices() const;
    [[nodiscard]] std::optional<Device> device(std::uint8_t address) const;

    // Designed for a GUI timer: callbacks never run on the worker thread.
    [[nodiscard]] std::vector<ControllerEvent> drainEvents();

private:
    struct Work;
    using Task = std::function<void(OperationId, const std::shared_ptr<std::atomic_bool>&)>;

    [[nodiscard]] OperationId enqueue(std::string name, Task task);
    void workerLoop(std::stop_token stopToken);
    void emit(ControllerEvent event);
    void logFrame(EventKind kind, OperationId operation, const hdlc::Bytes& encoded);
    [[nodiscard]] bool transmit(OperationId operation,
                                const hdlc::Bytes& encoded,
                                const std::shared_ptr<std::atomic_bool>& cancelled);
    void noteFrameActivity() noexcept;
    [[nodiscard]] std::optional<hdlc::Frame> transact(OperationId operation,
                                                      const hdlc::Frame& request,
                                                      std::chrono::milliseconds timeout,
                                                      const std::shared_ptr<std::atomic_bool>& cancelled);
    [[nodiscard]] std::optional<aisg3::Response> transactAisg3(
        OperationId operation,
        std::uint8_t address,
        const aisg3::PrimaryCommand& command,
        std::chrono::milliseconds timeout,
        const std::shared_ptr<std::atomic_bool>& cancelled);
    void scanAisg3(OperationId operation,
                   const ScanOptions& options,
                   const std::shared_ptr<std::atomic_bool>& cancelled);
    void scanAisg2(OperationId operation,
                   const ScanOptions& options,
                   const std::shared_ptr<std::atomic_bool>& cancelled);
    [[nodiscard]] Device requireDevice(std::uint8_t address) const;
    void storeDevice(Device device, OperationId operation, bool added = false);
    [[nodiscard]] std::uint8_t nextControl(std::uint8_t address);
    [[nodiscard]] std::uint16_t nextAisg3Sequence() noexcept;

    std::shared_ptr<ITransport> transport_;
    std::jthread worker_;

    mutable std::mutex stateMutex_;
    bool connected_{};
    bool activeWork_{};
    std::unordered_map<std::uint8_t, Device> devices_;
    std::unordered_map<std::uint8_t, aisg::ControlSequence> sequences_;
    std::uint16_t aisg3Sequence_{};
    std::chrono::milliseconds minimumFrameInterval_{};
    std::optional<std::chrono::steady_clock::time_point> lastFrameActivity_;

    mutable std::mutex queueMutex_;
    std::condition_variable_any queueCondition_;
    std::deque<Work> workQueue_;
    std::unordered_map<OperationId, std::shared_ptr<std::atomic_bool>> cancellation_;
    std::atomic<OperationId> nextOperation_{1};

    mutable std::mutex eventMutex_;
    std::vector<ControllerEvent> events_;
    hdlc::StreamDecoder decoder_;
};

[[nodiscard]] const char* toString(EventKind value) noexcept;

} // namespace atc
