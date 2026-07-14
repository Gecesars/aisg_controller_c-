#pragma once

#include "atc/domain.hpp"
#include "atc/hdlc.hpp"
#include "atc/transport.hpp"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

namespace atc {

struct SimulatedDevice {
    Device device;
    std::vector<std::uint8_t> alarmCodes;
};

class SimulatedTransport final : public ITransport {
public:
    SimulatedTransport();
    explicit SimulatedTransport(std::vector<SimulatedDevice> devices);

    void open(const TransportConfig& config) override;
    void close() noexcept override;
    [[nodiscard]] bool isOpen() const noexcept override;
    void write(std::span<const std::uint8_t> bytes) override;
    [[nodiscard]] hdlc::Bytes read(std::chrono::milliseconds timeout,
                                   std::size_t maximumBytes = 4096) override;
    [[nodiscard]] std::string description() const override;

    [[nodiscard]] std::vector<SimulatedDevice> snapshot() const;

private:
    void process(const hdlc::Frame& request);
    void queue(const hdlc::Frame& response);

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool open_{};
    std::vector<SimulatedDevice> devices_;
    std::deque<std::uint8_t> received_;
    hdlc::StreamDecoder decoder_;
};

} // namespace atc
