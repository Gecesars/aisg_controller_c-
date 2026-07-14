#pragma once

#include "atc/transport.hpp"

#include <mutex>
#include <string>

namespace atc {

class PosixSerialTransport final : public ITransport {
public:
    PosixSerialTransport() = default;
    ~PosixSerialTransport() override;

    PosixSerialTransport(const PosixSerialTransport&) = delete;
    PosixSerialTransport& operator=(const PosixSerialTransport&) = delete;

    void open(const TransportConfig& config) override;
    void close() noexcept override;
    [[nodiscard]] bool isOpen() const noexcept override;
    void write(std::span<const std::uint8_t> bytes) override;
    [[nodiscard]] hdlc::Bytes read(std::chrono::milliseconds timeout,
                                   std::size_t maximumBytes = 4096) override;
    [[nodiscard]] std::string description() const override;

private:
    mutable std::mutex mutex_;
    int descriptor_{-1};
    std::string endpoint_;
};

} // namespace atc
