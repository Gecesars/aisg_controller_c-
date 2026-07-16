#pragma once

#include "atc/hdlc.hpp"

#include <chrono>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>

namespace atc {

struct TransportConfig {
    std::string endpoint;
    unsigned int baudRate{115200};
    bool rs485{};
    std::chrono::milliseconds readTimeout{100};
    // Minimum quiet time on the bus before the controller transmits another
    // frame. Zero keeps the transport unpaced (used by unit tests/simulator).
    std::chrono::milliseconds minimumFrameInterval{};
};

class TransportError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class ITransport {
public:
    virtual ~ITransport() = default;

    virtual void open(const TransportConfig& config) = 0;
    virtual void close() noexcept = 0;
    [[nodiscard]] virtual bool isOpen() const noexcept = 0;
    virtual void write(std::span<const std::uint8_t> bytes) = 0;
    [[nodiscard]] virtual hdlc::Bytes read(std::chrono::milliseconds timeout,
                                           std::size_t maximumBytes = 4096) = 0;
    [[nodiscard]] virtual std::string description() const = 0;
};

} // namespace atc
