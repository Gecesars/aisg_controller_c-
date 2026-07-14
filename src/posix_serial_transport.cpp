#include "atc/posix_serial_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#if defined(__linux__)
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif

namespace atc {
namespace {

speed_t baudConstant(const unsigned int baudRate) {
    switch (baudRate) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
    default: throw TransportError("unsupported serial baud rate: " + std::to_string(baudRate));
    }
}

std::string systemError(const std::string& action) {
    return action + ": " + std::strerror(errno);
}

int pollTimeout(const std::chrono::milliseconds timeout) {
    return static_cast<int>(std::clamp<std::int64_t>(timeout.count(), 0, INT_MAX));
}

} // namespace

PosixSerialTransport::~PosixSerialTransport() { close(); }

void PosixSerialTransport::open(const TransportConfig& config) {
    std::scoped_lock lock(mutex_);
    if (descriptor_ >= 0) {
        throw TransportError("serial transport is already open");
    }
    if (config.endpoint.empty()) {
        throw TransportError("serial endpoint is empty");
    }

    const auto descriptor = ::open(config.endpoint.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor < 0) {
        throw TransportError(systemError("cannot open " + config.endpoint));
    }

    termios settings{};
    if (::tcgetattr(descriptor, &settings) != 0) {
        const auto message = systemError("cannot read serial settings");
        ::close(descriptor);
        throw TransportError(message);
    }
    ::cfmakeraw(&settings);
    const auto speed = baudConstant(config.baudRate);
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);
    settings.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    settings.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
    settings.c_cflag |= CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::tcsetattr(descriptor, TCSANOW, &settings) != 0) {
        const auto message = systemError("cannot configure serial endpoint");
        ::close(descriptor);
        throw TransportError(message);
    }

#if defined(__linux__) && defined(TIOCSRS485)
    if (config.rs485) {
        serial_rs485 rs485{};
        rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
        if (::ioctl(descriptor, TIOCSRS485, &rs485) != 0) {
            const auto message = systemError("cannot enable RS-485 mode");
            ::close(descriptor);
            throw TransportError(message);
        }
    }
#else
    if (config.rs485) {
        ::close(descriptor);
        throw TransportError("RS-485 mode is not supported on this platform");
    }
#endif

    ::tcflush(descriptor, TCIOFLUSH);
    descriptor_ = descriptor;
    endpoint_ = config.endpoint;
}

void PosixSerialTransport::close() noexcept {
    std::scoped_lock lock(mutex_);
    if (descriptor_ >= 0) {
        ::close(descriptor_);
        descriptor_ = -1;
    }
    endpoint_.clear();
}

bool PosixSerialTransport::isOpen() const noexcept {
    std::scoped_lock lock(mutex_);
    return descriptor_ >= 0;
}

void PosixSerialTransport::write(const std::span<const std::uint8_t> bytes) {
    std::scoped_lock lock(mutex_);
    if (descriptor_ < 0) {
        throw TransportError("serial transport is closed");
    }

    std::size_t offset = 0;
    while (offset < bytes.size()) {
        pollfd descriptor{descriptor_, POLLOUT, 0};
        const auto ready = ::poll(&descriptor, 1, 1000);
        if (ready <= 0) {
            throw TransportError(ready == 0 ? "serial write timed out" : systemError("serial poll failed"));
        }
        const auto written = ::write(descriptor_, bytes.data() + offset, bytes.size() - offset);
        if (written < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            throw TransportError(systemError("serial write failed"));
        }
        offset += static_cast<std::size_t>(written);
    }
    if (::tcdrain(descriptor_) != 0) {
        throw TransportError(systemError("serial drain failed"));
    }
}

hdlc::Bytes PosixSerialTransport::read(const std::chrono::milliseconds timeout,
                                       const std::size_t maximumBytes) {
    std::scoped_lock lock(mutex_);
    if (descriptor_ < 0) {
        throw TransportError("serial transport is closed");
    }
    if (maximumBytes == 0) {
        return {};
    }

    pollfd descriptor{descriptor_, POLLIN, 0};
    const auto ready = ::poll(&descriptor, 1, pollTimeout(timeout));
    if (ready == 0) {
        return {};
    }
    if (ready < 0) {
        if (errno == EINTR) {
            return {};
        }
        throw TransportError(systemError("serial poll failed"));
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        throw TransportError("serial endpoint reported an I/O error");
    }

    hdlc::Bytes result(std::min<std::size_t>(maximumBytes, 65536));
    const auto count = ::read(descriptor_, result.data(), result.size());
    if (count < 0) {
        if (errno == EINTR || errno == EAGAIN) {
            return {};
        }
        throw TransportError(systemError("serial read failed"));
    }
    result.resize(static_cast<std::size_t>(count));
    return result;
}

std::string PosixSerialTransport::description() const {
    std::scoped_lock lock(mutex_);
    return endpoint_.empty() ? "POSIX serial (closed)" : "POSIX serial " + endpoint_;
}

} // namespace atc
