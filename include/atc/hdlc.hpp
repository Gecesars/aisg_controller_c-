#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace atc::hdlc {

using Bytes = std::vector<std::uint8_t>;

inline constexpr std::uint8_t flag = 0x7E;
inline constexpr std::uint8_t escape = 0x7D;
inline constexpr std::uint8_t escapeXor = 0x20;

struct Frame {
    std::uint8_t address{};
    std::uint8_t control{};
    Bytes information;

    friend bool operator==(const Frame&, const Frame&) = default;
};

enum class DecodeError { none, tooShort, missingFlag, invalidEscape, checksumMismatch, frameTooLarge };

struct DecodeResult {
    std::optional<Frame> frame;
    DecodeError error{DecodeError::none};
    std::string message;

    [[nodiscard]] explicit operator bool() const noexcept { return frame.has_value(); }
};

[[nodiscard]] std::uint16_t crc16X25(std::span<const std::uint8_t> data) noexcept;
[[nodiscard]] Bytes encode(const Frame& frame);
[[nodiscard]] DecodeResult decode(std::span<const std::uint8_t> encoded);

class StreamDecoder {
public:
    explicit StreamDecoder(std::size_t maximumFrameBytes = 4096);

    [[nodiscard]] std::vector<DecodeResult> push(std::span<const std::uint8_t> bytes);
    void reset() noexcept;

private:
    [[nodiscard]] DecodeResult finishFrame();

    std::size_t maximumFrameBytes_;
    Bytes buffer_;
    bool inFrame_{};
    bool escaped_{};
    bool dropping_{};
};

[[nodiscard]] std::string toHex(std::span<const std::uint8_t> bytes);

} // namespace atc::hdlc
