#include "atc/hdlc.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace atc::hdlc {
namespace {

DecodeResult decodePayload(const std::span<const std::uint8_t> payload) {
    if (payload.size() < 4) {
        return {std::nullopt, DecodeError::tooShort,
                "an HDLC frame needs address, control and two FCS bytes"};
    }
    if (payload.size() > maximumUnescapedFrameBytes) {
        return {std::nullopt, DecodeError::frameTooLarge,
                "HDLC frame exceeds the AISG maximum of 268 octets"};
    }

    const auto data = payload.first(payload.size() - 2);
    const auto receivedFcs = static_cast<std::uint16_t>(payload[payload.size() - 2]) |
                             (static_cast<std::uint16_t>(payload.back()) << 8U);
    const auto calculatedFcs = crc16X25(data);
    if (receivedFcs != calculatedFcs) {
        return {std::nullopt, DecodeError::checksumMismatch,
                "CRC-16/X-25 mismatch"};
    }

    Frame frame;
    frame.address = payload[0];
    frame.control = payload[1];
    frame.information.assign(payload.begin() + 2, payload.end() - 2);
    return {std::move(frame), DecodeError::none, {}};
}

} // namespace

std::uint16_t crc16X25(const std::span<const std::uint8_t> data) noexcept {
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : data) {
        crc ^= byte;
        for (unsigned int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U
                      ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                      : static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return static_cast<std::uint16_t>(~crc);
}

Bytes encode(const Frame& frame) {
    if (frame.information.size() > maximumInformationBytes) {
        throw std::invalid_argument("HDLC information exceeds the AISG maximum of 264 octets");
    }
    Bytes payload;
    payload.reserve(frame.information.size() + 4);
    payload.push_back(frame.address);
    payload.push_back(frame.control);
    payload.insert(payload.end(), frame.information.begin(), frame.information.end());
    const auto fcs = crc16X25(payload);
    payload.push_back(static_cast<std::uint8_t>(fcs & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>((fcs >> 8U) & 0xFFU));

    Bytes encoded;
    encoded.reserve(payload.size() + 2);
    encoded.push_back(flag);
    for (const auto byte : payload) {
        if (byte == flag || byte == escape) {
            encoded.push_back(escape);
            encoded.push_back(static_cast<std::uint8_t>(byte ^ escapeXor));
        } else {
            encoded.push_back(byte);
        }
    }
    encoded.push_back(flag);
    return encoded;
}

DecodeResult decode(const std::span<const std::uint8_t> encoded) {
    if (encoded.size() < 2 || encoded.front() != flag || encoded.back() != flag) {
        return {std::nullopt, DecodeError::missingFlag,
                "an encoded HDLC frame must start and end with 0x7E"};
    }

    Bytes payload;
    payload.reserve(encoded.size() - 2);
    bool escaped = false;
    for (std::size_t index = 1; index + 1 < encoded.size(); ++index) {
        const auto byte = encoded[index];
        if (escaped) {
            payload.push_back(static_cast<std::uint8_t>(byte ^ escapeXor));
            escaped = false;
        } else if (byte == escape) {
            escaped = true;
        } else if (byte == flag) {
            return {std::nullopt, DecodeError::missingFlag,
                    "unexpected unescaped flag inside HDLC frame"};
        } else {
            payload.push_back(byte);
        }
    }
    if (escaped) {
        return {std::nullopt, DecodeError::invalidEscape,
                "HDLC frame ends with an incomplete escape"};
    }
    return decodePayload(payload);
}

StreamDecoder::StreamDecoder(const std::size_t maximumFrameBytes)
    : maximumFrameBytes_(maximumFrameBytes) {}

std::vector<DecodeResult> StreamDecoder::push(const std::span<const std::uint8_t> bytes) {
    std::vector<DecodeResult> results;
    for (const auto byte : bytes) {
        if (byte == flag) {
            if (inFrame_ && escaped_) {
                results.push_back({std::nullopt, DecodeError::invalidEscape,
                                   "HDLC frame ended with an incomplete escape"});
            } else if (inFrame_ && !dropping_ && !buffer_.empty()) {
                results.push_back(finishFrame());
            }
            buffer_.clear();
            inFrame_ = true;
            escaped_ = false;
            dropping_ = false;
            continue;
        }

        if (!inFrame_ || dropping_) {
            continue;
        }

        if (escaped_) {
            buffer_.push_back(static_cast<std::uint8_t>(byte ^ escapeXor));
            escaped_ = false;
        } else if (byte == escape) {
            escaped_ = true;
        } else {
            buffer_.push_back(byte);
        }

        if (buffer_.size() > maximumFrameBytes_) {
            results.push_back({std::nullopt, DecodeError::frameTooLarge,
                               "HDLC frame exceeded configured maximum size"});
            buffer_.clear();
            escaped_ = false;
            dropping_ = true;
        }
    }
    return results;
}

void StreamDecoder::reset() noexcept {
    buffer_.clear();
    inFrame_ = false;
    escaped_ = false;
    dropping_ = false;
}

DecodeResult StreamDecoder::finishFrame() { return decodePayload(buffer_); }

std::string toHex(const std::span<const std::uint8_t> bytes) {
    std::ostringstream stream;
    stream << std::hex << std::uppercase << std::setfill('0');
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        if (index != 0) {
            stream << ' ';
        }
        stream << std::setw(2) << static_cast<unsigned int>(bytes[index]);
    }
    return stream.str();
}

} // namespace atc::hdlc
