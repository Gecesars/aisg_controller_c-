#include "ret/ret_hdlc.h"

#include <string.h>

uint16_t ret_hdlc_crc16_x25(const uint8_t *data, const size_t length) {
    uint16_t crc = 0xFFFFu;
    size_t index;

    if ((data == NULL) && (length != 0u)) {
        return 0u;
    }

    for (index = 0u; index < length; ++index) {
        unsigned bit;
        crc ^= data[index];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = ((crc & 1u) != 0u)
                      ? (uint16_t)((crc >> 1u) ^ 0x8408u)
                      : (uint16_t)(crc >> 1u);
        }
    }
    return (uint16_t)~crc;
}

static bool append_wire_octet(uint8_t *wire,
                              const size_t capacity,
                              size_t *length,
                              const uint8_t octet) {
    if ((octet == RET_HDLC_FLAG) || (octet == RET_HDLC_ESCAPE)) {
        if ((*length + 2u) > capacity) {
            return false;
        }
        wire[(*length)++] = RET_HDLC_ESCAPE;
        wire[(*length)++] = (uint8_t)(octet ^ RET_HDLC_ESCAPE_XOR);
        return true;
    }
    if ((*length + 1u) > capacity) {
        return false;
    }
    wire[(*length)++] = octet;
    return true;
}

bool ret_hdlc_encode(const ret_hdlc_frame_t *frame,
                     uint8_t *wire,
                     const size_t wire_capacity,
                     size_t *wire_length) {
    uint8_t raw[RET_HDLC_MAX_RAW];
    size_t raw_length;
    size_t encoded_length = 0u;
    size_t index;
    uint16_t fcs;

    if ((frame == NULL) || (wire == NULL) || (wire_length == NULL) ||
        (frame->information_length > RET_HDLC_MAX_INFO) ||
        (wire_capacity < 2u)) {
        return false;
    }

    raw[0] = frame->address;
    raw[1] = frame->control;
    if (frame->information_length != 0u) {
        memcpy(&raw[2], frame->information, frame->information_length);
    }
    raw_length = (size_t)frame->information_length + 2u;
    fcs = ret_hdlc_crc16_x25(raw, raw_length);
    raw[raw_length++] = (uint8_t)(fcs & 0xFFu);
    raw[raw_length++] = (uint8_t)(fcs >> 8u);

    wire[encoded_length++] = RET_HDLC_FLAG;
    for (index = 0u; index < raw_length; ++index) {
        if (!append_wire_octet(wire, wire_capacity, &encoded_length, raw[index])) {
            return false;
        }
    }
    if (encoded_length >= wire_capacity) {
        return false;
    }
    wire[encoded_length++] = RET_HDLC_FLAG;
    *wire_length = encoded_length;
    return true;
}

void ret_hdlc_decoder_init(ret_hdlc_decoder_t *decoder) {
    if (decoder != NULL) {
        memset(decoder, 0, sizeof(*decoder));
    }
}

static ret_hdlc_decoder_result_t finish_frame(ret_hdlc_decoder_t *decoder,
                                              ret_hdlc_frame_t *frame) {
    uint16_t received_fcs;
    uint16_t expected_fcs;
    uint16_t information_length;

    if (decoder->length < 4u) {
        return RET_HDLC_DECODER_TOO_SHORT;
    }

    received_fcs = (uint16_t)decoder->raw[decoder->length - 2u] |
                   (uint16_t)((uint16_t)decoder->raw[decoder->length - 1u] << 8u);
    expected_fcs = ret_hdlc_crc16_x25(decoder->raw, decoder->length - 2u);
    if (received_fcs != expected_fcs) {
        return RET_HDLC_DECODER_BAD_FCS;
    }

    information_length = (uint16_t)(decoder->length - 4u);
    frame->address = decoder->raw[0];
    frame->control = decoder->raw[1];
    frame->information_length = information_length;
    if (information_length != 0u) {
        memcpy(frame->information, &decoder->raw[2], information_length);
    }
    return RET_HDLC_DECODER_FRAME;
}

ret_hdlc_decoder_result_t ret_hdlc_decoder_push(ret_hdlc_decoder_t *decoder,
                                                const uint8_t octet,
                                                ret_hdlc_frame_t *frame) {
    ret_hdlc_decoder_result_t result = RET_HDLC_DECODER_NONE;

    if ((decoder == NULL) || (frame == NULL)) {
        return RET_HDLC_DECODER_NONE;
    }

    if (octet == RET_HDLC_FLAG) {
        if (decoder->in_frame && decoder->escaped) {
            result = RET_HDLC_DECODER_BAD_ESCAPE;
        } else if (decoder->in_frame && !decoder->dropping &&
                   (decoder->length != 0u)) {
            result = finish_frame(decoder, frame);
        }
        decoder->length = 0u;
        decoder->in_frame = true;
        decoder->escaped = false;
        decoder->dropping = false;
        return result;
    }

    if (!decoder->in_frame || decoder->dropping) {
        return RET_HDLC_DECODER_NONE;
    }

    if (decoder->escaped) {
        decoder->escaped = false;
        if (decoder->length >= RET_HDLC_MAX_RAW) {
            decoder->dropping = true;
            return RET_HDLC_DECODER_TOO_LONG;
        }
        decoder->raw[decoder->length++] = (uint8_t)(octet ^ RET_HDLC_ESCAPE_XOR);
    } else if (octet == RET_HDLC_ESCAPE) {
        decoder->escaped = true;
    } else {
        if (decoder->length >= RET_HDLC_MAX_RAW) {
            decoder->dropping = true;
            return RET_HDLC_DECODER_TOO_LONG;
        }
        decoder->raw[decoder->length++] = octet;
    }
    return RET_HDLC_DECODER_NONE;
}
