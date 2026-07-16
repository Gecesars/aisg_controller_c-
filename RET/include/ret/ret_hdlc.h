#ifndef RET_HDLC_H
#define RET_HDLC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RET_HDLC_FLAG 0x7Eu
#define RET_HDLC_ESCAPE 0x7Du
#define RET_HDLC_ESCAPE_XOR 0x20u

/* AISG 2.0 requires every secondary to support N >= 78 octets.  This
 * implementation supports a 260-octet unescaped frame (256 octets INFO). */
#define RET_HDLC_MAX_INFO 256u
#define RET_HDLC_MAX_RAW (RET_HDLC_MAX_INFO + 4u)
#define RET_HDLC_MAX_WIRE ((RET_HDLC_MAX_RAW * 2u) + 2u)

typedef struct {
    uint8_t address;
    uint8_t control;
    uint16_t information_length;
    uint8_t information[RET_HDLC_MAX_INFO];
} ret_hdlc_frame_t;

typedef enum {
    RET_HDLC_DECODER_NONE = 0,
    RET_HDLC_DECODER_FRAME,
    RET_HDLC_DECODER_BAD_FCS,
    RET_HDLC_DECODER_TOO_SHORT,
    RET_HDLC_DECODER_TOO_LONG,
    RET_HDLC_DECODER_BAD_ESCAPE
} ret_hdlc_decoder_result_t;

typedef struct {
    uint8_t raw[RET_HDLC_MAX_RAW];
    uint16_t length;
    bool in_frame;
    bool escaped;
    bool dropping;
} ret_hdlc_decoder_t;

uint16_t ret_hdlc_crc16_x25(const uint8_t *data, size_t length);

bool ret_hdlc_encode(const ret_hdlc_frame_t *frame,
                     uint8_t *wire,
                     size_t wire_capacity,
                     size_t *wire_length);

void ret_hdlc_decoder_init(ret_hdlc_decoder_t *decoder);

ret_hdlc_decoder_result_t ret_hdlc_decoder_push(ret_hdlc_decoder_t *decoder,
                                                uint8_t octet,
                                                ret_hdlc_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
