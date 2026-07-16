#include "ret/ret_protocol.h"

#include <limits.h>
#include <string.h>

#define RET_CONFIG_SCHEMA 0x00020001u

#define RET_ALL_STATIONS 0xFFu
#define RET_NO_STATION 0x00u

#define RET_CONTROL_PF 0x10u
#define RET_CONTROL_SNRM 0x83u
#define RET_CONTROL_DISC 0x43u
#define RET_CONTROL_XID 0xAFu
#define RET_CONTROL_UA 0x63u
#define RET_CONTROL_DM 0x0Fu
#define RET_CONTROL_FRMR 0x87u

#define RET_XID_FI 0x81u
#define RET_XID_GI_HDLC 0x80u
#define RET_XID_GI_USER 0xF0u

#define RET_PI_UID 1u
#define RET_PI_ADDRESS 2u
#define RET_PI_UID_MASK 3u
#define RET_PI_DEVICE_TYPE 4u
#define RET_PI_RELEASE_ID 5u
#define RET_PI_VENDOR_CODE 6u
#define RET_PI_RESET_DEVICE 7u
#define RET_PI_AISG_VERSION 20u
#define RET_PI_SUBSTANCE_VERSION 21u

#define RET_AISG_VERSION 2u
#define RET_3GPP_RELEASE 6u
#define RET_SUBSTANCE_VERSION 0u

#define RET_DEFAULT_INFO_LENGTH 74u
/* HAL_GetTick() is quantized to 1 ms. Scheduling at +4 guarantees that at
 * least 3.000 ms elapsed even when the closing flag arrived just before a
 * tick boundary. */
#define RET_MINIMUM_FRAME_DELAY_MS 4u
#define RET_MAXIMUM_RESPONSE_DELAY_MS 9u
#define RET_LINK_TIMEOUT_MS 180000u

#define RET_PROC_RESET_SOFTWARE 0x03u
#define RET_PROC_GET_ALARM_STATUS 0x04u
#define RET_PROC_GET_INFORMATION 0x05u
#define RET_PROC_CLEAR_ACTIVE_ALARMS 0x06u
#define RET_PROC_ALARM_INDICATION 0x07u
#define RET_PROC_SELF_TEST 0x0Au
#define RET_PROC_SET_DEVICE_DATA 0x0Eu
#define RET_PROC_GET_DEVICE_DATA 0x0Fu
#define RET_PROC_READ_USER_DATA 0x10u
#define RET_PROC_WRITE_USER_DATA 0x11u
#define RET_PROC_ALARM_SUBSCRIBE 0x12u
#define RET_PROC_CALIBRATE 0x31u
#define RET_PROC_SEND_CONFIGURATION 0x32u
#define RET_PROC_SET_TILT 0x33u
#define RET_PROC_GET_TILT 0x34u
#define RET_PROC_DOWNLOAD_START 0x40u
#define RET_PROC_DOWNLOAD_APPLICATION 0x41u
#define RET_PROC_DOWNLOAD_END 0x42u
#define RET_PROC_VENDOR_SPECIFIC 0x90u
#define RET_PROC_ANTENNA_CALIBRATE 0x80u
#define RET_PROC_ANTENNA_SET_TILT 0x81u
#define RET_PROC_ANTENNA_GET_TILT 0x82u
#define RET_PROC_ANTENNA_SET_DEVICE_DATA 0x83u
#define RET_PROC_ANTENNA_GET_DEVICE_DATA 0x84u
#define RET_PROC_ANTENNA_ALARM_INDICATION 0x85u
#define RET_PROC_ANTENNA_CLEAR_ALARMS 0x86u
#define RET_PROC_ANTENNA_GET_ALARMS 0x87u
#define RET_PROC_ANTENNA_COUNT 0x88u
#define RET_PROC_ANTENNA_SEND_CONFIGURATION 0x89u

#define RET_RC_OK 0x00u
#define RET_RC_MOTOR_JAM 0x02u
#define RET_RC_ACTUATOR_JAM 0x03u
#define RET_RC_BUSY 0x05u
#define RET_RC_CHECKSUM_ERROR 0x06u
#define RET_RC_FAIL 0x0Bu
#define RET_RC_NOT_CALIBRATED 0x0Eu
#define RET_RC_NOT_CONFIGURED 0x0Fu
#define RET_RC_HARDWARE_ERROR 0x11u
#define RET_RC_OUT_OF_RANGE 0x13u
#define RET_RC_UNKNOWN_PROCEDURE 0x19u
#define RET_RC_READ_ONLY 0x1Du
#define RET_RC_UNKNOWN_PARAMETER 0x1Eu
#define RET_RC_FORMAT_ERROR 0x24u
#define RET_RC_UNSUPPORTED_PROCEDURE 0x25u

typedef struct {
    uint8_t *bytes;
    size_t capacity;
    size_t length;
} byte_writer_t;

typedef struct {
    bool has_uid;
    uint8_t uid_length;
    const uint8_t *uid;
    bool has_mask;
    uint8_t mask_length;
    const uint8_t *mask;
    bool has_address;
    uint8_t address;
    bool has_device_type;
    uint8_t device_type;
    bool has_vendor;
    const uint8_t *vendor;
    bool reset;
    uint8_t uid_occurrences;
    uint8_t mask_occurrences;
    uint8_t parameter_count;
    bool only_scan_parameters;
} xid_user_parameters_t;

static uint32_t now_ms(const ret_device_t *device) {
    return (device->platform.millis != NULL)
               ? device->platform.millis(device->platform.context)
               : 0u;
}

static bool time_reached(const uint32_t now, const uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static size_t fixed_string_length(const char *text, const size_t capacity) {
    size_t length = 0u;
    while ((length < capacity) && (text[length] != '\0')) {
        ++length;
    }
    return length;
}

static void copy_text(char *destination,
                      const size_t destination_size,
                      const char *source) {
    size_t length;
    if ((destination == NULL) || (destination_size == 0u)) {
        return;
    }
    memset(destination, 0, destination_size);
    if (source == NULL) {
        return;
    }
    length = strlen(source);
    if (length > destination_size) {
        length = destination_size;
    }
    memcpy(destination, source, length);
}

static void copy_right_aligned(char *destination,
                               const size_t destination_size,
                               const char *source) {
    size_t length;
    memset(destination, 0, destination_size);
    length = strlen(source);
    if (length > destination_size) {
        source += length - destination_size;
        length = destination_size;
    }
    memcpy(destination + destination_size - length, source, length);
}

static void put_u16_le(uint8_t *destination, const uint16_t value) {
    destination[0] = (uint8_t)(value & 0xFFu);
    destination[1] = (uint8_t)(value >> 8u);
}

static uint16_t get_u16_le(const uint8_t *source) {
    return (uint16_t)source[0] | (uint16_t)((uint16_t)source[1] << 8u);
}

static int16_t get_i16_le(const uint8_t *source) {
    return (int16_t)get_u16_le(source);
}

static uint32_t get_u32_be(const uint8_t *source) {
    return ((uint32_t)source[0] << 24u) |
           ((uint32_t)source[1] << 16u) |
           ((uint32_t)source[2] << 8u) |
           (uint32_t)source[3];
}

static void put_u32_be(uint8_t *destination, const uint32_t value) {
    destination[0] = (uint8_t)(value >> 24u);
    destination[1] = (uint8_t)(value >> 16u);
    destination[2] = (uint8_t)(value >> 8u);
    destination[3] = (uint8_t)value;
}

static bool writer_put(byte_writer_t *writer, const uint8_t value) {
    if (writer->length >= writer->capacity) {
        return false;
    }
    writer->bytes[writer->length++] = value;
    return true;
}

static bool writer_bytes(byte_writer_t *writer,
                         const uint8_t *data,
                         const size_t length) {
    if ((length > (writer->capacity - writer->length)) ||
        ((data == NULL) && (length != 0u))) {
        return false;
    }
    if (length != 0u) {
        memcpy(&writer->bytes[writer->length], data, length);
        writer->length += length;
    }
    return true;
}

static bool writer_parameter(byte_writer_t *writer,
                             const uint8_t identifier,
                             const uint8_t *value,
                             const size_t length) {
    return (length <= UINT8_MAX) && writer_put(writer, identifier) &&
           writer_put(writer, (uint8_t)length) &&
           writer_bytes(writer, value, length);
}

static bool begin_group(byte_writer_t *writer,
                        const uint8_t group_identifier,
                        size_t *group_start) {
    *group_start = writer->length;
    return writer_put(writer, RET_XID_FI) &&
           writer_put(writer, group_identifier) && writer_put(writer, 0u);
}

static bool finish_group(byte_writer_t *writer, const size_t group_start) {
    const size_t group_length = writer->length - group_start - 3u;
    if (group_length > UINT8_MAX) {
        return false;
    }
    writer->bytes[group_start + 2u] = (uint8_t)group_length;
    return true;
}

void ret_config_set_defaults(ret_config_t *config) {
    ret_antenna_config_t *antenna;
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->schema_version = RET_CONFIG_SCHEMA;
    config->device_type = RET_DEVICE_TYPE_SINGLE;
    config->vendor_code[0] = 'T';
    config->vendor_code[1] = 'Y';
    config->unit_id_length = 16u;
    memcpy(config->unit_id, "RET0000000000001", 16u);
    copy_text(config->product_number, sizeof(config->product_number), "ATC-RET");
    copy_text(config->serial_number, sizeof(config->serial_number), "RET0000000000001");
    copy_text(config->hardware_version, sizeof(config->hardware_version), "STM32F405");
    copy_text(config->software_version, sizeof(config->software_version), "1.0.0");
    config->antenna_count = 1u;

    antenna = &config->antennas[0];
    copy_right_aligned(antenna->model, sizeof(antenna->model), "ATC-RET-PANEL");
    copy_right_aligned(antenna->serial, sizeof(antenna->serial), "ANT0000000000001");
    antenna->operating_bands = 0x0010u;
    antenna->beamwidth[0] = 70u;
    antenna->gain_tenths_db[0] = 150u;
    antenna->minimum_tilt_tenths = 0;
    antenna->maximum_tilt_tenths = 100;
    antenna->current_tilt_tenths = 20;
    antenna->configured = true;
    antenna->calibrated = false;
}

bool ret_config_is_valid(const ret_config_t *config) {
    size_t information_data_length;
    uint8_t index;

    if ((config == NULL) || (config->schema_version != RET_CONFIG_SCHEMA) ||
        ((config->device_type != RET_DEVICE_TYPE_SINGLE) &&
         (config->device_type != RET_DEVICE_TYPE_MULTI)) ||
        (config->unit_id_length == 0u) || (config->unit_id_length > 17u) ||
        (config->antenna_count == 0u) ||
        (config->antenna_count > RET_MAX_ANTENNAS) ||
        ((config->device_type == RET_DEVICE_TYPE_SINGLE) &&
         (config->antenna_count != 1u))) {
        return false;
    }

    information_data_length = 5u +
        fixed_string_length(config->product_number, sizeof(config->product_number)) +
        fixed_string_length(config->serial_number, sizeof(config->serial_number)) +
        fixed_string_length(config->hardware_version, sizeof(config->hardware_version)) +
        fixed_string_length(config->software_version, sizeof(config->software_version));
    if (information_data_length > (RET_DEFAULT_INFO_LENGTH - 3u)) {
        return false;
    }

    for (index = 0u; index < config->antenna_count; ++index) {
        const ret_antenna_config_t *antenna = &config->antennas[index];
        if ((antenna->minimum_tilt_tenths > antenna->maximum_tilt_tenths) ||
            (antenna->current_tilt_tenths < antenna->minimum_tilt_tenths) ||
            (antenna->current_tilt_tenths > antenna->maximum_tilt_tenths) ||
            (antenna->vendor_configuration_length > RET_VENDOR_CONFIG_SIZE)) {
            return false;
        }
    }
    return true;
}

static bool persist(ret_device_t *device) {
    return (device->platform.storage_save == NULL) ||
           device->platform.storage_save(device->platform.context, &device->config);
}

static void clear_runtime_state(ret_device_t *device,
                                const bool clear_address,
                                const bool reset_application) {
    device->receive_sequence = 0u;
    device->send_sequence = 0u;
    device->remote_ready = true;
    device->outstanding_i_frame_valid = false;
    device->pending_link_response_valid = false;
    device->reset_after_i_ack = false;
    device->reset_after_transmit = false;
    device->l7_queue_head = 0u;
    device->l7_queue_count = 0u;
    if (reset_application) {
        if ((device->tcc.type != RET_TCC_NONE) &&
            (device->platform.motor_stop != NULL)) {
            device->platform.motor_stop(device->platform.context);
        }
        memset(device->configuration_transfer_active, 0,
               sizeof(device->configuration_transfer_active));
        device->alarm_subscribed = false;
        memset(device->common_alarms, 0, sizeof(device->common_alarms));
        memset(device->antenna_alarms, 0, sizeof(device->antenna_alarms));
        memset(&device->tcc, 0, sizeof(device->tcc));
    }
    if (clear_address) {
        device->address = RET_NO_STATION;
        device->link_state = RET_LINK_NO_ADDRESS;
    }
}

void ret_device_init(ret_device_t *device,
                     const ret_platform_t *platform,
                     const ret_config_t *fallback_config) {
    ret_config_t fallback;
    bool loaded = false;

    if ((device == NULL) || (platform == NULL)) {
        return;
    }
    memset(device, 0, sizeof(*device));
    device->platform = *platform;

    if (device->platform.storage_load != NULL) {
        loaded = device->platform.storage_load(device->platform.context,
                                               &device->config);
    }
    if (!loaded || !ret_config_is_valid(&device->config)) {
        if ((fallback_config != NULL) && ret_config_is_valid(fallback_config)) {
            device->config = *fallback_config;
        } else {
            ret_config_set_defaults(&fallback);
            device->config = fallback;
        }
        (void)persist(device);
    }

    ret_hdlc_decoder_init(&device->decoder);
    device->maximum_tx_information = RET_DEFAULT_INFO_LENGTH;
    device->maximum_rx_information = RET_DEFAULT_INFO_LENGTH;
    clear_runtime_state(device, true, true);
    device->last_addressed_frame_ms = now_ms(device);
}

static bool queue_l7(ret_device_t *device,
                     const uint8_t procedure,
                     const uint8_t *data,
                     const uint16_t data_length) {
    uint8_t slot;
    ret_l7_message_t *message;
    const uint16_t message_length = (uint16_t)(data_length + 3u);

    if ((device->l7_queue_count >= RET_L7_QUEUE_DEPTH) ||
        (message_length > RET_HDLC_MAX_INFO) ||
        (message_length > device->maximum_tx_information)) {
        return false;
    }
    slot = (uint8_t)((device->l7_queue_head + device->l7_queue_count) %
                     RET_L7_QUEUE_DEPTH);
    message = &device->l7_queue[slot];
    message->data[0] = procedure;
    put_u16_le(&message->data[1], data_length);
    if (data_length != 0u) {
        memcpy(&message->data[3], data, data_length);
    }
    message->length = message_length;
    ++device->l7_queue_count;
    return true;
}

static bool queue_ok(ret_device_t *device,
                     const uint8_t procedure,
                     const uint8_t *extra,
                     const uint16_t extra_length) {
    uint8_t data[RET_HDLC_MAX_INFO - 3u];
    if (extra_length > (sizeof(data) - 1u)) {
        return false;
    }
    data[0] = RET_RC_OK;
    if (extra_length != 0u) {
        memcpy(&data[1], extra, extra_length);
    }
    return queue_l7(device, procedure, data, (uint16_t)(extra_length + 1u));
}

static bool queue_error(ret_device_t *device,
                        const uint8_t procedure,
                        const uint8_t reason) {
    const uint8_t data[2] = {RET_RC_FAIL, reason};
    return queue_l7(device, procedure, data, sizeof(data));
}

static bool queue_multi_ok(ret_device_t *device,
                           const uint8_t procedure,
                           const uint8_t antenna_number,
                           const uint8_t *extra,
                           const uint16_t extra_length) {
    uint8_t data[RET_HDLC_MAX_INFO - 3u];
    if (extra_length > (sizeof(data) - 2u)) {
        return false;
    }
    data[0] = antenna_number;
    data[1] = RET_RC_OK;
    if (extra_length != 0u) {
        memcpy(&data[2], extra, extra_length);
    }
    return queue_l7(device, procedure, data, (uint16_t)(extra_length + 2u));
}

static bool queue_multi_error(ret_device_t *device,
                              const uint8_t procedure,
                              const uint8_t antenna_number,
                              const uint8_t reason) {
    const uint8_t data[3] = {antenna_number, RET_RC_FAIL, reason};
    return queue_l7(device, procedure, data, sizeof(data));
}

static ret_alarm_t *find_alarm(ret_alarm_t alarms[RET_MAX_ALARMS],
                               const uint8_t code,
                               const bool create) {
    size_t index;
    ret_alarm_t *empty = NULL;
    for (index = 0u; index < RET_MAX_ALARMS; ++index) {
        if (alarms[index].code == code) {
            return &alarms[index];
        }
        if ((empty == NULL) && (alarms[index].code == 0u)) {
            empty = &alarms[index];
        }
    }
    if (create && (empty != NULL)) {
        empty->code = code;
    }
    return create ? empty : NULL;
}

void ret_device_set_common_alarm(ret_device_t *device,
                                 const uint8_t alarm_code,
                                 const bool active) {
    ret_alarm_t *alarm;
    if ((device == NULL) || (alarm_code == 0u)) {
        return;
    }
    alarm = find_alarm(device->common_alarms, alarm_code, true);
    if ((alarm != NULL) && (alarm->active != active)) {
        alarm->active = active;
        alarm->changed = true;
    }
}

void ret_device_set_antenna_alarm(ret_device_t *device,
                                  const uint8_t antenna_number,
                                  const uint8_t alarm_code,
                                  const bool active) {
    ret_alarm_t *alarm;
    if ((device == NULL) || (antenna_number == 0u) ||
        (antenna_number > device->config.antenna_count) ||
        (alarm_code == 0u)) {
        return;
    }
    alarm = find_alarm(device->antenna_alarms[antenna_number - 1u],
                       alarm_code, true);
    if ((alarm != NULL) && (alarm->active != active)) {
        alarm->active = active;
        alarm->changed = true;
    }
}

static void mark_active_alarms_changed(ret_alarm_t alarms[RET_MAX_ALARMS]) {
    size_t index;
    for (index = 0u; index < RET_MAX_ALARMS; ++index) {
        if (alarms[index].active) {
            alarms[index].changed = true;
        }
    }
}

static uint16_t active_alarm_codes(const ret_alarm_t alarms[RET_MAX_ALARMS],
                                   uint8_t *output,
                                   const uint16_t capacity) {
    size_t index;
    uint16_t length = 0u;
    for (index = 0u; (index < RET_MAX_ALARMS) && (length < capacity); ++index) {
        if (alarms[index].active) {
            output[length++] = alarms[index].code;
        }
    }
    return length;
}

static void queue_changed_alarm_set(ret_device_t *device,
                                    const uint8_t procedure,
                                    const uint8_t antenna_number,
                                    ret_alarm_t alarms[RET_MAX_ALARMS]) {
    uint8_t data[(RET_MAX_ALARMS * 2u) + 1u];
    uint16_t length = 0u;
    size_t index;

    if ((procedure == RET_PROC_ANTENNA_ALARM_INDICATION)) {
        data[length++] = antenna_number;
    }
    for (index = 0u; index < RET_MAX_ALARMS; ++index) {
        if (alarms[index].changed) {
            data[length++] = alarms[index].code;
            data[length++] = alarms[index].active ? 1u : 0u;
        }
    }
    if (length > ((procedure == RET_PROC_ANTENNA_ALARM_INDICATION) ? 1u : 0u)) {
        if (queue_l7(device, procedure, data, length)) {
            for (index = 0u; index < RET_MAX_ALARMS; ++index) {
                alarms[index].changed = false;
            }
        }
    }
}

static void queue_alarm_indications(ret_device_t *device) {
    uint8_t antenna;
    if (!device->alarm_subscribed) {
        return;
    }
    queue_changed_alarm_set(device, RET_PROC_ALARM_INDICATION, 0u,
                            device->common_alarms);
    if (device->config.device_type == RET_DEVICE_TYPE_MULTI) {
        for (antenna = 1u; antenna <= device->config.antenna_count; ++antenna) {
            queue_changed_alarm_set(device, RET_PROC_ANTENNA_ALARM_INDICATION,
                                    antenna,
                                    device->antenna_alarms[antenna - 1u]);
        }
    }
}

static void schedule_link_response(ret_device_t *device,
                                   const ret_hdlc_frame_t *frame,
                                   uint32_t delay_ms) {
    if (delay_ms < RET_MINIMUM_FRAME_DELAY_MS) {
        delay_ms = RET_MINIMUM_FRAME_DELAY_MS;
    }
    if (delay_ms > RET_MAXIMUM_RESPONSE_DELAY_MS) {
        delay_ms = RET_MAXIMUM_RESPONSE_DELAY_MS;
    }
    device->pending_link_response = *frame;
    device->pending_link_response_valid = true;
    device->pending_link_response_at_ms = now_ms(device) + delay_ms;
}

static ret_hdlc_frame_t make_supervisory(const ret_device_t *device,
                                         const uint8_t type) {
    ret_hdlc_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.address = device->address;
    frame.control = (uint8_t)(((device->receive_sequence & 7u) << 5u) |
                              RET_CONTROL_PF | type);
    return frame;
}

static void acknowledge_transmit(ret_device_t *device, const uint8_t nr) {
    if (!device->outstanding_i_frame_valid) {
        return;
    }
    if ((nr & 7u) == (device->send_sequence & 7u)) {
        device->outstanding_i_frame_valid = false;
        if (device->reset_after_i_ack) {
            device->reset_after_i_ack = false;
            if (device->platform.application_reset != NULL) {
                device->platform.application_reset(device->platform.context);
            }
        }
    }
}

static bool acknowledgement_is_valid(const ret_device_t *device,
                                     const uint8_t nr) {
    const uint8_t acknowledged = (uint8_t)(nr & 7u);
    if (device->outstanding_i_frame_valid) {
        return (acknowledged == (device->send_sequence & 7u)) ||
               (acknowledged == ((device->send_sequence + 7u) & 7u));
    }
    return acknowledged == (device->send_sequence & 7u);
}

static void schedule_frame_reject(ret_device_t *device,
                                  const uint8_t rejected_control,
                                  const uint8_t reason_flags) {
    ret_hdlc_frame_t reject;
    memset(&reject, 0, sizeof(reject));
    reject.address = device->address;
    reject.control = (uint8_t)(RET_CONTROL_FRMR | RET_CONTROL_PF);
    reject.information[0] = rejected_control;
    reject.information[1] =
        (uint8_t)(((device->receive_sequence & 7u) << 5u) |
                  ((device->send_sequence & 7u) << 1u));
    reject.information[2] = reason_flags;
    reject.information_length = 3u;
    schedule_link_response(device, &reject, RET_MINIMUM_FRAME_DELAY_MS);
}

static bool make_next_i_frame(ret_device_t *device, ret_hdlc_frame_t *frame) {
    ret_l7_message_t *message;
    if ((device->l7_queue_count == 0u) || device->outstanding_i_frame_valid ||
        !device->remote_ready) {
        return false;
    }
    message = &device->l7_queue[device->l7_queue_head];
    memset(frame, 0, sizeof(*frame));
    frame->address = device->address;
    frame->control = (uint8_t)(((device->receive_sequence & 7u) << 5u) |
                               RET_CONTROL_PF |
                               ((device->send_sequence & 7u) << 1u));
    frame->information_length = message->length;
    memcpy(frame->information, message->data, message->length);
    device->l7_queue_head = (uint8_t)((device->l7_queue_head + 1u) %
                                      RET_L7_QUEUE_DEPTH);
    --device->l7_queue_count;
    device->outstanding_i_frame = *frame;
    device->outstanding_i_frame_valid = true;
    device->send_sequence = (uint8_t)((device->send_sequence + 1u) & 7u);
    return true;
}

static void respond_to_poll(ret_device_t *device) {
    ret_hdlc_frame_t response;
    queue_alarm_indications(device);
    if (!device->remote_ready) {
        response = make_supervisory(device, 0x01u);
    } else if (device->outstanding_i_frame_valid) {
        response = device->outstanding_i_frame;
    } else if (!make_next_i_frame(device, &response)) {
        response = make_supervisory(device, 0x01u);
    }
    schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
}

static size_t make_unique_id(const ret_device_t *device,
                             uint8_t unique_id[19],
                             const bool padded) {
    const size_t unit_length = device->config.unit_id_length;
    unique_id[0] = (uint8_t)device->config.vendor_code[0];
    unique_id[1] = (uint8_t)device->config.vendor_code[1];
    if (padded) {
        const size_t zero_count = 17u - unit_length;
        memset(&unique_id[2], 0, zero_count);
        memcpy(&unique_id[2u + zero_count], device->config.unit_id, unit_length);
        return 19u;
    }
    memcpy(&unique_id[2], device->config.unit_id, unit_length);
    return unit_length + 2u;
}

static bool assignment_uid_matches(const ret_device_t *device,
                                   const uint8_t *requested,
                                   const size_t requested_length) {
    uint8_t actual[19];
    const size_t actual_length = make_unique_id(device, actual, false);
    if (requested_length > actual_length) {
        return false;
    }
    return (requested_length == 0u) ||
           (memcmp(&actual[actual_length - requested_length], requested,
                   requested_length) == 0);
}

static bool scan_uid_matches(const ret_device_t *device,
                             const uint8_t *requested,
                             const uint8_t *mask,
                             const size_t length) {
    uint8_t actual[19];
    size_t index;
    size_t left_length;
    size_t right_length;
    if (length > sizeof(actual)) {
        return false;
    }
    (void)make_unique_id(device, actual, true);
    left_length = (length < 2u) ? length : 2u;
    right_length = length - left_length;
    for (index = 0u; index < left_length; ++index) {
        if ((uint8_t)(actual[index] & mask[index]) != requested[index]) {
            return false;
        }
    }
    for (index = 0u; index < right_length; ++index) {
        const size_t actual_index = sizeof(actual) - right_length + index;
        const size_t request_index = left_length + index;
        if ((uint8_t)(actual[actual_index] & mask[request_index]) !=
            requested[request_index]) {
            return false;
        }
    }
    return true;
}

static bool parse_user_parameters(const uint8_t *parameters,
                                  const size_t length,
                                  xid_user_parameters_t *parsed) {
    size_t offset = 0u;
    memset(parsed, 0, sizeof(*parsed));
    parsed->only_scan_parameters = true;
    while (offset < length) {
        uint8_t identifier;
        uint8_t parameter_length;
        const uint8_t *value;
        if ((length - offset) < 2u) {
            return false;
        }
        identifier = parameters[offset++];
        parameter_length = parameters[offset++];
        if (parameter_length > (length - offset)) {
            return false;
        }
        value = &parameters[offset];
        ++parsed->parameter_count;
        switch (identifier) {
        case RET_PI_UID:
            ++parsed->uid_occurrences;
            parsed->has_uid = true;
            parsed->uid_length = parameter_length;
            parsed->uid = value;
            break;
        case RET_PI_ADDRESS:
            parsed->only_scan_parameters = false;
            if (parameter_length != 1u) {
                return false;
            }
            parsed->has_address = true;
            parsed->address = value[0];
            break;
        case RET_PI_UID_MASK:
            ++parsed->mask_occurrences;
            parsed->has_mask = true;
            parsed->mask_length = parameter_length;
            parsed->mask = value;
            break;
        case RET_PI_DEVICE_TYPE:
            parsed->only_scan_parameters = false;
            if (parameter_length != 1u) {
                return false;
            }
            parsed->has_device_type = true;
            parsed->device_type = value[0];
            break;
        case RET_PI_VENDOR_CODE:
            parsed->only_scan_parameters = false;
            if (parameter_length != 2u) {
                return false;
            }
            parsed->has_vendor = true;
            parsed->vendor = value;
            break;
        case RET_PI_RESET_DEVICE:
            parsed->only_scan_parameters = false;
            if (parameter_length != 0u) {
                return false;
            }
            parsed->reset = true;
            break;
        default:
            parsed->only_scan_parameters = false;
            break;
        }
        offset += parameter_length;
    }
    return true;
}

static bool write_identity_group(ret_device_t *device,
                                 byte_writer_t *writer,
                                 const bool include_vendor) {
    uint8_t unique_id[19];
    const size_t unique_length = make_unique_id(device, unique_id, false);
    const uint8_t type = device->config.device_type;
    size_t group;
    if (!begin_group(writer, RET_XID_GI_USER, &group) ||
        !writer_parameter(writer, RET_PI_UID, unique_id, unique_length) ||
        !writer_parameter(writer, RET_PI_DEVICE_TYPE, &type, 1u)) {
        return false;
    }
    if (include_vendor &&
        !writer_parameter(writer, RET_PI_VENDOR_CODE,
                          (const uint8_t *)device->config.vendor_code, 2u)) {
        return false;
    }
    return finish_group(writer, group);
}

static bool write_negotiated_group(ret_device_t *device,
                                   byte_writer_t *writer,
                                   const uint8_t group_identifier,
                                   const uint8_t *parameters,
                                   const size_t parameter_length,
                                   uint16_t *maximum_tx_information,
                                   uint16_t *maximum_rx_information) {
    size_t group;
    size_t offset = 0u;
    size_t parameter_count = 0u;
    if (!begin_group(writer, group_identifier, &group)) {
        return false;
    }
    while (offset < parameter_length) {
        uint8_t identifier;
        uint8_t length;
        const uint8_t *value;
        uint8_t response[4];
        size_t response_length = 0u;
        if ((parameter_length - offset) < 2u) {
            return false;
        }
        identifier = parameters[offset++];
        length = parameters[offset++];
        if (length > (parameter_length - offset)) {
            return false;
        }
        value = &parameters[offset];

        if (group_identifier == RET_XID_GI_HDLC) {
            if (((identifier == 5u) || (identifier == 6u)) && (length == 4u)) {
                uint32_t bits = get_u32_be(value);
                const uint32_t maximum_bits = RET_HDLC_MAX_INFO * 8u;
                if (bits > maximum_bits) {
                    bits = maximum_bits;
                }
                if (bits < (RET_DEFAULT_INFO_LENGTH * 8u)) {
                    bits = RET_DEFAULT_INFO_LENGTH * 8u;
                }
                put_u32_be(response, bits);
                response_length = 4u;
                if (identifier == 5u) {
                    *maximum_tx_information = (uint16_t)(bits / 8u);
                } else {
                    *maximum_rx_information = (uint16_t)(bits / 8u);
                }
            } else if (((identifier == 7u) || (identifier == 8u)) &&
                       (length == 1u)) {
                response[0] = 1u;
                response_length = 1u;
            }
        } else if (group_identifier == RET_XID_GI_USER) {
            if ((identifier == RET_PI_RELEASE_ID) && (length == 1u)) {
                response[0] = (value[0] < RET_3GPP_RELEASE) ? value[0]
                                                            : RET_3GPP_RELEASE;
                response_length = 1u;
            } else if ((identifier == RET_PI_AISG_VERSION) && (length == 1u)) {
                response[0] = RET_AISG_VERSION;
                response_length = 1u;
            } else if ((identifier == RET_PI_SUBSTANCE_VERSION) &&
                       (length == 2u) &&
                       (value[0] == device->config.device_type)) {
                response[0] = device->config.device_type;
                response[1] = RET_SUBSTANCE_VERSION;
                response_length = 2u;
            }
        }
        if (response_length != 0u) {
            if (!writer_parameter(writer, identifier, response, response_length)) {
                return false;
            }
            ++parameter_count;
        }
        offset += length;
    }
    if (parameter_count == 0u) {
        writer->length = group;
        return true;
    }
    return finish_group(writer, group);
}

static void process_xid(ret_device_t *device, const ret_hdlc_frame_t *request) {
    ret_hdlc_frame_t response;
    byte_writer_t writer;
    size_t offset = 0u;
    bool has_response_parameters = false;
    bool scan = false;
    bool assignment = false;
    bool assignment_matches = true;
    bool reset = false;
    uint8_t assigned_address = RET_NO_STATION;
    xid_user_parameters_t user = {0};
    uint8_t group_count = 0u;
    bool scan_shape_valid = false;
    bool reset_shape_valid = false;
    uint16_t proposed_maximum_tx = device->maximum_tx_information;
    uint16_t proposed_maximum_rx = device->maximum_rx_information;

    memset(&response, 0, sizeof(response));
    response.address = device->address;
    response.control = (uint8_t)(RET_CONTROL_XID | RET_CONTROL_PF);
    writer.bytes = response.information;
    writer.capacity = sizeof(response.information);
    writer.length = 0u;

    while (offset < request->information_length) {
        uint8_t format;
        uint8_t group;
        uint8_t group_length;
        const uint8_t *parameters;
        if ((request->information_length - offset) < 3u) {
            return;
        }
        format = request->information[offset++];
        group = request->information[offset++];
        group_length = request->information[offset++];
        if ((format != RET_XID_FI) ||
            (group_length > (request->information_length - offset))) {
            return;
        }
        parameters = &request->information[offset];
        ++group_count;
        if (group == RET_XID_GI_USER) {
            xid_user_parameters_t current;
            if (!parse_user_parameters(parameters, group_length, &current)) {
                return;
            }
            if (current.has_mask) {
                if (!current.has_uid ||
                    (current.uid_length != current.mask_length)) {
                    return;
                }
                scan = true;
                user = current;
                scan_shape_valid = current.only_scan_parameters &&
                                   (current.parameter_count == 2u) &&
                                   (current.uid_occurrences == 1u) &&
                                   (current.mask_occurrences == 1u);
            }
            if (current.has_address) {
                assignment = true;
                assigned_address = current.address;
                if ((assigned_address == RET_NO_STATION) ||
                    (assigned_address == RET_ALL_STATIONS)) {
                    assignment_matches = false;
                }
                if (current.has_uid &&
                    !assignment_uid_matches(device, current.uid,
                                            current.uid_length)) {
                    assignment_matches = false;
                }
                if (current.has_device_type &&
                    (current.device_type != device->config.device_type)) {
                    assignment_matches = false;
                }
                if (current.has_vendor &&
                    (memcmp(current.vendor, device->config.vendor_code, 2u) != 0)) {
                    assignment_matches = false;
                }
            }
            reset = reset || current.reset;
            if (current.reset) {
                reset_shape_valid = (current.parameter_count == 1u);
            }
        }
        {
            const size_t previous_length = writer.length;
            if (!write_negotiated_group(device, &writer, group, parameters,
                                        group_length, &proposed_maximum_tx,
                                        &proposed_maximum_rx)) {
                return;
            }
            has_response_parameters = has_response_parameters ||
                                      (writer.length != previous_length);
        }
        offset += group_length;
    }

    if ((scan && (!scan_shape_valid || (group_count != 1u))) ||
        (reset && (!reset_shape_valid || (group_count != 1u)))) {
        return;
    }

    if (reset) {
        if (request->address == RET_ALL_STATIONS) {
            if (device->platform.system_reset != NULL) {
                device->platform.system_reset(device->platform.context);
            }
            return;
        }
        response.address = device->address;
        response.information_length = (uint16_t)writer.length;
        device->reset_after_transmit = true;
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
        return;
    }

    if (scan) {
        if ((device->link_state != RET_LINK_NO_ADDRESS) ||
            !scan_uid_matches(device, user.uid, user.mask, user.uid_length)) {
            return;
        }
        writer.length = 0u;
        if (!write_identity_group(device, &writer, true)) {
            return;
        }
        response.address = RET_NO_STATION;
        response.information_length = (uint16_t)writer.length;
        schedule_link_response(device, &response,
                               RET_MINIMUM_FRAME_DELAY_MS +
                               (device->config.unit_id[device->config.unit_id_length - 1u] % 7u));
        return;
    }

    if (assignment) {
        if (!assignment_matches || (device->link_state != RET_LINK_NO_ADDRESS)) {
            return;
        }
        device->maximum_tx_information = proposed_maximum_tx;
        device->maximum_rx_information = proposed_maximum_rx;
        device->address = assigned_address;
        device->link_state = RET_LINK_ADDRESS_ASSIGNED;
        device->last_addressed_frame_ms = now_ms(device);
        writer.length = 0u;
        if (!write_identity_group(device, &writer, false)) {
            return;
        }
        response.address = device->address;
        response.information_length = (uint16_t)writer.length;
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
        return;
    }

    if (has_response_parameters && (request->address == device->address) &&
        (device->link_state != RET_LINK_NO_ADDRESS)) {
        device->maximum_tx_information = proposed_maximum_tx;
        device->maximum_rx_information = proposed_maximum_rx;
        response.address = device->address;
        response.information_length = (uint16_t)writer.length;
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
    }
}

static bool procedure_is_multi(const uint8_t procedure) {
    return (procedure >= RET_PROC_ANTENNA_CALIBRATE) &&
           (procedure <= RET_PROC_ANTENNA_SEND_CONFIGURATION);
}

static bool procedure_is_single_specific(const uint8_t procedure) {
    return (procedure == RET_PROC_SET_DEVICE_DATA) ||
           (procedure == RET_PROC_GET_DEVICE_DATA) ||
           (procedure == RET_PROC_CALIBRATE) ||
           (procedure == RET_PROC_SEND_CONFIGURATION) ||
           (procedure == RET_PROC_SET_TILT) ||
           (procedure == RET_PROC_GET_TILT);
}

static bool procedure_allowed_during_tcc(const uint8_t procedure) {
    return (procedure == RET_PROC_RESET_SOFTWARE) ||
           (procedure == RET_PROC_GET_ALARM_STATUS) ||
           (procedure == RET_PROC_GET_INFORMATION) ||
           (procedure == RET_PROC_GET_TILT) ||
           (procedure == RET_PROC_ANTENNA_GET_TILT) ||
           (procedure == RET_PROC_GET_DEVICE_DATA) ||
           (procedure == RET_PROC_ANTENNA_GET_DEVICE_DATA) ||
           (procedure == RET_PROC_ANTENNA_GET_ALARMS);
}

static bool valid_antenna(const ret_device_t *device, const uint8_t number) {
    return (number != 0u) && (number <= device->config.antenna_count);
}

static void respond_information(ret_device_t *device) {
    uint8_t data[RET_DEFAULT_INFO_LENGTH];
    size_t length = 0u;
    const char *values[4] = {device->config.product_number,
                             device->config.serial_number,
                             device->config.hardware_version,
                             device->config.software_version};
    const size_t capacities[4] = {sizeof(device->config.product_number),
                                  sizeof(device->config.serial_number),
                                  sizeof(device->config.hardware_version),
                                  sizeof(device->config.software_version)};
    size_t index;
    data[length++] = RET_RC_OK;
    for (index = 0u; index < 4u; ++index) {
        const size_t value_length = fixed_string_length(values[index], capacities[index]);
        data[length++] = (uint8_t)value_length;
        memcpy(&data[length], values[index], value_length);
        length += value_length;
    }
    (void)queue_l7(device, RET_PROC_GET_INFORMATION, data, (uint16_t)length);
}

static void respond_alarm_status(ret_device_t *device,
                                 const uint8_t procedure,
                                 const uint8_t antenna_number) {
    uint8_t codes[RET_MAX_ALARMS];
    uint16_t count;
    if (antenna_number == 0u) {
        count = active_alarm_codes(device->common_alarms, codes, sizeof(codes));
        (void)queue_ok(device, procedure, codes, count);
    } else if (valid_antenna(device, antenna_number)) {
        count = active_alarm_codes(device->antenna_alarms[antenna_number - 1u],
                                   codes, sizeof(codes));
        (void)queue_multi_ok(device, procedure, antenna_number, codes, count);
    } else {
        (void)queue_multi_error(device, procedure, antenna_number,
                                RET_RC_FORMAT_ERROR);
    }
}

static void clear_alarm_set(ret_alarm_t alarms[RET_MAX_ALARMS]) {
    size_t index;
    for (index = 0u; index < RET_MAX_ALARMS; ++index) {
        if (alarms[index].active) {
            alarms[index].active = false;
            alarms[index].changed = true;
        }
    }
}

static bool field_value(const ret_antenna_config_t *antenna,
                        const uint8_t field,
                        uint8_t *value,
                        uint16_t *length) {
    size_t index;
    switch (field) {
    case 0x01u:
        memcpy(value, antenna->model, sizeof(antenna->model));
        *length = sizeof(antenna->model);
        return true;
    case 0x02u:
        memcpy(value, antenna->serial, sizeof(antenna->serial));
        *length = sizeof(antenna->serial);
        return true;
    case 0x03u:
        put_u16_le(value, antenna->operating_bands);
        *length = 2u;
        return true;
    case 0x04u:
        for (index = 0u; index < 4u; ++index) {
            put_u16_le(&value[index * 2u], antenna->beamwidth[index]);
        }
        *length = 8u;
        return true;
    case 0x05u:
        memcpy(value, antenna->gain_tenths_db, sizeof(antenna->gain_tenths_db));
        *length = sizeof(antenna->gain_tenths_db);
        return true;
    case 0x06u:
        put_u16_le(value, (uint16_t)antenna->maximum_tilt_tenths);
        *length = 2u;
        return true;
    case 0x07u:
        put_u16_le(value, (uint16_t)antenna->minimum_tilt_tenths);
        *length = 2u;
        return true;
    case 0x21u:
        memcpy(value, antenna->installation_date, sizeof(antenna->installation_date));
        *length = sizeof(antenna->installation_date);
        return true;
    case 0x22u:
        memcpy(value, antenna->installer_id, sizeof(antenna->installer_id));
        *length = sizeof(antenna->installer_id);
        return true;
    case 0x23u:
        memcpy(value, antenna->base_station_id, sizeof(antenna->base_station_id));
        *length = sizeof(antenna->base_station_id);
        return true;
    case 0x24u:
        memcpy(value, antenna->sector_id, sizeof(antenna->sector_id));
        *length = sizeof(antenna->sector_id);
        return true;
    case 0x25u:
        put_u16_le(value, antenna->bearing);
        *length = 2u;
        return true;
    case 0x26u:
        put_u16_le(value, (uint16_t)antenna->mechanical_tilt_tenths);
        *length = 2u;
        return true;
    default:
        return false;
    }
}

static uint8_t set_field_value(ret_antenna_config_t *antenna,
                               const uint8_t field,
                               const uint8_t *value,
                               const uint16_t length) {
    void *destination = NULL;
    uint16_t expected = 0u;
    switch (field) {
    case 0x01u:
    case 0x02u:
    case 0x03u:
    case 0x04u:
    case 0x05u:
    case 0x06u:
    case 0x07u:
        return RET_RC_READ_ONLY;
    case 0x21u:
        destination = antenna->installation_date;
        expected = sizeof(antenna->installation_date);
        break;
    case 0x22u:
        destination = antenna->installer_id;
        expected = sizeof(antenna->installer_id);
        break;
    case 0x23u:
        destination = antenna->base_station_id;
        expected = sizeof(antenna->base_station_id);
        break;
    case 0x24u:
        destination = antenna->sector_id;
        expected = sizeof(antenna->sector_id);
        break;
    case 0x25u:
        if (length != 2u) {
            return RET_RC_FORMAT_ERROR;
        }
        antenna->bearing = get_u16_le(value);
        return RET_RC_OK;
    case 0x26u:
        if (length != 2u) {
            return RET_RC_FORMAT_ERROR;
        }
        antenna->mechanical_tilt_tenths = get_i16_le(value);
        return RET_RC_OK;
    default:
        return RET_RC_UNKNOWN_PARAMETER;
    }
    if (length != expected) {
        return RET_RC_FORMAT_ERROR;
    }
    memcpy(destination, value, length);
    return RET_RC_OK;
}

static void get_device_data(ret_device_t *device,
                            const uint8_t procedure,
                            const uint8_t antenna_number,
                            const uint8_t field) {
    uint8_t value[32];
    uint16_t length = 0u;
    if (!valid_antenna(device, antenna_number)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        }
        return;
    }
    if (!field_value(&device->config.antennas[antenna_number - 1u], field,
                     value, &length)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_UNKNOWN_PARAMETER);
        } else {
            (void)queue_error(device, procedure, RET_RC_UNKNOWN_PARAMETER);
        }
        return;
    }
    if (procedure_is_multi(procedure)) {
        (void)queue_multi_ok(device, procedure, antenna_number, value, length);
    } else {
        (void)queue_ok(device, procedure, value, length);
    }
}

static void set_device_data(ret_device_t *device,
                            const uint8_t procedure,
                            const uint8_t antenna_number,
                            const uint8_t field,
                            const uint8_t *value,
                            const uint16_t length) {
    uint8_t result;
    if (!valid_antenna(device, antenna_number)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        }
        return;
    }
    result = set_field_value(&device->config.antennas[antenna_number - 1u],
                             field, value, length);
    if (result != RET_RC_OK) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number, result);
        } else {
            (void)queue_error(device, procedure, result);
        }
        return;
    }
    if (!persist(device)) {
        result = RET_RC_HARDWARE_ERROR;
    }
    if (result == RET_RC_OK) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_ok(device, procedure, antenna_number, NULL, 0u);
        } else {
            (void)queue_ok(device, procedure, NULL, 0u);
        }
    } else if (procedure_is_multi(procedure)) {
        (void)queue_multi_error(device, procedure, antenna_number, result);
    } else {
        (void)queue_error(device, procedure, result);
    }
}

static void send_configuration(ret_device_t *device,
                               const uint8_t procedure,
                               const uint8_t antenna_number,
                               const uint8_t *data,
                               const uint16_t length) {
    ret_antenna_config_t *antenna;
    const uint8_t antenna_index = (uint8_t)(antenna_number - 1u);
    const uint16_t overhead = procedure_is_multi(procedure) ? 4u : 3u;
    const uint16_t maximum_segment =
        (device->maximum_rx_information > overhead)
            ? (uint16_t)(device->maximum_rx_information - overhead)
            : 0u;
    if (!valid_antenna(device, antenna_number)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        }
        return;
    }
    antenna = &device->config.antennas[antenna_index];
    if ((maximum_segment == 0u) || (length > maximum_segment)) {
        device->configuration_transfer_active[antenna_index] = false;
        antenna->vendor_configuration_length = 0u;
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        }
        return;
    }
    if (!device->configuration_transfer_active[antenna_index]) {
        antenna->vendor_configuration_length = 0u;
    }
    if ((uint32_t)antenna->vendor_configuration_length + length >
        RET_VENDOR_CONFIG_SIZE) {
        device->configuration_transfer_active[antenna_index] = false;
        antenna->vendor_configuration_length = 0u;
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_OUT_OF_RANGE);
        } else {
            (void)queue_error(device, procedure, RET_RC_OUT_OF_RANGE);
        }
        return;
    }
    if (length != 0u) {
        memcpy(&antenna->vendor_configuration[antenna->vendor_configuration_length],
               data, length);
        antenna->vendor_configuration_length =
            (uint16_t)(antenna->vendor_configuration_length + length);
    }
    device->configuration_transfer_active[antenna_index] =
        length == maximum_segment;
    antenna->configured = !device->configuration_transfer_active[antenna_index];
    if (!persist(device)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_HARDWARE_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_HARDWARE_ERROR);
        }
        return;
    }
    if (procedure_is_multi(procedure)) {
        (void)queue_multi_ok(device, procedure, antenna_number, NULL, 0u);
    } else {
        (void)queue_ok(device, procedure, NULL, 0u);
    }
}

static void start_tcc(ret_device_t *device,
                      const ret_tcc_type_t type,
                      const uint8_t procedure,
                      const uint8_t antenna_number,
                      const int16_t target,
                      const uint32_t timeout_ms) {
    ret_motor_operation_t operation = RET_MOTOR_SET_TILT;
    if (type == RET_TCC_CALIBRATE) {
        operation = RET_MOTOR_CALIBRATE;
    } else if (type == RET_TCC_SELF_TEST) {
        operation = RET_MOTOR_SELF_TEST;
    }
    if ((device->platform.motor_start == NULL) ||
        !device->platform.motor_start(device->platform.context, operation,
                                      antenna_number, target)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_HARDWARE_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_HARDWARE_ERROR);
        }
        return;
    }
    device->tcc.type = type;
    device->tcc.procedure = procedure;
    device->tcc.antenna_number = antenna_number;
    device->tcc.target_tilt_tenths = target;
    device->tcc.deadline_ms = now_ms(device) + timeout_ms;
}

static void request_set_tilt(ret_device_t *device,
                             const uint8_t procedure,
                             const uint8_t antenna_number,
                             const int16_t target) {
    const ret_antenna_config_t *antenna;
    if (!valid_antenna(device, antenna_number)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        }
        return;
    }
    antenna = &device->config.antennas[antenna_number - 1u];
    if (!antenna->configured) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_NOT_CONFIGURED);
        } else {
            (void)queue_error(device, procedure, RET_RC_NOT_CONFIGURED);
        }
    } else if (!antenna->calibrated) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_NOT_CALIBRATED);
        } else {
            (void)queue_error(device, procedure, RET_RC_NOT_CALIBRATED);
        }
    } else if ((target < antenna->minimum_tilt_tenths) ||
               (target > antenna->maximum_tilt_tenths)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_OUT_OF_RANGE);
        } else {
            (void)queue_error(device, procedure, RET_RC_OUT_OF_RANGE);
        }
    } else {
        start_tcc(device, RET_TCC_SET_TILT, procedure, antenna_number,
                  target, 120000u);
    }
}

static void request_calibrate(ret_device_t *device,
                              const uint8_t procedure,
                              const uint8_t antenna_number) {
    if (!valid_antenna(device, antenna_number)) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        }
        return;
    }
    if (!device->config.antennas[antenna_number - 1u].configured) {
        if (procedure_is_multi(procedure)) {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_NOT_CONFIGURED);
        } else {
            (void)queue_error(device, procedure, RET_RC_NOT_CONFIGURED);
        }
        return;
    }
    start_tcc(device, RET_TCC_CALIBRATE, procedure, antenna_number, 0, 240000u);
}

static void handle_l7(ret_device_t *device,
                      const uint8_t *message,
                      const uint16_t message_length) {
    uint8_t procedure;
    uint16_t declared_length;
    const uint8_t *data;
    uint8_t antenna_number = 0u;

    if (message_length < 3u) {
        return;
    }
    procedure = message[0];
    declared_length = get_u16_le(&message[1]);
    data = &message[3];
    if (declared_length != (uint16_t)(message_length - 3u)) {
        (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        return;
    }

    if ((device->tcc.type != RET_TCC_NONE) &&
        !procedure_allowed_during_tcc(procedure)) {
        if (procedure_is_multi(procedure) && (declared_length != 0u)) {
            (void)queue_multi_error(device, procedure, data[0], RET_RC_BUSY);
        } else {
            (void)queue_error(device, procedure, RET_RC_BUSY);
        }
        return;
    }

    if ((device->config.device_type == RET_DEVICE_TYPE_SINGLE) &&
        procedure_is_multi(procedure)) {
        antenna_number = (declared_length != 0u) ? data[0] : 0u;
        if (procedure == RET_PROC_ANTENNA_COUNT) {
            (void)queue_error(device, procedure, RET_RC_UNSUPPORTED_PROCEDURE);
        } else {
            (void)queue_multi_error(device, procedure, antenna_number,
                                    RET_RC_UNSUPPORTED_PROCEDURE);
        }
        return;
    }
    if ((device->config.device_type == RET_DEVICE_TYPE_MULTI) &&
        procedure_is_single_specific(procedure)) {
        (void)queue_error(device, procedure, RET_RC_UNSUPPORTED_PROCEDURE);
        return;
    }

    switch (procedure) {
    case RET_PROC_RESET_SOFTWARE:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            if (device->platform.motor_stop != NULL) {
                device->platform.motor_stop(device->platform.context);
            }
            memset(device->common_alarms, 0, sizeof(device->common_alarms));
            memset(device->antenna_alarms, 0, sizeof(device->antenna_alarms));
            memset(&device->tcc, 0, sizeof(device->tcc));
            memset(device->configuration_transfer_active, 0,
                   sizeof(device->configuration_transfer_active));
            device->alarm_subscribed = false;
            device->l7_queue_head = 0u;
            device->l7_queue_count = 0u;
            if (queue_ok(device, procedure, NULL, 0u)) {
                device->reset_after_i_ack = true;
            }
        }
        break;
    case RET_PROC_GET_ALARM_STATUS:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            respond_alarm_status(device, procedure, 0u);
        }
        break;
    case RET_PROC_GET_INFORMATION:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            respond_information(device);
        }
        break;
    case RET_PROC_CLEAR_ACTIVE_ALARMS:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            clear_alarm_set(device->common_alarms);
            (void)queue_ok(device, procedure, NULL, 0u);
        }
        break;
    case RET_PROC_ALARM_SUBSCRIBE:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            device->alarm_subscribed = true;
            mark_active_alarms_changed(device->common_alarms);
            for (antenna_number = 0u;
                 antenna_number < device->config.antenna_count;
                 ++antenna_number) {
                mark_active_alarms_changed(device->antenna_alarms[antenna_number]);
            }
            (void)queue_ok(device, procedure, NULL, 0u);
        }
        break;
    case RET_PROC_SELF_TEST:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else if (!device->config.antennas[0].calibrated) {
            (void)queue_error(device, procedure, RET_RC_NOT_CALIBRATED);
        } else {
            start_tcc(device, RET_TCC_SELF_TEST, procedure, 1u,
                      device->config.antennas[0].current_tilt_tenths, 120000u);
        }
        break;
    case RET_PROC_READ_USER_DATA:
        if (declared_length != 3u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            const uint16_t offset = get_u16_le(data);
            const uint8_t count = data[2];
            if (((uint32_t)offset + count > RET_USER_DATA_SIZE) ||
                ((uint16_t)count + 4u > device->maximum_tx_information)) {
                (void)queue_error(device, procedure, RET_RC_OUT_OF_RANGE);
            } else {
                (void)queue_ok(device, procedure,
                               &device->config.user_data[offset], count);
            }
        }
        break;
    case RET_PROC_WRITE_USER_DATA:
        if (declared_length < 3u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            const uint16_t offset = get_u16_le(data);
            const uint8_t count = data[2];
            if ((uint16_t)(count + 3u) != declared_length) {
                (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
            } else if ((uint32_t)offset + count > RET_USER_DATA_SIZE) {
                (void)queue_error(device, procedure, RET_RC_OUT_OF_RANGE);
            } else {
                memcpy(&device->config.user_data[offset], &data[3], count);
                if (persist(device)) {
                    (void)queue_ok(device, procedure, NULL, 0u);
                } else {
                    (void)queue_error(device, procedure, RET_RC_HARDWARE_ERROR);
                }
            }
        }
        break;
    case RET_PROC_SET_DEVICE_DATA:
        if (declared_length < 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            set_device_data(device, procedure, 1u, data[0], &data[1],
                            (uint16_t)(declared_length - 1u));
        }
        break;
    case RET_PROC_GET_DEVICE_DATA:
        if (declared_length != 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            get_device_data(device, procedure, 1u, data[0]);
        }
        break;
    case RET_PROC_CALIBRATE:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            request_calibrate(device, procedure, 1u);
        }
        break;
    case RET_PROC_SEND_CONFIGURATION:
        send_configuration(device, procedure, 1u, data, declared_length);
        break;
    case RET_PROC_SET_TILT:
        if (declared_length != 2u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            request_set_tilt(device, procedure, 1u, get_i16_le(data));
        }
        break;
    case RET_PROC_GET_TILT:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else if (!device->config.antennas[0].configured) {
            (void)queue_error(device, procedure, RET_RC_NOT_CONFIGURED);
        } else if (!device->config.antennas[0].calibrated) {
            (void)queue_error(device, procedure, RET_RC_NOT_CALIBRATED);
        } else {
            uint8_t tilt[2];
            put_u16_le(tilt, (uint16_t)device->config.antennas[0].current_tilt_tenths);
            (void)queue_ok(device, procedure, tilt, sizeof(tilt));
        }
        break;
    case RET_PROC_ANTENNA_CALIBRATE:
        if (declared_length != 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            request_calibrate(device, procedure, data[0]);
        }
        break;
    case RET_PROC_ANTENNA_SET_TILT:
        if (declared_length != 3u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            request_set_tilt(device, procedure, data[0], get_i16_le(&data[1]));
        }
        break;
    case RET_PROC_ANTENNA_GET_TILT:
        if (declared_length != 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else if (!valid_antenna(device, data[0])) {
            (void)queue_multi_error(device, procedure, data[0], RET_RC_FORMAT_ERROR);
        } else {
            const ret_antenna_config_t *antenna = &device->config.antennas[data[0] - 1u];
            uint8_t tilt[2];
            if (!antenna->configured) {
                (void)queue_multi_error(device, procedure, data[0],
                                        RET_RC_NOT_CONFIGURED);
            } else if (!antenna->calibrated) {
                (void)queue_multi_error(device, procedure, data[0],
                                        RET_RC_NOT_CALIBRATED);
            } else {
                put_u16_le(tilt, (uint16_t)antenna->current_tilt_tenths);
                (void)queue_multi_ok(device, procedure, data[0], tilt, sizeof(tilt));
            }
        }
        break;
    case RET_PROC_ANTENNA_SET_DEVICE_DATA:
        if (declared_length < 2u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            set_device_data(device, procedure, data[0], data[1], &data[2],
                            (uint16_t)(declared_length - 2u));
        }
        break;
    case RET_PROC_ANTENNA_GET_DEVICE_DATA:
        if (declared_length != 2u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            get_device_data(device, procedure, data[0], data[1]);
        }
        break;
    case RET_PROC_ANTENNA_CLEAR_ALARMS:
        if (declared_length != 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else if (!valid_antenna(device, data[0])) {
            (void)queue_multi_error(device, procedure, data[0], RET_RC_FORMAT_ERROR);
        } else {
            clear_alarm_set(device->antenna_alarms[data[0] - 1u]);
            (void)queue_multi_ok(device, procedure, data[0], NULL, 0u);
        }
        break;
    case RET_PROC_ANTENNA_GET_ALARMS:
        if (declared_length != 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            respond_alarm_status(device, procedure, data[0]);
        }
        break;
    case RET_PROC_ANTENNA_COUNT:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            const uint8_t count = device->config.antenna_count;
            (void)queue_ok(device, procedure, &count, 1u);
        }
        break;
    case RET_PROC_ANTENNA_SEND_CONFIGURATION:
        if (declared_length < 1u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            send_configuration(device, procedure, data[0], &data[1],
                               (uint16_t)(declared_length - 1u));
        }
        break;
    case RET_PROC_DOWNLOAD_START:
    case RET_PROC_DOWNLOAD_END:
        if (declared_length != 0u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_UNSUPPORTED_PROCEDURE);
        }
        break;
    case RET_PROC_DOWNLOAD_APPLICATION:
        (void)queue_error(device, procedure, RET_RC_UNSUPPORTED_PROCEDURE);
        break;
    case RET_PROC_VENDOR_SPECIFIC:
        if (declared_length < 2u) {
            (void)queue_error(device, procedure, RET_RC_FORMAT_ERROR);
        } else {
            (void)queue_error(device, procedure, RET_RC_UNSUPPORTED_PROCEDURE);
        }
        break;
    case RET_PROC_ALARM_INDICATION:
        (void)queue_error(device, procedure, RET_RC_UNSUPPORTED_PROCEDURE);
        break;
    case RET_PROC_ANTENNA_ALARM_INDICATION:
        antenna_number = (declared_length != 0u) ? data[0] : 0u;
        (void)queue_multi_error(device, procedure, antenna_number,
                                RET_RC_UNSUPPORTED_PROCEDURE);
        break;
    default:
        (void)queue_error(device, procedure, RET_RC_UNKNOWN_PROCEDURE);
        break;
    }
}

static void process_i_frame(ret_device_t *device, const ret_hdlc_frame_t *frame) {
    const uint8_t ns = (uint8_t)((frame->control >> 1u) & 7u);
    const uint8_t nr = (uint8_t)((frame->control >> 5u) & 7u);
    const bool poll = (frame->control & RET_CONTROL_PF) != 0u;
    if (!acknowledgement_is_valid(device, nr)) {
        schedule_frame_reject(device, frame->control, 0x08u);
        return;
    }
    acknowledge_transmit(device, nr);

    if (ns == device->receive_sequence) {
        if (frame->information_length <= device->maximum_rx_information) {
            device->receive_sequence = (uint8_t)((device->receive_sequence + 1u) & 7u);
            handle_l7(device, frame->information, frame->information_length);
        } else {
            schedule_frame_reject(device, frame->control, 0x04u);
            return;
        }
    } else if (ns != ((device->receive_sequence + 7u) & 7u)) {
        schedule_frame_reject(device, frame->control, 0x08u);
        return;
    }

    if (poll) {
        respond_to_poll(device);
    }
}

static void process_connected_frame(ret_device_t *device,
                                    const ret_hdlc_frame_t *frame) {
    const uint8_t control_without_pf = (uint8_t)(frame->control &
                                                  (uint8_t)~RET_CONTROL_PF);
    if ((frame->control & 1u) == 0u) {
        process_i_frame(device, frame);
        return;
    }
    if ((frame->control & 3u) == 1u) {
        const uint8_t nr = (uint8_t)((frame->control >> 5u) & 7u);
        const uint8_t type = (uint8_t)(frame->control & 0x0Fu);
        if ((type != 0x01u) && (type != 0x05u)) {
            schedule_frame_reject(device, frame->control, 0x01u);
            return;
        }
        if (!acknowledgement_is_valid(device, nr)) {
            schedule_frame_reject(device, frame->control, 0x08u);
            return;
        }
        acknowledge_transmit(device, nr);
        device->remote_ready = (type == 0x01u);
        if ((frame->control & RET_CONTROL_PF) != 0u) {
            if ((type == 0x01u) && device->outstanding_i_frame_valid &&
                (nr != device->send_sequence)) {
                schedule_link_response(device, &device->outstanding_i_frame,
                                       RET_MINIMUM_FRAME_DELAY_MS);
            } else {
                respond_to_poll(device);
            }
        }
        return;
    }

    if (control_without_pf == RET_CONTROL_DISC) {
        ret_hdlc_frame_t response;
        memset(&response, 0, sizeof(response));
        response.address = device->address;
        response.control = (uint8_t)(RET_CONTROL_UA | RET_CONTROL_PF);
        device->link_state = RET_LINK_ADDRESS_ASSIGNED;
        clear_runtime_state(device, false, false);
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
    } else if (control_without_pf == RET_CONTROL_SNRM) {
        ret_hdlc_frame_t response;
        memset(&response, 0, sizeof(response));
        response.address = device->address;
        response.control = (uint8_t)(RET_CONTROL_UA | RET_CONTROL_PF);
        clear_runtime_state(device, false, false);
        device->link_state = RET_LINK_CONNECTED;
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
    } else {
        schedule_frame_reject(device, frame->control, 0x01u);
    }
}

static void process_frame(ret_device_t *device, const ret_hdlc_frame_t *frame) {
    const uint8_t control_without_pf = (uint8_t)(frame->control &
                                                  (uint8_t)~RET_CONTROL_PF);
    const bool broadcast = frame->address == RET_ALL_STATIONS;
    const bool addressed = (device->link_state != RET_LINK_NO_ADDRESS) &&
                           (frame->address == device->address);

    if (control_without_pf == RET_CONTROL_XID) {
        if (broadcast || addressed ||
            ((device->link_state == RET_LINK_NO_ADDRESS) &&
             (frame->address == RET_NO_STATION))) {
            if (addressed) {
                device->last_addressed_frame_ms = now_ms(device);
            }
            process_xid(device, frame);
        }
        return;
    }
    if (!addressed) {
        return;
    }
    device->last_addressed_frame_ms = now_ms(device);

    if ((control_without_pf == RET_CONTROL_SNRM) &&
        (device->link_state == RET_LINK_ADDRESS_ASSIGNED)) {
        ret_hdlc_frame_t response;
        clear_runtime_state(device, false, false);
        device->link_state = RET_LINK_CONNECTED;
        memset(&response, 0, sizeof(response));
        response.address = device->address;
        response.control = (uint8_t)(RET_CONTROL_UA | RET_CONTROL_PF);
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
        return;
    }
    if (device->link_state != RET_LINK_CONNECTED) {
        ret_hdlc_frame_t response;
        memset(&response, 0, sizeof(response));
        response.address = device->address;
        response.control = (uint8_t)(RET_CONTROL_DM | RET_CONTROL_PF);
        schedule_link_response(device, &response, RET_MINIMUM_FRAME_DELAY_MS);
        return;
    }
    process_connected_frame(device, frame);
}

void ret_device_receive(ret_device_t *device,
                        const uint8_t *data,
                        const size_t length) {
    size_t index;
    ret_hdlc_frame_t frame;
    if ((device == NULL) || ((data == NULL) && (length != 0u))) {
        return;
    }
    for (index = 0u; index < length; ++index) {
        const ret_hdlc_decoder_result_t result =
            ret_hdlc_decoder_push(&device->decoder, data[index], &frame);
        if (result == RET_HDLC_DECODER_FRAME) {
            process_frame(device, &frame);
        }
    }
}

static uint8_t motor_result_code(const ret_motor_result_t result) {
    switch (result) {
    case RET_MOTOR_JAM:
        return RET_RC_MOTOR_JAM;
    case RET_ACTUATOR_JAM:
        return RET_RC_ACTUATOR_JAM;
    case RET_MOTOR_HARDWARE_ERROR:
    default:
        return RET_RC_HARDWARE_ERROR;
    }
}

static void complete_tcc(ret_device_t *device, const ret_motor_result_t result) {
    const ret_tcc_t completed = device->tcc;
    ret_antenna_config_t *antenna =
        &device->config.antennas[completed.antenna_number - 1u];
    memset(&device->tcc, 0, sizeof(device->tcc));

    if (result == RET_MOTOR_OK) {
        if (completed.type == RET_TCC_SET_TILT) {
            antenna->current_tilt_tenths = completed.target_tilt_tenths;
        } else if (completed.type == RET_TCC_CALIBRATE) {
            antenna->calibrated = true;
        }
        if ((completed.type != RET_TCC_SELF_TEST) && !persist(device)) {
            if (procedure_is_multi(completed.procedure)) {
                (void)queue_multi_error(device, completed.procedure,
                                        completed.antenna_number,
                                        RET_RC_HARDWARE_ERROR);
            } else {
                (void)queue_error(device, completed.procedure,
                                  RET_RC_HARDWARE_ERROR);
            }
            return;
        }
        if (completed.type == RET_TCC_SELF_TEST) {
            uint8_t alarms[RET_MAX_ALARMS];
            const uint16_t count = active_alarm_codes(device->common_alarms,
                                                       alarms, sizeof(alarms));
            (void)queue_ok(device, completed.procedure, alarms, count);
        } else if (procedure_is_multi(completed.procedure)) {
            (void)queue_multi_ok(device, completed.procedure,
                                 completed.antenna_number, NULL, 0u);
        } else {
            (void)queue_ok(device, completed.procedure, NULL, 0u);
        }
    } else if (procedure_is_multi(completed.procedure)) {
        (void)queue_multi_error(device, completed.procedure,
                                completed.antenna_number,
                                motor_result_code(result));
    } else {
        (void)queue_error(device, completed.procedure,
                          motor_result_code(result));
    }
}

static void poll_tcc(ret_device_t *device, const uint32_t now) {
    ret_motor_result_t result;
    int16_t position;
    if (device->tcc.type == RET_TCC_NONE) {
        return;
    }
    if (time_reached(now, device->tcc.deadline_ms)) {
        if (device->platform.motor_stop != NULL) {
            device->platform.motor_stop(device->platform.context);
        }
        complete_tcc(device, RET_MOTOR_JAM);
        return;
    }
    if (device->platform.motor_poll == NULL) {
        complete_tcc(device, RET_MOTOR_HARDWARE_ERROR);
        return;
    }
    position = device->config.antennas[device->tcc.antenna_number - 1u]
                   .current_tilt_tenths;
    result = device->platform.motor_poll(device->platform.context,
                                         device->tcc.antenna_number,
                                         &position);
    device->config.antennas[device->tcc.antenna_number - 1u]
        .current_tilt_tenths = position;
    if ((result != RET_MOTOR_RUNNING) && (result != RET_MOTOR_IDLE)) {
        complete_tcc(device, result);
    }
}

void ret_device_poll(ret_device_t *device) {
    uint32_t now;
    if (device == NULL) {
        return;
    }
    now = now_ms(device);
    poll_tcc(device, now);

    if ((device->link_state != RET_LINK_NO_ADDRESS) &&
        time_reached(now, device->last_addressed_frame_ms + RET_LINK_TIMEOUT_MS)) {
        if (device->platform.motor_stop != NULL) {
            device->platform.motor_stop(device->platform.context);
        }
        if (device->platform.system_reset != NULL) {
            device->platform.system_reset(device->platform.context);
        } else {
            clear_runtime_state(device, true, true);
        }
        return;
    }

    if (device->pending_link_response_valid &&
        time_reached(now, device->pending_link_response_at_ms)) {
        uint8_t wire[RET_HDLC_MAX_WIRE];
        size_t wire_length = 0u;
        if ((device->platform.transmit != NULL) &&
            ret_hdlc_encode(&device->pending_link_response, wire, sizeof(wire),
                            &wire_length)) {
            device->platform.transmit(device->platform.context, wire, wire_length);
        }
        device->pending_link_response_valid = false;
        if (device->reset_after_transmit) {
            device->reset_after_transmit = false;
            if (device->platform.system_reset != NULL) {
                device->platform.system_reset(device->platform.context);
            }
        }
    }
}

ret_link_state_t ret_device_link_state(const ret_device_t *device) {
    return (device != NULL) ? device->link_state : RET_LINK_NO_ADDRESS;
}

uint8_t ret_device_address(const ret_device_t *device) {
    return (device != NULL) ? device->address : RET_NO_STATION;
}
