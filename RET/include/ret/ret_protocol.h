#ifndef RET_PROTOCOL_H
#define RET_PROTOCOL_H

#include "ret/ret_hdlc.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RET_MAX_ANTENNAS 4u
#define RET_MAX_ALARMS 16u
#define RET_L7_QUEUE_DEPTH 6u
#define RET_USER_DATA_SIZE 256u
#define RET_VENDOR_CONFIG_SIZE 1024u

#define RET_DEVICE_TYPE_SINGLE 0x01u
#define RET_DEVICE_TYPE_MULTI 0x11u

typedef enum {
    RET_LINK_NO_ADDRESS = 0,
    RET_LINK_ADDRESS_ASSIGNED,
    RET_LINK_CONNECTED
} ret_link_state_t;

typedef enum {
    RET_MOTOR_SET_TILT = 0,
    RET_MOTOR_CALIBRATE,
    RET_MOTOR_SELF_TEST
} ret_motor_operation_t;

typedef enum {
    RET_MOTOR_IDLE = 0,
    RET_MOTOR_RUNNING,
    RET_MOTOR_OK,
    RET_MOTOR_JAM,
    RET_ACTUATOR_JAM,
    RET_MOTOR_HARDWARE_ERROR
} ret_motor_result_t;

typedef struct {
    char model[15];
    char serial[17];
    uint16_t operating_bands;
    uint16_t beamwidth[4];
    uint8_t gain_tenths_db[4];
    int16_t minimum_tilt_tenths;
    int16_t maximum_tilt_tenths;
    int16_t current_tilt_tenths;
    char installation_date[6];
    char installer_id[5];
    char base_station_id[32];
    char sector_id[32];
    uint16_t bearing;
    int16_t mechanical_tilt_tenths;
    uint16_t vendor_configuration_length;
    uint8_t vendor_configuration[RET_VENDOR_CONFIG_SIZE];
    bool configured;
    bool calibrated;
} ret_antenna_config_t;

typedef struct {
    uint32_t schema_version;
    uint8_t device_type;
    char vendor_code[2];
    uint8_t unit_id_length;
    uint8_t unit_id[17];
    char product_number[32];
    char serial_number[17];
    char hardware_version[16];
    char software_version[16];
    uint8_t antenna_count;
    ret_antenna_config_t antennas[RET_MAX_ANTENNAS];
    uint8_t user_data[RET_USER_DATA_SIZE];
} ret_config_t;

typedef struct {
    void *context;
    uint32_t (*millis)(void *context);
    void (*transmit)(void *context, const uint8_t *data, size_t length);
    bool (*storage_load)(void *context, ret_config_t *config);
    bool (*storage_save)(void *context, const ret_config_t *config);
    bool (*motor_start)(void *context,
                        ret_motor_operation_t operation,
                        uint8_t antenna_number,
                        int16_t target_tilt_tenths);
    ret_motor_result_t (*motor_poll)(void *context,
                                    uint8_t antenna_number,
                                    int16_t *position_tenths);
    void (*motor_stop)(void *context);
    void (*application_reset)(void *context);
    void (*system_reset)(void *context);
} ret_platform_t;

typedef struct {
    uint8_t code;
    bool active;
    bool changed;
} ret_alarm_t;

typedef struct {
    uint16_t length;
    uint8_t data[RET_HDLC_MAX_INFO];
} ret_l7_message_t;

typedef enum {
    RET_TCC_NONE = 0,
    RET_TCC_SELF_TEST,
    RET_TCC_CALIBRATE,
    RET_TCC_SET_TILT
} ret_tcc_type_t;

typedef struct {
    ret_tcc_type_t type;
    uint8_t procedure;
    uint8_t antenna_number;
    int16_t target_tilt_tenths;
    uint32_t deadline_ms;
} ret_tcc_t;

typedef struct {
    ret_platform_t platform;
    ret_config_t config;
    ret_hdlc_decoder_t decoder;

    ret_link_state_t link_state;
    uint8_t address;
    uint8_t receive_sequence;
    uint8_t send_sequence;
    bool remote_ready;
    uint16_t maximum_tx_information;
    uint16_t maximum_rx_information;
    uint32_t last_addressed_frame_ms;

    ret_hdlc_frame_t pending_link_response;
    bool pending_link_response_valid;
    uint32_t pending_link_response_at_ms;
    bool reset_after_transmit;

    ret_hdlc_frame_t outstanding_i_frame;
    bool outstanding_i_frame_valid;
    bool reset_after_i_ack;

    ret_l7_message_t l7_queue[RET_L7_QUEUE_DEPTH];
    uint8_t l7_queue_head;
    uint8_t l7_queue_count;

    ret_tcc_t tcc;
    bool configuration_transfer_active[RET_MAX_ANTENNAS];
    bool alarm_subscribed;
    ret_alarm_t common_alarms[RET_MAX_ALARMS];
    ret_alarm_t antenna_alarms[RET_MAX_ANTENNAS][RET_MAX_ALARMS];
} ret_device_t;

void ret_config_set_defaults(ret_config_t *config);

bool ret_config_is_valid(const ret_config_t *config);

void ret_device_init(ret_device_t *device,
                     const ret_platform_t *platform,
                     const ret_config_t *fallback_config);

void ret_device_receive(ret_device_t *device,
                        const uint8_t *data,
                        size_t length);

void ret_device_poll(ret_device_t *device);

void ret_device_set_common_alarm(ret_device_t *device,
                                 uint8_t alarm_code,
                                 bool active);

void ret_device_set_antenna_alarm(ret_device_t *device,
                                  uint8_t antenna_number,
                                  uint8_t alarm_code,
                                  bool active);

ret_link_state_t ret_device_link_state(const ret_device_t *device);

uint8_t ret_device_address(const ret_device_t *device);

#ifdef __cplusplus
}
#endif

#endif
