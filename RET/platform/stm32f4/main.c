#include "ret_stm32_platform.h"

#include "ret_board_config.h"

#include <string.h>

#define RET_BOOT_TIME_MS 3000u
#define RET_ALARM_HARDWARE_ERROR 0x11u

static ret_device_t device;

static char hex_digit(const uint8_t value) {
    if (value < 10u) {
        return (char)('0' + (int)value);
    }
    return (char)('A' + (int)value - 10);
}

static void append_hex16(char *destination, const uint16_t value) {
    destination[0] = hex_digit((uint8_t)((value >> 12u) & 0x0Fu));
    destination[1] = hex_digit((uint8_t)((value >> 8u) & 0x0Fu));
    destination[2] = hex_digit((uint8_t)((value >> 4u) & 0x0Fu));
    destination[3] = hex_digit((uint8_t)(value & 0x0Fu));
}

static void set_reference_defaults(ret_config_t *config) {
    uint32_t uid0;
    uint32_t uid1;
    char identity[18];
    ret_antenna_config_t *antenna;

    ret_config_set_defaults(config);
    config->vendor_code[0] = 'T';
    config->vendor_code[1] = 'Y';

    uid0 = HAL_GetUIDw0();
    uid1 = HAL_GetUIDw1();
    memset(identity, 0, sizeof(identity));
    memcpy(identity, "0912R2-", 7u);
    append_hex16(&identity[7], (uint16_t)uid0);
    append_hex16(&identity[11], (uint16_t)uid1);
    config->unit_id_length = 15u;
    memcpy(config->unit_id, identity, config->unit_id_length);
    memset(config->serial_number, 0, sizeof(config->serial_number));
    memcpy(config->serial_number, identity, config->unit_id_length);
    memset(config->antennas[0].serial, 0,
           sizeof(config->antennas[0].serial));
    memcpy(config->antennas[0].serial, identity, config->unit_id_length);
    memset(config->product_number, 0, sizeof(config->product_number));
    memcpy(config->product_number, "R09120", 6u);
    memset(config->hardware_version, 0, sizeof(config->hardware_version));
    memcpy(config->hardware_version, "RET_v2-STM32F4", 14u);
    memset(config->software_version, 0, sizeof(config->software_version));
    memcpy(config->software_version, "2.0.0", 5u);

    antenna = &config->antennas[0];
    memcpy(antenna->model, "PAINEL_OMNI_B28", sizeof(antenna->model));
    antenna->operating_bands = 0x0010u;
    memset(antenna->beamwidth, 0, sizeof(antenna->beamwidth));
    antenna->beamwidth[0] = 70u;
    memset(antenna->gain_tenths_db, 0, sizeof(antenna->gain_tenths_db));
    antenna->gain_tenths_db[0] = 150u;
    antenna->gain_tenths_db[1] = 130u;
    antenna->minimum_tilt_tenths = 20;
    antenna->maximum_tilt_tenths = 100;
    antenna->current_tilt_tenths = 20;
    antenna->bearing = 145u;
    antenna->mechanical_tilt_tenths = 110;
    antenna->configured = true;
    antenna->calibrated = false;
}

int main(void) {
    ret_platform_t platform;
    ret_config_t fallback;
    uint8_t received[64];
    size_t received_length;

    HAL_Init();
    ret_stm32_system_clock_config();
    ret_stm32_hardware_init(&g_ret_stm32);
    set_reference_defaults(&fallback);
    platform = ret_stm32_make_platform(&g_ret_stm32);
    ret_device_init(&device, &platform, &fallback);
    ret_stm32_set_tilt_limits(&g_ret_stm32,
                              device.config.antennas[0].minimum_tilt_tenths,
                              device.config.antennas[0].maximum_tilt_tenths);
    if (!g_ret_stm32.adc_calibrated) {
        device.config.antennas[0].calibrated = false;
    }

    /* TS 25.461 defines the first three seconds after power-up as start-up. */
    HAL_Delay(RET_BOOT_TIME_MS);
    __disable_irq();
    g_ret_stm32.uart_rx_head = 0u;
    g_ret_stm32.uart_rx_tail = 0u;
    __enable_irq();
    HAL_GPIO_WritePin(RET_MOTOR_SLEEP_PORT, RET_MOTOR_SLEEP_PIN, GPIO_PIN_SET);

    for (;;) {
        received_length = 0u;
        while ((received_length < sizeof(received)) &&
               ret_stm32_uart_read(&g_ret_stm32,
                                   &received[received_length])) {
            ++received_length;
        }
        if (received_length != 0u) {
            ret_device_receive(&device, received, received_length);
        }

        ret_device_set_common_alarm(
            &device, RET_ALARM_HARDWARE_ERROR,
            ret_stm32_driver_fault(&g_ret_stm32));
        ret_device_poll(&device);

        HAL_GPIO_WritePin(
            RET_STATUS_LED_PORT, RET_STATUS_LED_PIN,
            (ret_device_link_state(&device) == RET_LINK_CONNECTED)
                ? GPIO_PIN_SET
                : GPIO_PIN_RESET);
    }
}
