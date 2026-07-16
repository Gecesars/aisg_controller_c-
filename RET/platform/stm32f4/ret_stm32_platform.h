#ifndef RET_STM32_PLATFORM_H
#define RET_STM32_PLATFORM_H

#include "ret/ret_protocol.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    UART_HandleTypeDef uart;
    ADC_HandleTypeDef adc;
    volatile uint16_t uart_rx_head;
    volatile uint16_t uart_rx_tail;
    uint8_t uart_rx_octet;
    uint8_t uart_rx_buffer[256];

    ret_motor_operation_t motor_operation;
    ret_motor_result_t motor_result;
    uint8_t motor_phase;
    int8_t motor_direction;
    int16_t tilt_min_tenths;
    int16_t tilt_max_tenths;
    int16_t motor_target_tenths;
    int16_t calibration_return_tenths;
    uint16_t adc_min;
    uint16_t adc_max;
    uint16_t last_adc;
    uint32_t motor_started_ms;
    uint32_t last_motion_ms;
    bool adc_calibrated;
    bool reset_requested;
} ret_stm32_context_t;

extern ret_stm32_context_t g_ret_stm32;

void ret_stm32_hardware_init(ret_stm32_context_t *context);

ret_platform_t ret_stm32_make_platform(ret_stm32_context_t *context);

bool ret_stm32_uart_read(ret_stm32_context_t *context, uint8_t *octet);

void ret_stm32_set_tilt_limits(ret_stm32_context_t *context,
                               int16_t minimum_tenths,
                               int16_t maximum_tenths);

bool ret_stm32_driver_fault(const ret_stm32_context_t *context);

void ret_stm32_system_clock_config(void);

void ret_stm32_error(void);

#endif
