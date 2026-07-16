#include "ret_stm32_platform.h"

#include "ret_board_config.h"
#include "ret_flash_storage.h"

#include <string.h>

ret_stm32_context_t g_ret_stm32;

static int absolute_int(const int value) {
    return (value < 0) ? -value : value;
}

static uint32_t platform_millis(void *opaque) {
    (void)opaque;
    return HAL_GetTick();
}

static bool read_adc(ret_stm32_context_t *context, uint16_t *value) {
    if ((HAL_ADC_Start(&context->adc) != HAL_OK) ||
        (HAL_ADC_PollForConversion(&context->adc, 5u) != HAL_OK)) {
        (void)HAL_ADC_Stop(&context->adc);
        return false;
    }
    *value = (uint16_t)HAL_ADC_GetValue(&context->adc);
    return HAL_ADC_Stop(&context->adc) == HAL_OK;
}

static bool limit_min_active(void) {
    return HAL_GPIO_ReadPin(RET_LIMIT_MIN_PORT, RET_LIMIT_MIN_PIN) ==
           GPIO_PIN_RESET;
}

static bool limit_max_active(void) {
    return HAL_GPIO_ReadPin(RET_LIMIT_MAX_PORT, RET_LIMIT_MAX_PIN) ==
           GPIO_PIN_RESET;
}

bool ret_stm32_driver_fault(const ret_stm32_context_t *context) {
    (void)context;
    return HAL_GPIO_ReadPin(RET_MOTOR_FAULT_PORT, RET_MOTOR_FAULT_PIN) ==
           GPIO_PIN_RESET;
}

static void motor_drive(ret_stm32_context_t *context, const int8_t direction) {
    context->motor_direction = direction;
    HAL_GPIO_WritePin(RET_MOTOR_SLEEP_PORT, RET_MOTOR_SLEEP_PIN, GPIO_PIN_SET);
    if (direction > 0) {
        HAL_GPIO_WritePin(RET_MOTOR_IN1_PORT, RET_MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RET_MOTOR_IN2_PORT, RET_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    } else if (direction < 0) {
        HAL_GPIO_WritePin(RET_MOTOR_IN1_PORT, RET_MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RET_MOTOR_IN2_PORT, RET_MOTOR_IN2_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RET_MOTOR_IN1_PORT, RET_MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RET_MOTOR_IN2_PORT, RET_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    }
}

static void motor_stop_callback(void *opaque) {
    ret_stm32_context_t *context = opaque;
    if (context == NULL) {
        return;
    }
    motor_drive(context, 0);
    context->motor_result = RET_MOTOR_IDLE;
    context->motor_phase = 0u;
}

static int16_t adc_to_tilt(const ret_stm32_context_t *context,
                           const uint16_t raw) {
    const int32_t denominator =
        (int32_t)context->adc_max - (int32_t)context->adc_min;
    int32_t tilt;
    if (denominator == 0) {
        return context->tilt_min_tenths;
    }
    tilt = context->tilt_min_tenths +
           (((int32_t)raw - context->adc_min) *
            ((int32_t)context->tilt_max_tenths -
             context->tilt_min_tenths)) /
               denominator;
    if (tilt < context->tilt_min_tenths) {
        tilt = context->tilt_min_tenths;
    } else if (tilt > context->tilt_max_tenths) {
        tilt = context->tilt_max_tenths;
    }
    return (int16_t)tilt;
}

static int8_t direction_to_target(const ret_stm32_context_t *context,
                                  const int16_t current,
                                  const int16_t target) {
    (void)context;
    return (target > current) ? 1 : -1;
}

static bool motor_start_callback(void *opaque,
                                 const ret_motor_operation_t operation,
                                 const uint8_t antenna_number,
                                 const int16_t target_tilt_tenths) {
    ret_stm32_context_t *context = opaque;
    uint16_t raw;
    int16_t current;
    if ((context == NULL) || (antenna_number != 1u) ||
        (context->motor_result == RET_MOTOR_RUNNING) ||
        ret_stm32_driver_fault(context) || !read_adc(context, &raw)) {
        return false;
    }

    context->motor_operation = operation;
    context->motor_result = RET_MOTOR_RUNNING;
    context->motor_target_tenths = target_tilt_tenths;
    context->motor_started_ms = HAL_GetTick();
    context->last_motion_ms = context->motor_started_ms;
    context->last_adc = raw;
    context->motor_phase = 1u;
    current = adc_to_tilt(context, raw);

    if (operation == RET_MOTOR_CALIBRATE) {
        context->calibration_return_tenths = current;
        motor_drive(context, -1);
    } else if (operation == RET_MOTOR_SELF_TEST) {
        motor_drive(context, 0);
    } else if (absolute_int((int)current - target_tilt_tenths) <=
               RET_POSITION_TOLERANCE_TENTHS) {
        context->motor_result = RET_MOTOR_OK;
        motor_drive(context, 0);
    } else {
        motor_drive(context,
                    direction_to_target(context, current,
                                        target_tilt_tenths));
    }
    return true;
}

static ret_motor_result_t motor_fail(ret_stm32_context_t *context,
                                     const ret_motor_result_t result) {
    motor_drive(context, 0);
    context->motor_result = result;
    return result;
}

static ret_motor_result_t motor_poll_callback(void *opaque,
                                              const uint8_t antenna_number,
                                              int16_t *position_tenths) {
    ret_stm32_context_t *context = opaque;
    uint16_t raw;
    uint32_t now;
    int16_t current;
    if ((context == NULL) || (position_tenths == NULL) ||
        (antenna_number != 1u)) {
        return RET_MOTOR_HARDWARE_ERROR;
    }
    if (context->motor_result != RET_MOTOR_RUNNING) {
        return context->motor_result;
    }
    if (ret_stm32_driver_fault(context) || !read_adc(context, &raw)) {
        return motor_fail(context, RET_MOTOR_HARDWARE_ERROR);
    }

    now = HAL_GetTick();
    current = adc_to_tilt(context, raw);
    *position_tenths = current;
    if (absolute_int((int)raw - context->last_adc) >= 3) {
        context->last_adc = raw;
        context->last_motion_ms = now;
    }

    if (context->motor_operation == RET_MOTOR_SELF_TEST) {
        if ((uint32_t)(now - context->motor_started_ms) >=
            RET_MOTOR_SETTLE_MS) {
            return motor_fail(context, RET_MOTOR_OK);
        }
        return RET_MOTOR_RUNNING;
    }

    if (context->motor_operation == RET_MOTOR_CALIBRATE) {
        if ((context->motor_phase == 1u) && limit_min_active()) {
            context->adc_min = raw;
            context->motor_phase = 2u;
            context->last_motion_ms = now;
            motor_drive(context, 1);
            return RET_MOTOR_RUNNING;
        }
        if ((context->motor_phase == 2u) && limit_max_active()) {
            if (absolute_int((int)raw - context->adc_min) <
                (int)RET_ADC_MINIMUM_SPAN) {
                return motor_fail(context, RET_MOTOR_HARDWARE_ERROR);
            }
            context->adc_max = raw;
            context->adc_calibrated = true;
            context->motor_phase = 3u;
            current = adc_to_tilt(context, raw);
            *position_tenths = current;
            context->last_motion_ms = now;
            if (absolute_int((int)current -
                             context->calibration_return_tenths) <=
                RET_POSITION_TOLERANCE_TENTHS) {
                return motor_fail(context, RET_MOTOR_OK);
            }
            motor_drive(context,
                        direction_to_target(context, current,
                                            context->calibration_return_tenths));
            return RET_MOTOR_RUNNING;
        }
        if (context->motor_phase == 3u) {
            if (absolute_int((int)current -
                             context->calibration_return_tenths) <=
                RET_POSITION_TOLERANCE_TENTHS) {
                return motor_fail(context, RET_MOTOR_OK);
            }
            if (((context->motor_direction < 0) && limit_min_active()) ||
                ((context->motor_direction > 0) && limit_max_active())) {
                return motor_fail(context, RET_ACTUATOR_JAM);
            }
        }
    } else {
        if (absolute_int((int)current - context->motor_target_tenths) <=
            RET_POSITION_TOLERANCE_TENTHS) {
            return motor_fail(context, RET_MOTOR_OK);
        }
        if (((context->motor_direction < 0) && limit_min_active()) ||
            ((context->motor_direction > 0) && limit_max_active())) {
            return motor_fail(context, RET_ACTUATOR_JAM);
        }
    }

    if ((uint32_t)(now - context->last_motion_ms) >=
        RET_MOTOR_STALL_TIMEOUT_MS) {
        return motor_fail(context, RET_MOTOR_JAM);
    }
    return RET_MOTOR_RUNNING;
}

static void platform_transmit(void *opaque,
                              const uint8_t *data,
                              const size_t length) {
    ret_stm32_context_t *context = opaque;
    if ((context == NULL) || (data == NULL) || (length == 0u) ||
        (length > UINT16_MAX)) {
        return;
    }
    HAL_GPIO_WritePin(RET_RS485_DE_PORT, RET_RS485_DE_PIN, GPIO_PIN_SET);
    if (HAL_UART_Transmit(&context->uart, (uint8_t *)(uintptr_t)data,
                          (uint16_t)length, 1000u) == HAL_OK) {
        while (__HAL_UART_GET_FLAG(&context->uart, UART_FLAG_TC) == RESET) {
        }
    }
    HAL_GPIO_WritePin(RET_RS485_DE_PORT, RET_RS485_DE_PIN, GPIO_PIN_RESET);
}

static void platform_reset(void *opaque) {
    ret_stm32_context_t *context = opaque;
    motor_stop_callback(context);
    context->reset_requested = true;
    __DSB();
    NVIC_SystemReset();
}

static void platform_application_reset(void *opaque) {
    ret_stm32_context_t *context = opaque;
    motor_stop_callback(context);
}

ret_platform_t ret_stm32_make_platform(ret_stm32_context_t *context) {
    ret_platform_t platform;
    memset(&platform, 0, sizeof(platform));
    platform.context = context;
    platform.millis = platform_millis;
    platform.transmit = platform_transmit;
    platform.storage_load = ret_stm32_storage_load;
    platform.storage_save = ret_stm32_storage_save;
    platform.motor_start = motor_start_callback;
    platform.motor_poll = motor_poll_callback;
    platform.motor_stop = motor_stop_callback;
    platform.application_reset = platform_application_reset;
    platform.system_reset = platform_reset;
    return platform;
}

void ret_stm32_set_tilt_limits(ret_stm32_context_t *context,
                               const int16_t minimum_tenths,
                               const int16_t maximum_tenths) {
    if ((context != NULL) && (minimum_tenths < maximum_tenths)) {
        context->tilt_min_tenths = minimum_tenths;
        context->tilt_max_tenths = maximum_tenths;
    }
}

static void gpio_init(void) {
    GPIO_InitTypeDef gpio;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(RET_RS485_DE_PORT, RET_RS485_DE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_MOTOR_IN1_PORT, RET_MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_MOTOR_IN2_PORT, RET_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_MOTOR_SLEEP_PORT, RET_MOTOR_SLEEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_STATUS_LED_PORT, RET_STATUS_LED_PIN, GPIO_PIN_RESET);

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = RET_RS485_DE_PIN | RET_MOTOR_SLEEP_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = RET_MOTOR_IN1_PIN | RET_MOTOR_IN2_PIN;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = RET_STATUS_LED_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RET_STATUS_LED_PORT, &gpio);

    gpio.Pin = RET_MOTOR_FAULT_PIN | RET_LIMIT_MIN_PIN | RET_LIMIT_MAX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void uart_init(ret_stm32_context_t *context) {
    GPIO_InitTypeDef gpio;
    __HAL_RCC_USART1_CLK_ENABLE();

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = RET_UART_TX_PIN | RET_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = RET_UART_GPIO_AF;
    HAL_GPIO_Init(GPIOB, &gpio);

    context->uart.Instance = RET_UART_INSTANCE;
    context->uart.Init.BaudRate = 9600u;
    context->uart.Init.WordLength = UART_WORDLENGTH_8B;
    context->uart.Init.StopBits = UART_STOPBITS_1;
    context->uart.Init.Parity = UART_PARITY_NONE;
    context->uart.Init.Mode = UART_MODE_TX_RX;
    context->uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    context->uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&context->uart) != HAL_OK) {
        ret_stm32_error();
    }
    HAL_NVIC_SetPriority(USART1_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(&context->uart, &context->uart_rx_octet, 1u) !=
        HAL_OK) {
        ret_stm32_error();
    }
}

static void adc_init(ret_stm32_context_t *context) {
    ADC_ChannelConfTypeDef channel;
    GPIO_InitTypeDef gpio;
    __HAL_RCC_ADC1_CLK_ENABLE();

    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = RET_POSITION_ADC_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RET_POSITION_ADC_PORT, &gpio);

    context->adc.Instance = RET_POSITION_ADC_INSTANCE;
    context->adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    context->adc.Init.Resolution = ADC_RESOLUTION_12B;
    context->adc.Init.ScanConvMode = DISABLE;
    context->adc.Init.ContinuousConvMode = DISABLE;
    context->adc.Init.DiscontinuousConvMode = DISABLE;
    context->adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    context->adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    context->adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    context->adc.Init.NbrOfConversion = 1u;
    context->adc.Init.DMAContinuousRequests = DISABLE;
    context->adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&context->adc) != HAL_OK) {
        ret_stm32_error();
    }
    memset(&channel, 0, sizeof(channel));
    channel.Channel = RET_POSITION_ADC_CHANNEL;
    channel.Rank = 1u;
    channel.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    if (HAL_ADC_ConfigChannel(&context->adc, &channel) != HAL_OK) {
        ret_stm32_error();
    }
}

void ret_stm32_hardware_init(ret_stm32_context_t *context) {
    if (context == NULL) {
        ret_stm32_error();
    }
    memset(context, 0, sizeof(*context));
    context->adc_min = RET_ADC_DEFAULT_MIN;
    context->adc_max = RET_ADC_DEFAULT_MAX;
    context->tilt_min_tenths = 0;
    context->tilt_max_tenths = 100;
    context->motor_result = RET_MOTOR_IDLE;
    gpio_init();
    adc_init(context);
    uart_init(context);
}

bool ret_stm32_uart_read(ret_stm32_context_t *context, uint8_t *octet) {
    uint16_t tail;
    if ((context == NULL) || (octet == NULL) ||
        (context->uart_rx_tail == context->uart_rx_head)) {
        return false;
    }
    tail = context->uart_rx_tail;
    *octet = context->uart_rx_buffer[tail];
    context->uart_rx_tail = (uint16_t)((tail + 1u) & 0xFFu);
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    ret_stm32_context_t *context = &g_ret_stm32;
    if (uart->Instance == RET_UART_INSTANCE) {
        const uint16_t next =
            (uint16_t)((context->uart_rx_head + 1u) & 0xFFu);
        if (next != context->uart_rx_tail) {
            context->uart_rx_buffer[context->uart_rx_head] =
                context->uart_rx_octet;
            context->uart_rx_head = next;
        }
        (void)HAL_UART_Receive_IT(&context->uart, &context->uart_rx_octet, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart->Instance == RET_UART_INSTANCE) {
        __HAL_UART_CLEAR_OREFLAG(uart);
        (void)HAL_UART_Receive_IT(uart, &g_ret_stm32.uart_rx_octet, 1u);
    }
}

void ret_stm32_system_clock_config(void) {
    RCC_OscInitTypeDef oscillator;
    RCC_ClkInitTypeDef clock;
    memset(&oscillator, 0, sizeof(oscillator));
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 12u;
    oscillator.PLL.PLLN = 336u;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 7u;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        ret_stm32_error();
    }

    memset(&clock, 0, sizeof(clock));
    clock.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV4;
    clock.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_5) != HAL_OK) {
        ret_stm32_error();
    }
}

void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void ret_stm32_error(void) {
    __disable_irq();
    HAL_GPIO_WritePin(RET_MOTOR_IN1_PORT, RET_MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_MOTOR_IN2_PORT, RET_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_MOTOR_SLEEP_PORT, RET_MOTOR_SLEEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RET_RS485_DE_PORT, RET_RS485_DE_PIN, GPIO_PIN_RESET);
    for (;;) {
    }
}
