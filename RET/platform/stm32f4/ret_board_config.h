#ifndef RET_BOARD_CONFIG_H
#define RET_BOARD_CONFIG_H

/*
 * Pyboard/STM32F405RG wiring used by this firmware.
 *
 * AISG bus (an isolated, surge-protected RS-485 transceiver is mandatory):
 *   X9  / PB6  USART1_TX -> transceiver DI
 *   X10 / PB7  USART1_RX <- transceiver RO
 *   Y5  / PB12           -> transceiver DE and /RE
 *
 * DRV8833 and position feedback:
 *   Y1  / PC6  -> IN2
 *   Y2  / PC7  -> IN1
 *   Y3  / PB8  -> nSLEEP
 *   Y4  / PB9  <- nFAULT
 *   Y9  / PB10 <- minimum limit switch (active low)
 *   Y10 / PB11 <- maximum limit switch (active low)
 *   X1  / PA0  <- position potentiometer (ADC1_IN0, 0..3.3 V)
 */

#define RET_UART_INSTANCE USART1
#define RET_UART_TX_PORT GPIOB
#define RET_UART_TX_PIN GPIO_PIN_6
#define RET_UART_RX_PORT GPIOB
#define RET_UART_RX_PIN GPIO_PIN_7
#define RET_UART_GPIO_AF GPIO_AF7_USART1

#define RET_RS485_DE_PORT GPIOB
#define RET_RS485_DE_PIN GPIO_PIN_12

#define RET_MOTOR_IN1_PORT GPIOC
#define RET_MOTOR_IN1_PIN GPIO_PIN_7
#define RET_MOTOR_IN2_PORT GPIOC
#define RET_MOTOR_IN2_PIN GPIO_PIN_6
#define RET_MOTOR_SLEEP_PORT GPIOB
#define RET_MOTOR_SLEEP_PIN GPIO_PIN_8
#define RET_MOTOR_FAULT_PORT GPIOB
#define RET_MOTOR_FAULT_PIN GPIO_PIN_9

#define RET_LIMIT_MIN_PORT GPIOB
#define RET_LIMIT_MIN_PIN GPIO_PIN_10
#define RET_LIMIT_MAX_PORT GPIOB
#define RET_LIMIT_MAX_PIN GPIO_PIN_11

#define RET_POSITION_ADC_INSTANCE ADC1
#define RET_POSITION_ADC_PORT GPIOA
#define RET_POSITION_ADC_PIN GPIO_PIN_0
#define RET_POSITION_ADC_CHANNEL ADC_CHANNEL_0

/* Blue LED. PB4 keeps PA13/PA14 available for SWD. */
#define RET_STATUS_LED_PORT GPIOB
#define RET_STATUS_LED_PIN GPIO_PIN_4

#define RET_FLASH_SECTOR FLASH_SECTOR_11
#define RET_FLASH_ADDRESS 0x080E0000u
#define RET_FLASH_SIZE (128u * 1024u)

#define RET_ADC_DEFAULT_MIN 300u
#define RET_ADC_DEFAULT_MAX 3795u
#define RET_ADC_MINIMUM_SPAN 1000u
#define RET_POSITION_TOLERANCE_TENTHS 1
#define RET_MOTOR_STALL_TIMEOUT_MS 5000u
#define RET_MOTOR_SETTLE_MS 50u

#endif
