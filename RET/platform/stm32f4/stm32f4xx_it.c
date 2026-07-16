#include "ret_stm32_platform.h"

void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&g_ret_stm32.uart);
}
