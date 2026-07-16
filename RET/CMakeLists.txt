cmake_minimum_required(VERSION 3.20)
project(ret_aisg2_firmware VERSION 2.0.0 LANGUAGES C)

option(RET_BUILD_TESTS "Build protocol tests for the host" ON)
option(RET_BUILD_STM32 "Build firmware for STM32F405RG" OFF)
option(RET_BUILD_HOST "Build the POSIX bench RET executable" ON)

add_library(ret_core STATIC
    src/ret_hdlc.c
    src/ret_protocol.c
)
target_include_directories(ret_core PUBLIC include)
target_compile_features(ret_core PUBLIC c_std_11)
target_compile_options(ret_core PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow
)

if(RET_BUILD_HOST AND UNIX AND NOT CMAKE_CROSSCOMPILING)
    add_executable(ret_host platform/host/ret_host.c)
    target_link_libraries(ret_host PRIVATE ret_core)
    target_compile_options(ret_host PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
endif()

if(RET_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_executable(ret_tests tests/test_ret.c)
    target_link_libraries(ret_tests PRIVATE ret_core)
    target_compile_options(ret_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
    add_test(NAME ret_aisg2_protocol COMMAND ret_tests)
endif()

if(RET_BUILD_STM32)
    enable_language(ASM)
    if(NOT DEFINED STM32_CUBE_F4_PATH)
        message(FATAL_ERROR
            "Set STM32_CUBE_F4_PATH to an STM32CubeF4 checkout")
    endif()
    if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
        message(FATAL_ERROR "The STM32 target currently supports GNU Arm Embedded")
    endif()

    set(HAL ${STM32_CUBE_F4_PATH}/Drivers/STM32F4xx_HAL_Driver)
    set(CMSIS ${STM32_CUBE_F4_PATH}/Drivers/CMSIS)
    add_executable(ret_stm32f405.elf
        platform/stm32f4/main.c
        platform/stm32f4/ret_stm32_platform.c
        platform/stm32f4/ret_flash_storage.c
        platform/stm32f4/stm32f4xx_it.c
        ${HAL}/Src/stm32f4xx_hal.c
        ${HAL}/Src/stm32f4xx_hal_cortex.c
        ${HAL}/Src/stm32f4xx_hal_dma.c
        ${HAL}/Src/stm32f4xx_hal_rcc.c
        ${HAL}/Src/stm32f4xx_hal_rcc_ex.c
        ${HAL}/Src/stm32f4xx_hal_gpio.c
        ${HAL}/Src/stm32f4xx_hal_uart.c
        ${HAL}/Src/stm32f4xx_hal_flash.c
        ${HAL}/Src/stm32f4xx_hal_flash_ex.c
        ${HAL}/Src/stm32f4xx_hal_adc.c
        ${HAL}/Src/stm32f4xx_hal_adc_ex.c
        ${CMSIS}/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
        ${CMSIS}/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s
    )
    target_link_libraries(ret_stm32f405.elf PRIVATE ret_core)
    target_include_directories(ret_stm32f405.elf PRIVATE
        platform/stm32f4
        ${HAL}/Inc
        ${CMSIS}/Include
        ${CMSIS}/Device/ST/STM32F4xx/Include
    )
    target_compile_definitions(ret_stm32f405.elf PRIVATE
        USE_HAL_DRIVER STM32F405xx HSE_VALUE=12000000U
    )
    set(MCU_FLAGS
        -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    )
    target_compile_options(ret_core PRIVATE ${MCU_FLAGS})
    target_compile_options(ret_stm32f405.elf PRIVATE
        ${MCU_FLAGS} -ffunction-sections -fdata-sections
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
    target_link_options(ret_stm32f405.elf PRIVATE
        ${MCU_FLAGS}
        -T${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32f4/stm32f405rg_flash.ld
        --specs=nano.specs --specs=nosys.specs
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/ret_stm32f405.map
    )
    add_custom_command(TARGET ret_stm32f405.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:ret_stm32f405.elf> ret_stm32f405.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:ret_stm32f405.elf> ret_stm32f405.bin
        VERBATIM
    )
endif()
