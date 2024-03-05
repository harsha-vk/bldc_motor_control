cmake_minimum_required(VERSION 3.2)

set(CMAKE_TOOLCHAIN_FILE arm-none-eabi.cmake)

project(bldc_motor_control VERSION 1.0)

enable_language(C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(ASM_SOURCES Core/Startup/startup_stm32f302r8tx.s)
set(C_SOURCES Core/Src/main.c
            Core/Src/stm32f3xx_hal_msp.c
            Core/Src/stm32f3xx_it.c
            Core/Src/syscalls.c
            Core/Src/sysmem.c
            Core/Src/system_stm32f3xx.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
            Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
)
set(CXX_SOURCES Core/Src/Main/application.cpp)

set(ASM_INCLUDES)
set(C_INCLUDES ${CMAKE_SOURCE_DIR}/Core/Inc
            ${CMAKE_SOURCE_DIR}/Core/Inc/Main
            ${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc
            ${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
            ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
            ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)
set(CXX_INCLUDES ${CMAKE_SOURCE_DIR}/Core/Inc
            ${CMAKE_SOURCE_DIR}/Core/Inc/Main
            ${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc
            ${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
            ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
            ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

set(ASM_DEFS # -DDEBUG
)
set(C_DEFS -DUSE_HAL_DRIVER
        -DSTM32F302x8
        # -DDEBUG
)
set(CXX_DEFS -DUSE_HAL_DRIVER
        -DSTM32F302x8
        # -DDEBUG
)

set(MCU_PARAMS -mcpu=cortex-m4
            -mfpu=fpv4-sp-d16
            -mfloat-abi=hard
            -mthumb
)

set(ASM_FLAGS -g3
            -x assembler-with-cpp
            --specs=nano.specs)
set(C_FLAGS -g3
            -O0
            -Wall
            -ffunction-sections
            -fdata-sections
            -fstack-usage
            -fcyclomatic-complexity
            --specs=nano.specs
)
set(CXX_FLAGS -g3
            -O0
            -Wall
            -ffunction-sections
            -fdata-sections
            -fstack-usage
            -fcyclomatic-complexity 
            -fno-exceptions
            -fno-rtti
            -fno-use-cxa-atexit
            --specs=nano.specs
)

add_executable(${CMAKE_PROJECT_NAME})

target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${ASM_SOURCES}
                                            ${C_SOURCES}
                                            ${CXX_SOURCES}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:ASM>:${ASM_INCLUDES}>
                                                        $<$<COMPILE_LANGUAGE:C>:${C_INCLUDES}>
                                                        $<$<COMPILE_LANGUAGE:CXX>:${CXX_INCLUDES}>
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:ASM>:${ASM_DEFS}>
                                                        $<$<COMPILE_LANGUAGE:C>:${C_DEFS}>
                                                        $<$<COMPILE_LANGUAGE:CXX>:${CXX_DEFS}>
)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE ${MCU_PARAMS}
                                                    $<$<COMPILE_LANGUAGE:ASM>:${ASM_FLAGS}>
                                                    $<$<COMPILE_LANGUAGE:C>:${C_FLAGS}>
                                                    $<$<COMPILE_LANGUAGE:CXX>:${CXX_FLAGS}>
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE ${MCU_PARAMS}
                                                    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F302R8TX_FLASH.ld
                                                    --specs=nosys.specs
                                                    --specs=nano.specs
                                                    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
                                                    -Wl,--gc-sections
                                                    # -u _printf_float
                                                    # -u _scanf_float
                                                    -Wl,--start-group
                                                    -lc
                                                    -lm
                                                    -lstdc++
                                                    -lsupc++
                                                    -Wl,--end-group
                                                    -Wl,--print-memory-usage
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
                            COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
                            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
