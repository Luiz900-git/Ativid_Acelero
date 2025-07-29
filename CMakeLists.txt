# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==

# ====================================================================================

cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(Usar_SSD C CXX ASM)
pico_sdk_init()


add_subdirectory(lib/FatFs_SPI)    

include_directories( ${CMAKE_SOURCE_DIR}/inc ) # Inclui os files .h na pasta lib


add_executable(${PROJECT_NAME}  
        Cartao_FatFS_SPI.c
        hw_config.c
        )

    

target_link_libraries(${PROJECT_NAME} 
        pico_stdlib 
         hardware_pwm
        hardware_i2c
        FatFs_SPI
        hardware_clocks
        hardware_adc
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})



