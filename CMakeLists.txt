# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_FETCH_FROM_GIT on)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD pico2 CACHE STRING "Board type")


include(pico_sdk_import.cmake)



project(pico_base_station C CXX ASM)

pico_sdk_init()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/RadioLib")


link_directories(libmicroros)
add_executable(pico_base_station
    pico_base_station.cpp
    ${CMAKE_CURRENT_LIST_DIR}/include/pico_base_station.hpp
    pico_uart_transport.c
    pico_uart_transport.h 
    message_to_astro.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/RadioLib/src/hal/RPiPico/PicoHal.cpp"

)



target_link_libraries(pico_base_station
    pico_stdlib
    microros
    RadioLib
    hardware_gpio
    hardware_spi
    hardware_timer
    pico_multicore
    hardware_pwm
)

target_include_directories(pico_base_station PUBLIC
    libmicroros/include
)


SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_base_station 1)
pico_enable_stdio_uart(pico_base_station 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
target_compile_definitions(${PROJECT_NAME} PUBLIC RADIOLIB_BUILD_RPI_PICO)

# Generate UF2
pico_add_extra_outputs(pico_base_station)
