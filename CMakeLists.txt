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

set(PICO_BOARD pico CACHE STRING "Board type")


include(pico_sdk_import.cmake)



project(pico_robot C CXX ASM)
pico_sdk_init()


add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/RadioLib")


link_directories(libmicroros)
add_executable(pico_robot
    pico_robot.cpp
    ${CMAKE_CURRENT_LIST_DIR}/include/pico_robot.hpp
    pico_uart_transport.c
    pico_uart_transport.h
    "${CMAKE_CURRENT_SOURCE_DIR}/RadioLib/src/hal/RPiPico/PicoHal.cpp"
    message_to_astro.cpp
    message_to_astro.hpp
)


target_link_libraries(pico_robot
    pico_stdlib
    microros
    RadioLib
    hardware_gpio
    hardware_spi
    hardware_timer
    pico_multicore
    hardware_pwm
)

target_include_directories(pico_robot PUBLIC
    libmicroros/include
)


SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_uart(pico_robot 0)
pico_enable_stdio_usb(pico_robot 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
target_compile_definitions(${PROJECT_NAME} PUBLIC RADIOLIB_BUILD_RPI_PICO)

# Generate UF2
pico_add_extra_outputs(pico_robot)
