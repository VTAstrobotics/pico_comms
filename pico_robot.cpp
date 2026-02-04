#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joy.h>

#include <RadioLib.h>
#include "hal/RPiPico/PicoHal.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <string.h>

#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "include/pico_robot.hpp"
#include "message_to_astro.hpp"

extern "C"
{
#include "pico_uart_transport.h"
}

#define FREQUENCY 915.000   //
#define BANDWIDTH 125.0     // Sets LoRa bandwidth. Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define SPREADING_FACTOR 7  // Sets LoRa spreading factor. Allowed values range from 5 to 12.
#define CODING_RATE 5       // Sets LoRa coding rate 4/x denominator. Allowed x values range from 5 to 8.
#define CURRENT_LIMIT 140   // mA
#define OUTPUT_POWER 22     // dBm
#define LORA_PREAMBLE_LEN 8 // preambleLength LoRa preamble length in symbols. Allowed values range from 1 to 65535.
#define SYNC_WORD 0x12      // public default=0x12,  LoRaWAN default=0x34
#define LDRO false          // normally 'true' for SF-11 or 12
#define CRC true
#define IQINVERTED false
#define DATA_SHAPING RADIOLIB_SHAPING_1_0 // Data shaping = 1.0
#define TCXO_VOLTAGE 1.7                  // volts
#define WHITENING_INITIAL 0x00FF          // initial whitening LFSR value

PicoHal *hal = new PicoHal(SPI_PORT, SPI_MISO, SPI_MOSI, SPI_SCK);
SX1262 radio = new Module(hal, RFM_NSS, RFM_DIO1, RFM_RST, RFM_BUSY);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;

volatile bool operationDone = false;
void setFlag(void)
{
    operationDone = true;
    // // printf("INTERRUPT\n");
}

const uint LED_PIN = 25;

rcl_publisher_t publisher;
sensor_msgs__msg__Joy msg;

int main()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    stdio_init_all();

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    float axes_data[6];
    int32_t buttons_data[15];

    msg.axes.capacity = 6;
    msg.axes.data = axes_data;
    msg.buttons.capacity = 15;
    msg.buttons.data = buttons_data;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "/joy");
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    gpio_put(LED_PIN, 1);

    hal->pinMode(RFM_RST, 1);      // output
    hal->digitalWrite(RFM_RST, 1); // write high
    hal->spiBegin();               // fix from https://github.com/jgromes/RadioLib/issues/729

    int state = radio.begin(FREQUENCY,
                            BANDWIDTH,
                            SPREADING_FACTOR,
                            CODING_RATE,
                            SYNC_WORD,
                            OUTPUT_POWER,
                            LORA_PREAMBLE_LEN,
                            TCXO_VOLTAGE);
    radio.setCurrentLimit(CURRENT_LIMIT);
    radio.forceLDRO(LDRO);
    radio.setCRC(CRC);
    radio.invertIQ(IQINVERTED);
    radio.setWhitening(true, WHITENING_INITIAL);
    radio.explicitHeader();
    radio.setDio1Action(setFlag);

    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        // printf("success!");
    }
    else
    {
        // printf("failed, code ");
        while (true)
        {
            sleep_ms(10);
        }
    }

    while (true)
    {
        // printf("TESTING");
        if (operationDone)
        {
            operationDone = false;
            uint8_t str[11];
            int state = radio.readData(str, 11);

            if (state == RADIOLIB_ERR_NONE)
            {

                // // printf("[SX1262] Data:\t\t");
                // // printf("%s \n", (char *)str);
                char command_code = str[0];
                bytes_to_joy(&msg, str);

                rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
            }
            hal->delay(100);
            state = radio.startReceive();
            // printf("LISTENING\n");
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
