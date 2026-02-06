#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/string.h>

#include <RadioLib.h>
#include "hal/RPiPico/PicoHal.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <string.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"

#include "include/pico_base_station.hpp"
#include "message_to_astro.hpp"
// TODO: enable watchdog

extern "C"
{
#include "pico_uart_transport.h"
}

#define NUM_AXES 8
#define NUM_BUTTONS 20

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

int state = 0;

sensor_msgs__msg__Joy last_joy;

PicoHal *hal = new PicoHal(SPI_PORT, SPI_MISO, SPI_MOSI, SPI_SCK);
SX1262 radio = new Module(hal, RFM_NSS, RFM_DIO1, RFM_RST, RFM_BUSY);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;
volatile bool operationDone = false;
std_msgs__msg__String debug;

void setFlag(void)
{
    operationDone = true;
}
rcl_publisher_t debug_publisher;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

    char *data = joy_to_bytes(last_joy);
    state = radio.startTransmit(data, (size_t)10);
}

void joy_callback(const void *msg)
{
    if (msg == NULL)
    {
        return;
    }

    // Cast the void pointer to the correct message type
    
    const sensor_msgs__msg__Joy *joy_msg = (const sensor_msgs__msg__Joy *)msg;

    debug.data.size = 8;//strlen(8);
    debug.data.capacity = 9;//strlen(data) + 1;
    debug.data.data = joy_msg->axes.data[0];

    rcl_publish(&debug_publisher, &debug, NULL);

    // Copy the message safely
    sensor_msgs__msg__Joy__copy(joy_msg, &last_joy);
}

const uint LED_PIN = 25;

rcl_subscription_t joy_subscriber;
sensor_msgs__msg__Joy msg;

float last_joy_axes_storage[NUM_AXES];
int32_t last_joy_buttons_storage[NUM_BUTTONS];

float msg_axes_storage[NUM_AXES];
int32_t msg_buttons_storage[NUM_BUTTONS];
char debug_characters[NUM_AXES];

void init_joy_msgs_static()
{
    // Initialize messages
    sensor_msgs__msg__Joy__init(&last_joy);
    sensor_msgs__msg__Joy__init(&msg);

    // Assign static storage for axes
    last_joy.axes.data = last_joy_axes_storage;
    last_joy.axes.size = NUM_AXES;
    last_joy.axes.capacity = NUM_AXES;

    msg.axes.data = msg_axes_storage;
    msg.axes.size = NUM_AXES;
    msg.axes.capacity = NUM_AXES;

    // Assign static storage for buttons
    last_joy.buttons.data = last_joy_buttons_storage;
    last_joy.buttons.size = NUM_BUTTONS;
    last_joy.buttons.capacity = NUM_BUTTONS;

    msg.buttons.data = msg_buttons_storage;
    msg.buttons.size = NUM_BUTTONS;
    msg.buttons.capacity = NUM_BUTTONS;


    std_msgs__msg__String__init(&debug);
}

int main()
{

    // stdio_init_all();
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    init_joy_msgs_static();

    // while (!stdio_usb_connected()) {
    //     gpio_put(LED_PIN, 1);
    //     sleep_ms(50);
    //     gpio_put(LED_PIN, 0);
    //     sleep_ms(50);
    // }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    hal->pinMode(RFM_RST, 1);      // output
    hal->digitalWrite(RFM_RST, 1); // write high
    hal->spiBegin();               // fix from https://github.com/jgromes/RadioLib/issues/729

    state = radio.begin(FREQUENCY,
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

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 100;
    const uint8_t attempts = 1;
    gpio_put(LED_PIN, 0);

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    gpio_put(LED_PIN, 1);

    while (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK)
    {
        // Flash the LED to show we are searching for the agent
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_subscription_init_default(
        &joy_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "/joy");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_publisher_init_best_effort(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "tx_debug"

    );

    rclc_executor_init(&executor, &support.context, 12, &allocator);
    rclc_executor_add_subscription(&executor, &joy_subscriber, &msg, joy_callback, ALWAYS);
    rclc_executor_add_timer(&executor, &timer);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
