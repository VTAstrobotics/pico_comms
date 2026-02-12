#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joy.h>
#include "hardware/uart.h"

// #include "hal/RPiPico/PicoHal.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <string.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transport.h"
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <string.h>

const uint LED_PIN = 25;

rcl_publisher_t publisher;
sensor_msgs__msg__NavSatFix msg;

#define UART_ID uart0
#define BAUD_RATE 38400
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

// subject to change
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// need to have interrupt on UART message. After uart message is recieved, publish message
// circular buffer?

static volatile char gps_char;
static volatile bool received_gps_message = false;
static volatile bool ready_to_publish = false;
static volatile char gps_buffer_0[100];

static volatile char gps_buffer_1[100];

static volatile buff_select = 0;

static int head = 0;

void on_uart_rx(void)
{
    static int i = 0;

    char *gps_buffer_internal = (buff_select) ? (gps_buffer_1) : (gps_buffer_0);

    while (uart_is_readable(UART_ID))
    {
        gps_char = uart_getc(UART_ID);
        received_gps_message = true;
        if (i < (int)sizeof(gps_buffer_0) - 1) // prevent overflows
        {
            gps_buffer_internal[i++] = gps_char;
        }
        else
        {
            i = 0;
        }
        if (gps_char == '\n')
        {
            ready_to_publish = true;
            gps_buffer_internal[i] = '\0';
            i = 0;
            buff_select = !buff_select;
        }
    }
}

int parse_nmea_fields(char *line, char *fields[], int max_fields) //store character in array of pointers
{
    int n = 0;
    char *tok = strtok(line, ",");          

    while (tok && n < max_fields) {
        fields[n++] = tok;                  
        tok = strtok(NULL, ",");
    }
    return n;                              
}


void handle_navsat_publishing(rcl_publisher_t *publisher, sensor_msgs__msg__NavSatFix *msg)
{

    char fields_copy[100][8];

    parse_nmea_fields()

    char *gps_buffer_internal = (buff_select) ? (gps_buffer_1) : (gps_buffer_0);

    if (ready_to_publish)
    {
        ready_to_publish = false;
    }
}

int main()
{
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    /// \end:uart_advanced[]
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

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
    gpio_put(LED_PIN, 1);

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, sensor_msgs__msg__NavSatFix),
        "pico_publisher");

    rclc_executor_init(&executor, &support.context, 1, &allocator);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
