#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joy.h>
#include "hardware/uart.h"

// #include "hal/RPiPico/PicoHal.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <string>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transport.h"
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <vector>

const uint LED_PIN = 25;

rcl_publisher_t publisher;
sensor_msgs__msg__NavSatFix msg;

#define UART_ID uart1
#define BAUD_RATE 38400
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

// subject to change
#define UART_TX_PIN 8
#define UART_RX_PIN 9

static char gps_char;
static bool received_gps_message = false;
static volatile bool ready_to_publish = false;
static char gps_buffer_0[200];

static char gps_buffer_1[200];

static volatile bool buff_select = 0;

static int head = 0;

static float longitude;
static float latitude;
static float lat_direction;
static float long_direction;

static const int LONG_FIELD = 4; // latitude value
static const int LAT_FIELD = 2;  // longitude value
static const int LAT_DIR = 3;    // N or S
static const int LONG_DIR = 5;   // W or E

rcl_publisher_t navsat_publisher;
// rcl_publisher_t lon_publisher;

// sensor_msgs__msg__NavSatFix lat_msg;
sensor_msgs__msg__NavSatFix navsat_msg;

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

static double nmea_to_decimal_degrees(double ddmm)
{
    int deg = (int)(ddmm / 100.0);
    double minutes = ddmm - (deg * 100.0);
    return (double)deg + (minutes / 60.0);
}

void handle_navsat_publishing()
{
    if (!ready_to_publish)
    {
        return;
    }

    char *completed = (!buff_select) ? (gps_buffer_1) : (gps_buffer_0);

    char local_line[200];

    int UART_IRQ = UART1_IRQ;
    irq_set_enabled(UART_IRQ, false);
    strncpy(local_line, completed, sizeof(local_line)); // turn off interrupt while copying
    local_line[sizeof(local_line) - 1] = '\0';
    ready_to_publish = false;
    irq_set_enabled(UART_IRQ, true);

    std::vector<std::string> gps_fields;

    const char *delimiter = ",";
    std::string gps1(local_line);

    size_t start = 0;
    while (true)
    {
        size_t pos = gps1.find(delimiter, start);
        if (pos == std::string::npos)
        {
            gps_fields.push_back(gps1.substr(start));
            break;
        }
        gps_fields.push_back(gps1.substr(start, pos - start));
        start = pos + 1;
    }

    if (gps_fields.size() > (size_t)LONG_DIR && gps_fields[0] == "$GPGGA")
    {
        if (!gps_fields[LAT_FIELD].empty() && !gps_fields[LONG_FIELD].empty() &&
            !gps_fields[LAT_DIR].empty() && !gps_fields[LONG_DIR].empty())
        {
            double raw_lat = std::stod(gps_fields[LAT_FIELD]);
            double raw_lon = std::stod(gps_fields[LONG_FIELD]);

            char ns = gps_fields[LAT_DIR][0];
            char ew = gps_fields[LONG_DIR][0];

            double lat_dd = nmea_to_decimal_degrees(raw_lat);
            double lon_dd = nmea_to_decimal_degrees(raw_lon);

            if (ns == 'S')
                lat_dd = -lat_dd;
            if (ew == 'W')
                lon_dd = -lon_dd;

            navsat_msg.latitude = lat_dd;
            navsat_msg.longitude = lon_dd;

            rcl_ret_t pub_ret = rcl_publish(&navsat_publisher, &navsat_msg, NULL);
            (void)pub_ret;
        }
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

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

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

    sensor_msgs__msg__NavSatFix__init(&navsat_msg);
    rclc_publisher_init_default(
        &navsat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "navsat_publisher");

    // rclc_publisher_init_default(
    //     &lon_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
    //     "lon_publisher");

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "pico_publisher");

    rclc_executor_init(&executor, &support.context, 1, &allocator);

    while (true)
    {
        if (ready_to_publish)
        {
            handle_navsat_publishing();
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
