#include "bno055.hpp"
#include "stdio.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transport.h"
#include <cmath>

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

BNO055 imu(I2C_PORT);

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float euler_x, euler_y, euler_z;

    imu.read_gyro(&gyro_x, &gyro_y, &gyro_z);
    imu.read_accel(&accel_x, &accel_y, &accel_z);
    imu.read_orientation(&euler_x, &euler_y, &euler_z);

    const float deg_to_rad = 3.14159265358979323846f / 180.0f; // lol

    float yaw = euler_x * deg_to_rad;
    float roll = euler_y * deg_to_rad;
    float pitch = euler_z * deg_to_rad;

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

    imu_msg.angular_velocity.x = gyro_x * deg_to_rad;
    imu_msg.angular_velocity.y = gyro_y * deg_to_rad;
    imu_msg.angular_velocity.z = gyro_z * deg_to_rad;

    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;

    rcl_publish(&imu_pub, &imu_msg, NULL);
}

int main()
{
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    printf("Connected\n");

    while (!imu.begin(2, OpMode::IMU))
    {
        printf("Error: IMU failed to initialize\n");
        sleep_ms(100);
    }

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);
    sleep_ms(2000);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;

    rcl_ret_t rc;

    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK)
    {
        printf("support init failed\n");
        while (true)
        {
            sleep_ms(1000);
        }
    }
    rclc_node_init_default(&node, "bno055_node", "", &support);
    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");
    sensor_msgs__msg__Imu__init(&imu_msg);
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(20),
        timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        sleep_ms(1);
    }
}