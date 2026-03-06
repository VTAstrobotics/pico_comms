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

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13

BNO055 imu(I2C_PORT);

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL) {
        return;
    }

    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float orientation_x, orientation_y, orientation_z;

    imu.read_gyro(&gyro_x, &gyro_y, &gyro_z);
    imu.read_accel(&accel_x, &accel_y, &accel_z);
    imu.read_orientation(&orientation_x, &orientation_y, &orientation_z); //you can add gravity if needed e.g.  
    // imu.read_gravity(&grav_x, &grav_y, &grav_z);


    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;

    imu_msg.orientation.x = orientation_x;
    imu_msg.orientation.y = orientation_y;
    imu_msg.orientation.z = orientation_z;
    imu_msg.orientation.w = 1.0;

    imu_msg.orientation_covariance[0] = -1.0;

    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    rcl_publish(&imu_pub, &imu_msg, NULL);
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    printf("Connected\n");

    while (!imu.begin(2, OpMode::IMU)) {
        printf("Error: IMU failed to initialize\n");
        sleep_ms(100);
    }

    set_microros_transports();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;

    rcl_ret_t rc;

    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        printf("support init failed\n");
        while (true) { sleep_ms(1000); }
    }
    rclc_node_init_default(&node, "bno055_node", "", &support);
    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );
    imu_msg = sensor_msgs__msg__Imu__create()[0];
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(20),
        timer_callback
    );
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        sleep_ms(1);
    }
}