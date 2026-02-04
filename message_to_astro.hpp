#pragma once
#include "sensor_msgs/msg/joy.h"
#include <string>

#define MAX_BUTTONS_BYTE_1 8
#define MAX_BUTTONS_BYTE_2 7
#define MAX_AXIS 5

char *joy_to_bytes(sensor_msgs__msg__Joy joy_msg);
signed char joy_to_char(float analog_value);
void bytes_to_joy(sensor_msgs__msg__Joy *msg, uint8_t *bytes);