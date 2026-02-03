#pragma once
#include "sensor_msgs/msg/joy.h"
#include <string>

std::string joy_to_bytes(sensor_msgs__msg__Joy joy_msg);
signed char joy_to_char(float analog_value);
