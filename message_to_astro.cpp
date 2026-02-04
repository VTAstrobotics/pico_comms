#include "sensor_msgs/msg/joy.h"
#include <string>
#include "message_to_astro.hpp"

char *joy_to_bytes(sensor_msgs__msg__Joy joy_msg)
{

    char *message_to_send = new char[10]; // Extra byte for null termination

    char command_code = 20;
    message_to_send[0] = command_code;

    char button1 = (joy_msg.buttons.data[0] << 7) |
                   (joy_msg.buttons.data[1] << 6) |
                   (joy_msg.buttons.data[2] << 5) |
                   (joy_msg.buttons.data[3] << 4) |
                   (joy_msg.buttons.data[4] << 3) |
                   (joy_msg.buttons.data[5] << 2) |
                   (joy_msg.buttons.data[6] << 1) |
                   (joy_msg.buttons.data[7] << 0);

    message_to_send[1] = button1;

    char button2 = (joy_msg.buttons.data[8] << 7) |
                   (joy_msg.buttons.data[9] << 6) |
                   (joy_msg.buttons.data[10] << 5) |
                   (joy_msg.buttons.data[11] << 4) |
                   (joy_msg.buttons.data[12] << 3) |
                   (joy_msg.buttons.data[13] << 2) |
                   (joy_msg.buttons.data[14] << 1) |
                   0;

    message_to_send[2] = button2;

    message_to_send[3] = joy_to_char(joy_msg.axes.data[0]);
    message_to_send[4] = joy_to_char(joy_msg.axes.data[1]);
    message_to_send[5] = joy_to_char(joy_msg.axes.data[2]);
    message_to_send[6] = joy_to_char(joy_msg.axes.data[3]);
    message_to_send[7] = joy_to_char(joy_msg.axes.data[4]);
    message_to_send[8] = joy_to_char(joy_msg.axes.data[5]);
    message_to_send[9] = '\0';

    return message_to_send;
}

signed char joy_to_char(float analog_value)
{
    signed char result;
    if (analog_value > 0)
    {
        result = analog_value * 127;
    }
    else if (analog_value < 0)
    {
        result = analog_value * 128;
    }
    else
    {
        result = 0;
    }

    return result;
}

void bytes_to_joy(sensor_msgs__msg__Joy *msg, uint8_t *bytes)
{

    // buttons
    for (int i = 0; i < 8; i++)
    {
        msg->buttons.data[i] = (bytes[1] >> ((7) - i)) & 0x01;
    }
    for (int i = 0; i < 7; i++)
    {
        msg->buttons.data[i + (8)] = (bytes[2] >> ((7) - i)) & 0x01;
    }

    // axes
    for (int i = 0; i < 6; i++)
    {
        msg->axes.data[i] = bytes[i + (3)];
    }
}