#include "sensor_msgs/msg/joy.h"
#include <string>
#include "message_to_astro.hpp"

std::string joy_to_bytes(sensor_msgs__msg__Joy joy_msg){

    char message_to_send[10];//Extra byte for null termination

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

    return std::string(message_to_send);
}

signed char joy_to_char(float analog_value){
    signed char result;
    if(analog_value > 0){
        result = analog_value * 127;
    }
    else if(analog_value < 0){
        result = analog_value * 128;
    }
    else{
        result = 0;
    }

    return result;
}
