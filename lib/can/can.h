#ifndef LIB_CAN_CAN_H
#define LIB_CAN_CAN_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/twai.h>

using can_frame_data = twai_message_t;

enum CAN_BaudRate {
    b25K,
    b50K,
    b100k,
    b125k,
    b250k,
    b500k,
    b800k,
    b1000k
};

namespace ESP_CAN {

inline esp_err_t CAN_Transmit(const can_frame_data frame, TickType_t ticks_to_wait) {
    return twai_transmit((twai_message_t*)&frame, ticks_to_wait);
}

inline esp_err_t CAN_Receive(const can_frame_data* frame, TickType_t ticks_to_wait) {
    return twai_receive((twai_message_t*)frame, ticks_to_wait);
}

class CAN {
private:
    twai_general_config_t set_pin;
    twai_timing_config_t set_baudrate;
    twai_filter_config_t set_filter;

public:
    void CAN_Setup(gpio_num_t TX_PIN, gpio_num_t RX_PIN, CAN_BaudRate baudrate);
    esp_err_t CAN_Start();
};

}

#endif