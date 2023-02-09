#include <can.h>

namespace ESP_CAN {
    void CAN::CAN_Setup(gpio_num_t TX_PIN, gpio_num_t RX_PIN, CAN_BaudRate baudrate) {
        set_pin = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
        set_filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        switch (baudrate) {
            case b50K:
                set_baudrate = TWAI_TIMING_CONFIG_50KBITS();
                break;
            case b100k:
                set_baudrate = TWAI_TIMING_CONFIG_100KBITS();
                break;
            case b125k:
                set_baudrate = TWAI_TIMING_CONFIG_125KBITS();
                break;
            case b250k:
                set_baudrate = TWAI_TIMING_CONFIG_250KBITS();
                break;
            case b500k:
                set_baudrate = TWAI_TIMING_CONFIG_500KBITS();
                break;
            case b800k:
                set_baudrate = TWAI_TIMING_CONFIG_800KBITS();
                break;
            case b1000k:
                set_baudrate = TWAI_TIMING_CONFIG_1MBITS();
                break;
            default: 
                break;
        }
    }

    esp_err_t CAN::CAN_Start() {
        if (twai_driver_install(&set_pin, &set_baudrate, &set_filter) != ESP_OK){
            Serial.println("Install CAN Driver Failed");
            return ESP_FAIL;
        }
        if (twai_start() != ESP_OK) {
            Serial.println("Start CAN Failed");
            return ESP_FAIL;
        }
        Serial.println("CAN Started");
        return ESP_OK;
    }
}