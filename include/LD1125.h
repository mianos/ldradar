#pragma once

#include <memory>
#include <string>
#include <vector>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "RadarSensor.h"

class LD1125 : public RadarSensor {
    static constexpr const char* TAG = "LD1125";
public:
    // The application owns UART configuration and driver install.
    explicit LD1125(EventProc* event_processor, uart_port_t uart_port_in)
        : RadarSensor(event_processor),
          uartPort(uart_port_in) {
    }

    // Optional: call when you want to ensure module is in test mode.
    void verifyTestMode() {
        static constexpr char testModeCmd[] = "test_mode=1\r\n";
        static constexpr char saveCmd[] = "save\r\n";

        ESP_LOGI(TAG, "test mode called =========================================== ");
        enum class ModeState { IDLE, SENT, ACKED, TIMEOUT };

        ModeState mode_state = ModeState::IDLE;
        std::string response_buffer;
        int64_t start_time_ms = esp_timer_get_time() / 1000;

        uart_flush_input(uartPort);
        uart_write_bytes(uartPort, testModeCmd, sizeof(testModeCmd) - 1);
        mode_state = ModeState::SENT;

        while ((esp_timer_get_time() / 1000 - start_time_ms) < 1000) {
            uint8_t byte_value = 0;
            if (uart_read_bytes(uartPort, &byte_value, 1, 10 / portTICK_PERIOD_MS) > 0) {
                response_buffer.push_back(static_cast<char>(byte_value));
                if (response_buffer.find("str=") != std::string::npos) {
                    mode_state = ModeState::ACKED;
                    break;
                }
            }
        }

        if (mode_state == ModeState::ACKED) {
            uart_write_bytes(uartPort, saveCmd, sizeof(saveCmd) - 1);
            ESP_LOGI(TAG, "Test mode enabled and configuration saved.");
        } else {
            ESP_LOGI(TAG, "Failed to enable test mode: timeout.");
        }
    }

    std::vector<std::unique_ptr<Value>> get_decoded_radar_data() override {
        std::vector<std::unique_ptr<Value>> values_list;

        enum class ParseState { WAIT, OCC_MOV, DIS, STR };
        static ParseState parse_state = ParseState::WAIT;

        const int64_t timeout_ms = 5000;
        int64_t start_time_ms = esp_timer_get_time() / 1000;

        std::string distance_text;
        std::string strength_text;
        std::string result_type;
        uint8_t byte_value = 0;

        while (true) {
            if ((esp_timer_get_time() / 1000 - start_time_ms) >= timeout_ms) {
                break;
            }

            int length_read = uart_read_bytes(uartPort, &byte_value, 1, 10 / portTICK_PERIOD_MS);
            if (length_read <= 0) {
                break;
            }

            char incoming_char = static_cast<char>(byte_value);
            switch (parse_state) {
                case ParseState::WAIT:
                    if (incoming_char == 'o' || incoming_char == 'm') {
                        parse_state = ParseState::OCC_MOV;
                        result_type.push_back(incoming_char);
                    }
                    break;

                case ParseState::OCC_MOV:
                    result_type.push_back(incoming_char);
                    if (result_type == "occ," || result_type == "mov,") {
                        parse_state = ParseState::DIS;
                    } else if (result_type.length() > 4) {
                        result_type.clear();
                        parse_state = ParseState::WAIT;
                    }
                    break;

                case ParseState::DIS:
                    if (incoming_char == ',') {
                        parse_state = ParseState::STR;
                    } else if (incoming_char != ' ' &&
                               incoming_char != 'd' &&
                               incoming_char != 'i' &&
                               incoming_char != 's' &&
                               incoming_char != '=') {
                        distance_text.push_back(incoming_char);
                    }
                    break;

                case ParseState::STR:
                    if (incoming_char == '\n') {
                        std::string final_type = result_type.substr(0, result_type.length() - 1);
                        result_type.clear();

                        float power_value = 0.0f;
                        if (!strength_text.empty()) {
                            power_value = std::stof(strength_text) / 10.0f;
                            strength_text.clear();
                        }

                        float distance_value = std::stof(distance_text);
                        distance_text.clear();

                        if (final_type == "occ") {
                            values_list.push_back(std::make_unique<Occupancy>(distance_value, power_value));
                        } else {
                            values_list.push_back(std::make_unique<Movement>(distance_value, power_value));
                        }

                        parse_state = ParseState::WAIT;
                        return values_list;
                    } else if (incoming_char != ' ' &&
                               incoming_char != 's' &&
                               incoming_char != 't' &&
                               incoming_char != 'r' &&
                               incoming_char != '=') {
                        strength_text.push_back(incoming_char);
                    }
                    break;
            }
        }

        return values_list;
    }

private:
    const uart_port_t uartPort;
};

