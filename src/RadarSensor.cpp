#include <string_view>

#include "esp_timer.h"
#include "sdkconfig.h"

#include "RadarSensor.h"

void RadarSensor::process(float minimum_power) {
    auto values_list = get_decoded_radar_data();

    bool no_target_found = true;
    for (auto& value_ptr : values_list) {
#if 1
        if (value_ptr->etype() == "no") {
            if (currentState == STATE_DETECTED || currentState == STATE_DETECTED_ONCE) {
                if (ep) {
                    ep->Cleared();
                }
                currentState = STATE_CLEARED_ONCE;
                return;
            } else {
                currentState = STATE_NOT_DETECTED;
                return;
            }
        }
#endif
        if (value_ptr->get_power() >= minimum_power) {
            no_target_found = false;
            break;
        }
    }

    bool sent_detected_event = false;
    switch (currentState) {
        case STATE_NOT_DETECTED:
            if (!no_target_found) {
                for (auto& value_ptr : values_list) {
                    if (std::string_view(value_ptr->etype()) != "no") {
                        if (ep) {
                            ep->Detected(value_ptr.get());
                        }
                        sent_detected_event = true;
                    }
                }
                currentState = STATE_DETECTED_ONCE;
            }
            break;

        case STATE_DETECTED_ONCE:
            currentState = STATE_DETECTED;
            lastDetectionTimeMs = esp_timer_get_time() / 1000;
            break;

        case STATE_DETECTED:
            if (no_target_found) {
                const uint64_t now_ms = esp_timer_get_time() / 1000;
                const uint64_t timeout_ms = static_cast<uint64_t>(CONFIG_LD1125_DETECTION_TIMEOUT_MS);
                if ((now_ms - lastDetectionTimeMs) > timeout_ms) {
                    if (ep) {
                        ep->Cleared();
                    }
                    currentState = STATE_CLEARED_ONCE;
                }
            } else {
                lastDetectionTimeMs = esp_timer_get_time() / 1000;
            }
            break;

        case STATE_CLEARED_ONCE:
            currentState = STATE_NOT_DETECTED;
            break;
    }

    if (!sent_detected_event) {
        for (auto& value_ptr : values_list) {
            if (std::string_view(value_ptr->etype()) != "no") {
                if (ep) {
                    ep->TrackingUpdate(value_ptr.get());
                    ep->PresenceUpdate(value_ptr.get());
                }
            }
        }
    }
}

