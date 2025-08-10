#pragma once

#include <memory>
#include <string>
#include <vector>

#include "esp_timer.h"
#include "sdkconfig.h"
#include "Events.h"

class EventProc {
public:
    virtual void Detected(Value* value_ptr) = 0;
    virtual void Cleared() = 0;
    virtual void TrackingUpdate(Value* value_ptr) = 0;
    virtual void PresenceUpdate(Value* value_ptr) = 0;
};

class RadarSensor {
public:
    explicit RadarSensor(EventProc* event_processor) : ep(event_processor) {}

    virtual std::vector<std::unique_ptr<Value>> get_decoded_radar_data() = 0;

    void process(float minimum_power = 0.0f);

protected:
    enum DetectionState {
        STATE_NOT_DETECTED,
        STATE_DETECTED_ONCE,
        STATE_DETECTED,
        STATE_CLEARED_ONCE
    };

    EventProc* ep = nullptr;
    DetectionState currentState = STATE_NOT_DETECTED;
    uint64_t lastDetectionTimeMs = 0;
};

