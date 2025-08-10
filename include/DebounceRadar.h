#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "RadarSensor.h"

class DebounceRadar : public RadarSensor {
public:
    DebounceRadar(RadarSensor* sensor,
                  EventProc* event_processor,
                  uint32_t interval_ms);

    std::vector<std::unique_ptr<Value>> get_decoded_radar_data() override;

private:
    RadarSensor* sensor_;
    uint32_t intervalMs_;
    uint64_t windowEndMs_ = 0;

    std::vector<std::unique_ptr<Value>> lastSent_;
    std::vector<std::unique_ptr<Value>> pending_;

    static std::vector<std::unique_ptr<Value>> Clone(
        const std::vector<std::unique_ptr<Value>>& src);

    static bool Equal(
        const std::vector<std::unique_ptr<Value>>& left_values,
        const std::vector<std::unique_ptr<Value>>& right_values);
};

