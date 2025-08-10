#include "DebounceRadar.h"

#include <cmath>
#include "esp_timer.h"
#include "sdkconfig.h"

static inline float mainEps() {
#ifdef CONFIG_LD1125_EQUAL_MAIN_EPS_MILLI
    return static_cast<float>(CONFIG_LD1125_EQUAL_MAIN_EPS_MILLI) / 1000.0f;
#else
    return 0.0f;
#endif
}

static inline float powerEps() {
#ifdef CONFIG_LD1125_EQUAL_POWER_EPS_MILLI
    return static_cast<float>(CONFIG_LD1125_EQUAL_POWER_EPS_MILLI) / 1000.0f;
#else
    return 0.0f;
#endif
}

DebounceRadar::DebounceRadar(RadarSensor* sensor,
                             EventProc* event_processor,
                             uint32_t interval_ms)
    : RadarSensor(event_processor),
      sensor_(sensor),
      intervalMs_(interval_ms),
      windowEndMs_(0) {
}

std::vector<std::unique_ptr<Value>> DebounceRadar::get_decoded_radar_data() {
    const uint64_t nowMs = static_cast<uint64_t>(esp_timer_get_time() / 1000);
    const bool windowOpen = nowMs < windowEndMs_;

    // Trailing edge first: if window closed and a pending exists, emit it.
    if (!windowOpen && !pending_.empty()) {
        auto toSend = std::move(pending_);
        pending_.clear();
        lastSent_ = Clone(toSend);
        windowEndMs_ = nowMs + intervalMs_;
        return toSend;
    }

    auto rawValues = sensor_->get_decoded_radar_data();
    if (rawValues.empty()) {
        return {};
    }

    // First emission ever: send immediately and start window.
    if (lastSent_.empty()) {
        lastSent_ = Clone(rawValues);
        windowEndMs_ = nowMs + intervalMs_;
        return rawValues;
    }

    if (windowOpen) {
        // Inside window: remember latest only; suppress output.
        pending_ = Clone(rawValues);
        return {};
    }

    // Window closed: if changed beyond tolerance, send immediately and restart window.
    if (!Equal(rawValues, lastSent_)) {
        lastSent_ = Clone(rawValues);
        windowEndMs_ = nowMs + intervalMs_;
        return rawValues;
    }

    // No change and no pending: nothing to emit.
    return {};
}

std::vector<std::unique_ptr<Value>> DebounceRadar::Clone(
    const std::vector<std::unique_ptr<Value>>& src) {
    std::vector<std::unique_ptr<Value>> dst;
    dst.reserve(src.size());
    for (const auto& value_ptr : src) {
        dst.push_back(value_ptr->clone());
    }
    return dst;
}

bool DebounceRadar::Equal(
    const std::vector<std::unique_ptr<Value>>& left_values,
    const std::vector<std::unique_ptr<Value>>& right_values) {
    if (left_values.size() != right_values.size()) {
        return false;
    }

    const float epsMain = mainEps();
    const float epsPower = powerEps();

    for (size_t index = 0; index < left_values.size(); ++index) {
        const std::string typeLeft  = left_values[index]->etype();
        const std::string typeRight = right_values[index]->etype();
        if (typeLeft != typeRight) {
            return false;
        }

        if (epsMain == 0.0f && epsPower == 0.0f) {
            if (!left_values[index]->isEqual(*right_values[index])) {
                return false;
            }
        } else {
            const float mainLeft  = left_values[index]->get_main();
            const float mainRight = right_values[index]->get_main();
            const float powerLeft  = left_values[index]->get_power();
            const float powerRight = right_values[index]->get_power();
            if (std::fabs(mainLeft - mainRight) > epsMain) {
                return false;
            }
            if (std::fabs(powerLeft - powerRight) > epsPower) {
                return false;
            }
        }
    }
    return true;
}

