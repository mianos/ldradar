# HkLink LD1125 Radar Module for ESP-IDF

High level ESP-IDF C++ component for reading and debouncing LD1125 mmWave radar output over UART, reporting Presence, Tracking, and Detected/Cleared events with optional JSON rendering.

---

## Features

* LD1125 UART line-parser with power and distance extraction
* Event model encapsulated as `Value` types: `Movement`, `Occupancy`, `Range`, `NoTarget`
* State machine that emits `Detected`, `TrackingUpdate`, `PresenceUpdate`, and `Cleared`
* Debounce wrapper to limit noisy or overly frequent updates while preserving last observation
* Lightweight JSON serialization via `JsonWrapper`
* Configurable timeouts and per-feed throttling using `menuconfig`
* C++23, header-only interfaces for events, zero dynamic ownership leakage using `std::unique_ptr`

> The parser expects LD1125 text frames like:
>
> ```
> mov,dis=0.23,str=321\n
> occ,dis=1.80,str=95\n
> ```

---

## Directory Layout

```
components/hklink_ld1125/
├── CMakeLists.txt
├── Kconfig
├── include/
│   ├── DebounceRadar.h
│   ├── Events.h
│   ├── LD1125.h
│   └── RadarSensor.h
└── src/
    ├── DebounceRadar.cpp
    └── RadarSensor.cpp
```

---

## Build Integration

Add the component directory to your project and ensure `components/hklink_ld1125` is discoverable by your root `CMakeLists.txt` or `idf.py add-dependency` setup.

### Component CMake

```cmake
idf_component_register(
    SRCS
        "src/RadarSensor.cpp"
        "src/DebounceRadar.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_timer
        log
        jsonwrapper
)
# target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)
```

> Requires ESP-IDF components: `driver`, `esp_timer`, `log`, and a `jsonwrapper` helper present in your workspace.

---

## Configuration

Open `menuconfig` and navigate to: `LD radar`

### Options

* **Detection clear timeout (ms)** `CONFIG_LD1125_DETECTION_TIMEOUT_MS`

  * Time from last valid detection before a Cleared event is emitted.
  * Default: `3000`.

* **Presence minimum update interval (ms, 0 disables throttling)** `CONFIG_LD1125_PRESENCE_INTERVAL_MS`

  * Lower-bound interval between Presence updates.
  * Default: `0`.

* **Tracking minimum update interval (ms, 0 disables throttling)** `CONFIG_LD1125_TRACKING_INTERVAL_MS`

  * Lower-bound interval between Tracking updates.
  * Default: `0`.

#### Optional tolerances used by `DebounceRadar`

If your project defines the following symbols, `DebounceRadar` will use them to treat near-equal readings as unchanged:

* `CONFIG_LD1125_EQUAL_MAIN_EPS_MILLI` distance/speed epsilon in millivalue units
* `CONFIG_LD1125_EQUAL_POWER_EPS_MILLI` power epsilon in millivalue units

Example: set `CONFIG_LD1125_EQUAL_MAIN_EPS_MILLI=50` to apply 0.05 tolerance.

---

## API Overview

### Base interface: `RadarSensor`

```c++
class EventProc {
public:
    virtual void Detected(Value* value_ptr) = 0;
    virtual void Cleared() = 0;
    virtual void TrackingUpdate(Value* value_ptr) = 0;
    virtual void PresenceUpdate(Value* value_ptr) = 0;
};

class RadarSensor {
public:
    explicit RadarSensor(EventProc* event_processor);
    virtual std::vector<std::unique_ptr<Value>> get_decoded_radar_data() = 0;
    void process(float minimum_power = 0.0f);
};
```

### Concrete sensor: `LD1125`

* Construct with an `EventProc*` and `uart_port_t`.
* Poll via `get_decoded_radar_data` or drive via `process`.
* Optional `verifyTestMode()` will attempt to enable test mode and save config.

### Debouncer: `DebounceRadar`

Wrap any `RadarSensor` to suppress bursts while ensuring the last observation in a window is emitted. Set `interval_ms` to control the window.

```c++
DebounceRadar(RadarSensor* sensor,
              EventProc* event_processor,
              uint32_t interval_ms);
```

---

## UART Setup

Configure the UART used by the LD1125 in your application startup. The component expects that UART driver install and pin mux are owned by the application.

```c++
#include "driver/uart.h"

static void init_ld1125_uart(uart_port_t uart_port,
                             int gpio_tx,
                             int gpio_rx,
                             int baud_rate) {
    uart_config_t uart_cfg;
    uart_cfg.baud_rate = baud_rate;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(uart_port, &uart_cfg);
    uart_set_pin(uart_port, gpio_tx, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_port, 2048, 0, 0, nullptr, 0);
}
```

---

## Minimal Usage Example

Create an `EventProc` implementation and wire up `LD1125` with optional `DebounceRadar`.

```c++
#include <memory>
#include "esp_log.h"
#include "LD1125.h"
#include "DebounceRadar.h"

class ConsoleEvents : public EventProc {
public:
    void Detected(Value* value_ptr) override {
        JsonWrapper json;
        json.AddItem("event", "detected");
        value_ptr->toJson(json);
        ESP_LOGI("LD1125_APP", "%s", json.c_str());
    }
    void Cleared() override {
        ESP_LOGI("LD1125_APP", "{\"event\":\"cleared\"}");
    }
    void TrackingUpdate(Value* value_ptr) override {
        JsonWrapper json;
        json.AddItem("event", "tracking");
        value_ptr->toJson(json);
        ESP_LOGI("LD1125_APP", "%s", json.c_str());
    }
    void PresenceUpdate(Value* value_ptr) override {
        JsonWrapper json;
        json.AddItem("event", "presence");
        value_ptr->toJson(json);
        ESP_LOGI("LD1125_APP", "%s", json.c_str());
    }
};

extern "C" void app_main() {
    static ConsoleEvents handlers;

    const uart_port_t uart_port = UART_NUM_1;
    init_ld1125_uart(uart_port, /*tx*/GPIO_NUM_17, /*rx*/GPIO_NUM_16, /*baud*/115200);

    // Raw sensor
    auto sensor = std::make_unique<LD1125>(&handlers, uart_port);

    // Optional: ensure module is in test mode
    // sensor->verifyTestMode();

    // Debounce wrapper with 250 ms window
    DebounceRadar debounced(sensor.get(), &handlers, 250);

    // Simple polling loop
    while (true) {
        debounced.process(/*minimum_power*/ 0.0f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

Notes:

* `process` encapsulates the state machine and emits events based on parsed frames and `CONFIG_LD1125_DETECTION_TIMEOUT_MS`.
* Presence and tracking throttling should be enforced by your `EventProc` if you want per-channel rate control using `CONFIG_LD1125_PRESENCE_INTERVAL_MS` and `CONFIG_LD1125_TRACKING_INTERVAL_MS` or by placing a rate limiter in your implementation.

---

## Debouncing Behavior

`DebounceRadar` implements a trailing-edge window:

* First observation is emitted immediately and starts a window `[t0, t0+interval]`.
* During the window, new observations are suppressed and stored as pending.
* When the window closes, the most recent pending observation is emitted.
* If a new observation arrives after the window and differs beyond tolerance, it is emitted immediately and a new window begins.

This keeps logs quiet while still reflecting the latest state at the chosen cadence.

---

## JSON Output

Each `Value` can render itself into `JsonWrapper`. Example messages:

```json
{"event":"detected","type":"mov","distance":0.21,"power":31.8}
{"event":"tracking","type":"occ","distance":1.85,"power":9.5}
{"event":"cleared"}
```

Use these in application logs, MQTT, or WebSocket streams.

---

## Performance Guidelines

* UART RX buffer: at least 2 KB for sustained bursts
* Parser loop: keep `uart_read_bytes` timeout short to avoid blocking other tasks
* Call `process` at 10 to 20 ms cadence for responsive updates
* Avoid heavy work inside `EventProc`; queue to another task if needed

---

## Testing Tips

* Feed prerecorded LD1125 lines into the UART via a loopback or the `uart_write_bytes` API on a second port
* Verify state transitions by asserting the sequence: `Detected` then periodic `TrackingUpdate`/`PresenceUpdate`, followed by `Cleared` after timeout
* Adjust `CONFIG_LD1125_EQUAL_*` tolerances when sensor jitter is causing unnecessary emissions

---

## FAQ

**Why am I still seeing many presence lines?**

* Ensure `DebounceRadar` wraps your sensor with a sensible interval, for example 250 to 500 ms.
* Raise `CONFIG_LD1125_EQUAL_POWER_EPS_MILLI` to treat small power changes as equal.

**Do I have to use `verifyTestMode`?**

* No. It is provided to quickly set the module to test mode during bring-up. Production may avoid it.

**Who owns the UART driver?**

* Your app does. The component assumes UART has been configured and installed before use.
