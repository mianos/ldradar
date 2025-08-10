#pragma once
#include "esp_log.h"
#include "JsonWrapper.h"

struct Value {
    virtual const std::string etype() const { return "und"; }
    virtual void print() const { ESP_LOGE("Events::Value", "un-overridden '%s'", etype().c_str()); }
    virtual std::unique_ptr<Value> clone() const = 0;
    virtual bool isEqual(const Value& other) const = 0;
    virtual float get_main() const { return 0.0f; }
    virtual float get_power() const { return 0.0f; }

    virtual JsonWrapper& toJson(JsonWrapper& doc) const {
        doc.AddItem("type", etype());
        return doc;
    }
};

struct Range : public Value {
    float x = 0.0f;
    float y = 0.0f;
    float speed = 0.0f;
    int reference = 0;

    const std::string etype() const override { return "rng"; }

    Range(float x_in, float y_in, float speed_in, int reference_in = 0)
        : x(x_in), y(y_in), speed(speed_in), reference(reference_in) {}

    float get_main() const override { return speed; }

    void print() const override {
        ESP_LOGI("Events", "Range: speed %1.2f x pos %1.2f Y pos %1.2f %2d", speed, x, y, reference);
    }

    std::unique_ptr<Value> clone() const override {
        return std::unique_ptr<Value>(new Range(*this));
    }

    bool isEqual(const Value& other) const override {
        const Range& casted = static_cast<const Range&>(other);
        return x == casted.x && y == casted.y && speed == casted.speed && reference == casted.reference;
    }

    JsonWrapper& toJson(JsonWrapper& doc) const override {
        Value::toJson(doc);
        doc.AddItem("x", x);
        doc.AddItem("y", y);
        doc.AddItem("speed", speed);
        doc.AddItem("reference", reference);
        return doc;
    }
};

struct NoTarget : public Value {
    const std::string etype() const override { return "no"; }

    void print() const override { ESP_LOGI("Events", "no target"); }

    std::unique_ptr<Value> clone() const override { return std::unique_ptr<Value>(new NoTarget(*this)); }

    bool isEqual(const Value& other) const override { (void)other; return true; }
};

struct Movement : public Value {
    float distance = 0.0f;
    float power = 0.0f;

    const std::string etype() const override { return "mov"; }

    Movement(float distance_in, float power_in = 0.0f)
        : distance(distance_in), power(power_in) {}

    float get_main() const override { return distance; }
    float get_power() const override { return power; }

    void print() const override {
        ESP_LOGI("Events", "Movement: distance %.1f m, power %.1f units", distance, power);
    }

    std::unique_ptr<Value> clone() const override { return std::make_unique<Movement>(*this); }

    bool isEqual(const Value& other) const override {
        const auto& casted = static_cast<const Movement&>(other);
        return distance == casted.distance && power == casted.power;
    }

    JsonWrapper& toJson(JsonWrapper& doc) const override {
        Value::toJson(doc);
        doc.AddItem("distance", distance);
        doc.AddItem("power", power);
        return doc;
    }
};

struct Occupancy : public Value {
    float distance = 0.0f;
    float power = 0.0f;

    const std::string etype() const override { return "occ"; }

    Occupancy(float distance_in, float power_in = 0.0f)
        : distance(distance_in), power(power_in) {}

    float get_main() const override { return distance; }
    float get_power() const override { return power; }

    void print() const override {
        ESP_LOGI("Events", "Occupancy: distance %.1f m, power %.1f units", distance, power);
    }

    std::unique_ptr<Value> clone() const override { return std::make_unique<Occupancy>(*this); }

    bool isEqual(const Value& other) const override {
        const auto& casted = static_cast<const Occupancy&>(other);
        return distance == casted.distance && power == casted.power;
    }

    JsonWrapper& toJson(JsonWrapper& doc) const override {
        Value::toJson(doc);
        doc.AddItem("distance", distance);
        doc.AddItem("power", power);
        return doc;
    }
};

