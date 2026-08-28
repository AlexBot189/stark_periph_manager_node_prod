/*
 * barometer_device.h — 气压计传感器设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <string>

#include "framework/device.h"
#include "barometer/barometer_sensor.h"

namespace stark {

class BarometerDevice : public Device {
public:
    static Device* create() { return new BarometerDevice(); }

    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "barometer"; }

    bool initialize(const nlohmann::json& config) override;
    bool start() override { return m_baro.IsReady(); }
    void stop() override;

    /* 获取底层气压计源 (RT 线程直接用, 不经过虚接口, 零开销) */
    stark_periph_manager_node::IBarometerSource* source() { return &m_baro; }

private:
    std::string                                  m_name;
    stark_periph_manager_node::BarometerSensor   m_baro;
};

}  // namespace stark
