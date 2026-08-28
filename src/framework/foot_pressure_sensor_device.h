/*
 * foot_pressure_sensor_device.h — 足底压力传感器设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <string>

#include "framework/device.h"
#include "sensor/foot_pressure/FootPressureSensor.h"

namespace stark {

class FootPressureDevice : public Device {
public:
    static Device* create() { return new FootPressureDevice(); }

    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "foot_pressure"; }

    bool initialize(const nlohmann::json& config) override;
    bool start() override { return m_foot.IsReady(); }
    void stop() override;

    /* 获取底层足压传感器 (RT 线程直接用, 不经过虚接口, 零开销) */
    stark_periph_manager_node::FootPressureSensor* sensor() { return &m_foot; }

private:
    std::string                                    m_name;
    stark_periph_manager_node::FootPressureSensor  m_foot;
};

}  // namespace stark
