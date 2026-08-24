/*
 * foot_pressure_sensor_device.h — 足底压力传感器设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <string>

#include "framework/device.h"
#include "foot_pressure/FootPressureSensor.h"

namespace stark {

class FootPressureDevice : public ISensorDevice {
public:
    static Device* create() { return new FootPressureDevice(); }

    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "foot_pressure"; }

    bool initialize(const nlohmann::json& config) override;
    bool start() override { return m_foot.IsReady(); }
    void stop() override;
    bool read(SensorData& out) override;

private:
    std::string                                    m_name;
    stark_periph_manager_node::FootPressureSensor  m_foot;
    foot_pressure_data_t                           m_data;
};

}  // namespace stark
