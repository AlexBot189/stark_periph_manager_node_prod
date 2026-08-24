/*
 * imu_sensor_device.h — ICM45608 IMU 传感器设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <string>

#include "framework/device.h"
#include "imu/imu_sensor.h"

namespace stark {

class ImuIcm45608 : public ISensorDevice {
public:
    static Device* create() { return new ImuIcm45608(); }

    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "imu"; }

    bool initialize(const nlohmann::json& config) override;
    bool start() override { return m_imu.IsReady(); }
    void stop() override;
    bool read(SensorData& out) override;

private:
    std::string                              m_name;
    stark_periph_manager_node::ImuHALSensor  m_imu;
    imu_data_t                               m_data;
};

}  // namespace stark
