/*
 * imu_sensor_device.h — ICM45608 IMU 传感器设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <string>

#include "framework/device.h"
#include "sensor/imu/imu_sensor.h"

namespace stark {

class ImuIcm45608 : public Device {
public:
    static Device* create() { return new ImuIcm45608(); }

    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "imu"; }

    bool initialize(const nlohmann::json& config) override;
    bool start() override { return m_imu.IsReady(); }
    void stop() override;

    /* 获取底层 IMU 源 (RT 线程直接用, 不经过虚接口, 零开销) */
    stark_periph_manager_node::IImuSource* source() { return &m_imu; }

private:
    std::string                              m_name;
    stark_periph_manager_node::ImuHALSensor  m_imu;
};

}  // namespace stark
