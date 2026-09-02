/*
 * imu_bhi360_device.h — BHI360 IMU 传感器设备 (L0)
 *
 * 和 ImuIcm45608 平级，通过 config.json driver="bhi360" 切换。
 * 不改动现有 ICM45608 代码，框架零改动。
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <string>

#include "framework/device.h"
#include "sensor/imu/bhi360_imu_sensor.h"

namespace stark {

class ImuBhi360 : public Device {
public:
    static Device* create() { return new ImuBhi360(); }

    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "imu"; }

    bool initialize(const nlohmann::json& config) override;
    bool start() override { return m_imu.IsReady(); }
    void stop() override;

    /* 获取底层 IMU 源 (RT 线程直接用, 不经过虚接口, 零开销) */
    stark_periph_manager_node::IImuSource* source() { return &m_imu; }

private:
    std::string                                    m_name;
    stark_periph_manager_node::Bhi360IioSensor      m_imu;
};

}  // namespace stark
