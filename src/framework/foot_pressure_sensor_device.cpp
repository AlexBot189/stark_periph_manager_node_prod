/*
 * foot_pressure_sensor_device.cpp — 足底压力传感器设备实现 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/foot_pressure_sensor_device.h"
#include "framework/device_manager.h"

namespace stark {

REGISTER_DEVICE(foot_pressure, FootPressureDevice::create);

bool FootPressureDevice::initialize(const nlohmann::json& config)
{
    m_name = config.value("name", "foot_pressure");

    const std::string uart    = config.value("uart_dev", std::string("/dev/ttyS7"));
    const int         baud    = config.value("baud_rate", 460800);
    const int         timeout = config.value("timeout_ms", 10);

    return m_foot.Init(uart.c_str(), baud, timeout);
}

void FootPressureDevice::stop()
{
    m_foot.Deinit();
}

}  // namespace stark
