/*
 * imu_sensor_device.cpp — ICM45608 IMU 传感器设备实现 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/imu_sensor_device.h"
#include "framework/device_manager.h"

namespace stark {

REGISTER_DEVICE(imu_icm45608, ImuIcm45608::create);

bool ImuIcm45608::initialize(const nlohmann::json& config)
{
    m_name = config.value("name", "imu");

    stark_periph_manager_node::ImuConfig cfg;
    cfg.driver       = config.value("driver", std::string("invensense"));
    cfg.interface    = config.value("interface", std::string("i2c"));
    cfg.i2c_dev      = config.value("i2c_dev", std::string("/dev/i2c-3"));
    cfg.spi_dev      = config.value("spi_dev", std::string("/dev/spidev0.0"));
    cfg.spi_speed_hz = config.value("spi_speed_hz", 8000000u);
    cfg.spi_mode     = config.value("spi_mode", 0u);
    cfg.gpio_chip    = config.value("gpio_chip", std::string("gpiochip4"));
    cfg.gpio_line    = config.value("gpio_line", 6u);
    cfg.op_mode      = config.value("op_mode", 5);

    return m_imu.Init(cfg);
}

void ImuIcm45608::stop()
{
    m_imu.Deinit();
}

bool ImuIcm45608::read(SensorData& out)
{
    m_imu.Read(&m_data);

    out.type         = SENSOR_IMU;
    out.size         = sizeof(imu_data_t);
    out.data         = &m_data;
    out.timestamp_us = m_data.timestamp_us;
    return m_imu.IsReady();
}

}  // namespace stark
