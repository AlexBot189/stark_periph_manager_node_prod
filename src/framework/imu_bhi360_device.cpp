/*
 * imu_bhi360_device.cpp — BHI360 IMU 传感器设备实现 (L0)
 *
 * REGISTER_DEVICE 注册, 和 ImuIcm45608 完全平级。
 * config.json 中 driver="bhi360" 时由 CanDispatcher 选择此设备。
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/imu_bhi360_device.h"
#include "framework/device_manager.h"

namespace stark {

REGISTER_DEVICE(imu_bhi360, ImuBhi360::create);

bool ImuBhi360::initialize(const nlohmann::json& config)
{
    m_name = config.value("name", "imu");

    stark_periph_manager_node::ImuConfig cfg;
    cfg.driver       = config.value("driver", std::string("bhi360"));
    cfg.interface    = config.value("interface", std::string("spi"));
    cfg.spi_dev      = config.value("spi_dev", std::string("/dev/spidev0.0"));
    cfg.spi_speed_hz = config.value("spi_speed_hz", 8000000u);
    cfg.spi_mode     = config.value("spi_mode", 0u);
    cfg.gpio_chip    = config.value("gpio_chip", std::string("gpiochip4"));
    cfg.gpio_line    = config.value("gpio_line", 6u);
    cfg.op_mode      = config.value("op_mode", 5);

    /* 坐标轴映射 (和 ImuIcm45608 一致的解析逻辑) */
    if (config.contains("mount")) {
        const auto& m = config["mount"];
        auto axis_of = [](const std::string& s) -> int8_t {
            if (!s.empty() && (s[0] == 'x' || s[0] == 'X')) return 0;
            if (!s.empty() && (s[0] == 'y' || s[0] == 'Y')) return 1;
            return 2;  /* Z */
        };
        if (m.contains("robot_x")) {
            cfg.mount_axis[0] = axis_of(m["robot_x"].value("from_axis", std::string("Z")));
            cfg.mount_sign[0] = m["robot_x"].value("sign", -1);
        }
        if (m.contains("robot_y")) {
            cfg.mount_axis[1] = axis_of(m["robot_y"].value("from_axis", std::string("X")));
            cfg.mount_sign[1] = m["robot_y"].value("sign", -1);
        }
        if (m.contains("robot_z")) {
            cfg.mount_axis[2] = axis_of(m["robot_z"].value("from_axis", std::string("Y")));
            cfg.mount_sign[2] = m["robot_z"].value("sign", 1);
        }
    }

    return m_imu.Init(cfg);
}

void ImuBhi360::stop()
{
    m_imu.Deinit();
}

}  // namespace stark
