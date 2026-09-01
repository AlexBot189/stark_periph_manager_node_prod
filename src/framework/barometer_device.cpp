/*
 * barometer_device.cpp — 气压计传感器设备实现 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/barometer_device.h"
#include "framework/device_manager.h"

namespace stark {

REGISTER_DEVICE(barometer, BarometerDevice::create);

bool BarometerDevice::initialize(const nlohmann::json& config)
{
    m_name = config.value("name", "barometer");

    stark_periph_manager_node::BarometerConfig cfg;
    cfg.input_name       = config.value("input_name", std::string("bmp5xy"));
    cfg.sample_period_ms = config.value("sample_period_ms", 50u);
    cfg.osr_t            = config.value("osr_t", 6);
    cfg.osr_p            = config.value("osr_p", 2);
    cfg.odr              = config.value("odr", 15);
    cfg.power_mode       = config.value("power_mode", 1);
    cfg.sea_level_hpa    = config.value("sea_level_hpa", 1013.25f);
    cfg.iir_p            = config.value("iir_p", 3);
    cfg.iir_t            = config.value("iir_t", 3);

    return m_baro.Init(cfg);
}

void BarometerDevice::stop()
{
    m_baro.Deinit();
}

}  // namespace stark
