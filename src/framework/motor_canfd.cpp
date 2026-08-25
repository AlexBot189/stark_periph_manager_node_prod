/*
 * motor_canfd.cpp — 巨蟹 CANFD 电机设备实现 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/motor_canfd.h"
#include "framework/device_manager.h"

#include <cstdio>

namespace stark {

/* 静态注册: driver 名 "motor_canfd" */
REGISTER_DEVICE(motor_canfd, MotorCanfd::create);

MotorCanfd::~MotorCanfd()
{
    stop();
}

bool MotorCanfd::initialize(const nlohmann::json& config)
{
    m_name = config.value("name", "motor_canfd");

    const std::string can_iface = config.value("can_iface", "can0");
    const int  arb_rate  = config.value("arb_rate",  1000000);
    const int  data_rate = config.value("data_rate", 5000000);

    m_hal = motor_hal_create();
    if (!m_hal) {
        fprintf(stderr, "[MotorCanfd] motor_hal_create() failed\n");
        return false;
    }

    if (motor_hal_init(m_hal, can_iface.c_str(), arb_rate, data_rate) < 0) {
        fprintf(stderr, "[MotorCanfd] motor_hal_init(%s) failed\n", can_iface.c_str());
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }

    /* 解析 motors 列表, 数量可配 */
    if (config.contains("motors") && config["motors"].is_array()) {
        for (const auto& m : config["motors"]) {
            motor_config_t mc = {};
            mc.node_id           = m.value("id", 0);
            mc.heartbeat_ms      = m.value("heartbeat_ms", 2000u);
            mc.profile_accel     = m.value("profile_accel", 5000u);
            mc.profile_decel     = m.value("profile_decel", 5000u);
            mc.profile_velocity  = m.value("profile_velocity", 20u);
            mc.disable_watchdog  = m.value("disable_watchdog", true);
            mc.auto_enable       = m.value("auto_enable", false);
            mc.bootup_timeout_ms = m.value("bootup_timeout_ms", 5000);
            mc.tpdo_sync_count   = m.value("tpdo_sync_count", (uint8_t)1);

            if (mc.node_id == 0) {
                fprintf(stderr, "[MotorCanfd] motor id=0 invalid, skip\n");
                continue;
            }

            if (motor_hal_add_motor(m_hal, &mc) < 0) {
                fprintf(stderr, "[MotorCanfd] add motor id=%u failed\n", mc.node_id);
                continue;
            }
            m_motor_count++;
        }
    }

    if (m_motor_count == 0) {
        fprintf(stderr, "[MotorCanfd] no motor configured\n");
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }

    fprintf(stderr, "[MotorCanfd] initialized: %s %d motor(s)\n",
            m_name.c_str(), (int)m_motor_count);
    return true;
}

bool MotorCanfd::start()
{
    if (!m_hal) return false;
    return motor_hal_recv_start(m_hal) == 0;
}

void MotorCanfd::stop()
{
    if (!m_hal) return;
    motor_hal_recv_stop(m_hal);
    motor_hal_destroy(m_hal);
    m_hal = nullptr;
    m_motor_count = 0;
}

}  // namespace stark
