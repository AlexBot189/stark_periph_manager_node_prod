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
            m_node_ids.push_back(mc.node_id);
        }
    }

    if (m_node_ids.empty()) {
        fprintf(stderr, "[MotorCanfd] no motor configured\n");
        motor_hal_destroy(m_hal);
        m_hal = nullptr;
        return false;
    }

    fprintf(stderr, "[MotorCanfd] initialized: %s %d motor(s)\n",
            m_name.c_str(), (int)m_node_ids.size());
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
    m_node_ids.clear();
}

bool MotorCanfd::readFeedback(int index, MotorFeedback& fb)
{
    if (!m_hal || !validIndex(index)) return false;

    motor_feedback_t mfb;
    if (motor_hal_get_feedback(m_hal, m_node_ids[index], &mfb) != 0) {
        return false;
    }

    fb.position     = mfb.position;
    fb.velocity     = mfb.velocity;
    fb.current      = mfb.current_iq;
    fb.torque       = mfb.torque_nm;
    fb.temperature  = mfb.temperature;
    fb.error_code   = mfb.error_code;
    fb.mode         = mfb.mode;
    fb.status       = mfb.status_byte;
    fb.timestamp_us = mfb.timestamp_us;
    return true;
}

bool MotorCanfd::writeCommand(int index, const MotorCommand& cmd)
{
    if (!m_hal || !validIndex(index)) return false;

    multi_axis_cmd_t mc = {};
    mc.node_id       = m_node_ids[index];
    mc.mode          = (motor_mode_t)cmd.mode;
    mc.enable        = (cmd.flags & MOTOR_FLAG_ENABLE) != 0;
    mc.release_brake = (cmd.flags & MOTOR_FLAG_RELEASE_BRAKE) != 0;
    mc.clear_error   = (cmd.flags & MOTOR_FLAG_CLEAR_ERROR) != 0;
    mc.target1       = (int16_t)cmd.target1;
    mc.target2       = (uint16_t)cmd.target2;
    mc.feedforward   = (int16_t)cmd.feedforward;

    motor_hal_multi_ctrl(m_hal, &mc, 1);
    return true;
}

bool MotorCanfd::writeMitCommand(int index, float position, float velocity,
                                 float kp, float kd, float torque)
{
    if (!m_hal || !validIndex(index)) return false;
    return motor_hal_mit_control(m_hal, m_node_ids[index],
                                 position, velocity, kp, kd, torque) == 0;
}

bool MotorCanfd::enable(int index)
{
    if (!m_hal || !validIndex(index)) return false;
    return motor_hal_enable(m_hal, m_node_ids[index]) == 0;
}

bool MotorCanfd::disable(int index)
{
    if (!m_hal || !validIndex(index)) return false;
    return motor_hal_disable(m_hal, m_node_ids[index]) == 0;
}

bool MotorCanfd::clearFault(int index)
{
    if (!m_hal || !validIndex(index)) return false;
    return motor_hal_fault_reset(m_hal, m_node_ids[index]) == 0;
}

}  // namespace stark
