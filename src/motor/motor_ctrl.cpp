/*
 * stark_motor_ctrl.cpp — SDO/PDO/OD 控制封装实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 所有 SDO 控制路径与 motor_tool 对齐.
 * PDO 路径直接透传 motor_hal 非阻塞 API.
 */
#include "motor/motor_ctrl.h"
#include <log_helper/LogHelper.h>
#include <cstring>
#include <cmath>
#include <cerrno>

namespace stark_periph_manager_node {

StarkMotorCtrl::StarkMotorCtrl(motor_hal_t* hal)
    : m_hal(hal)
    , m_abs_accel(500)
    , m_abs_speed(10)
{
    memset(m_mode_cache, 0xFF, sizeof(m_mode_cache));
}

/*
 * 内部: 幂等 set_mode — 缓存模式, 跳过重复 SDO 写.
 */

int StarkMotorCtrl::_set_mode_cached(uint8_t id, motor_mode_t mode)
{
    if (id >= sizeof(m_mode_cache)) return -EINVAL;

    uint8_t target = (uint8_t)mode;
    if (m_mode_cache[id] == target) {
        return 0;  /* 幂等: 模式未变 */
    }

    int ret = motor_hal_set_mode(m_hal, id, mode);
    if (ret == 0) {
        m_mode_cache[id] = target;
    }
    return ret;
}

/*
 * 内部: Halt 当前运动 — 写 CW 0x010F + 轮询 statusword bit10
 */

void StarkMotorCtrl::_halt_motion(uint8_t id)
{
    motor_state_t st = motor_hal_get_state(m_hal, id);
    if (st != MOTOR_STATE_OP_ENABLED) return;

    motor_hal_sdo_write(m_hal, id, 0x6040, 0, 0x010F, 2);

    for (int i = 0; i < 25; i++) {
        usleep(20000);
        uint16_t sw = motor_hal_get_statusword(m_hal, id);
        if (sw & 0x0400) return;
    }
}

/* 系统命令 */

int StarkMotorCtrl::Startup(uint8_t id, uint32_t timeout_ms)
{
    ECO_INFO_NEW("[StarkMotorCtrl] startup motor {}", id);

    /* 尝试注册 (已注册则 EEXIST 忽略) */
    motor_config_t cfg = {};
    cfg.node_id           = id;
    cfg.heartbeat_ms      = 2000;
    cfg.profile_accel     = 5000;
    cfg.profile_decel     = 5000;
    cfg.profile_velocity  = 20;
    cfg.disable_watchdog  = true;
    cfg.auto_enable       = true;
    cfg.bootup_timeout_ms = timeout_ms;
    cfg.tpdo_sync_count   = 1;

    int ret = motor_hal_add_motor(m_hal, &cfg);
    if (ret != 0 && ret != -EEXIST) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} add failed: {}", id, ret);
        return ret;
    }

    ret = motor_hal_startup(m_hal, id, (int)timeout_ms);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} startup failed: {}", id, ret);
        return ret;
    }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {} ,  OPERATION_ENABLED", id);
    return 0;
}

int StarkMotorCtrl::Enable(uint8_t id)
{
    int ret = motor_hal_enable(m_hal, id);
    if (ret == 0) {
        ECO_INFO_NEW("[StarkMotorCtrl] motor {} enabled", id);
    } else {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} enable failed: {}", id, ret);
    }
    return ret;
}

int StarkMotorCtrl::Disable(uint8_t id)
{
    int ret = motor_hal_disable(m_hal, id);
    if (ret == 0) {
        ECO_INFO_NEW("[StarkMotorCtrl] motor {} disabled", id);
    } else {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} disable failed: {}", id, ret);
    }
    return ret;
}

int StarkMotorCtrl::FaultReset(uint8_t id)
{
    int ret = motor_hal_fault_reset(m_hal, id);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} fault_reset: {}", id, (ret == 0 ? "OK" : "FAIL"));
    return ret;
}

int StarkMotorCtrl::Reboot(uint8_t id)
{
    int ret = motor_hal_nmt_send(m_hal, id, NMT_CMD_RESET_NODE);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} reboot sent: {}", id, ret);
    return ret;
}

void StarkMotorCtrl::NmtStopAll()
{
    motor_hal_nmt_broadcast(m_hal, NMT_CMD_STOP);
    ECO_INFO_NEW("[StarkMotorCtrl] NMT broadcast STOP");
}

/* SDO 控制命令 (完整时序, 对齐 tool_hal.c) */

int StarkMotorCtrl::Torque(uint8_t id, int32_t ma)
{
    if (ma < -20000 || ma > 20000) {
        ECO_ERROR_NEW("[StarkMotorCtrl] torque {} mA out of range (-20000~20000)", ma);
        return -EINVAL;
    }

    int ret;

    /* 使能 (幂等: 检查状态, 避免每次调用重新走 shutdown→switchOn→enableOp) */
    motor_state_t st = motor_hal_get_state(m_hal, id);
    if (st != MOTOR_STATE_OP_ENABLED) {
        ret = motor_hal_enable(m_hal, id);
        if (ret != 0) {
            ECO_ERROR_NEW("[StarkMotorCtrl] motor {} enable failed: {}", id, ret);
            return ret;
        }
        /* enable 后重置模式缓存 (状态机重新走了) */
        memset(m_mode_cache, 0xFF, sizeof(m_mode_cache));
    }

    /* 切电流模式 */
    ret = _set_mode_cached(id, MOTOR_MODE_CURRENT);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} set_mode(CUR) failed: {}", id, ret);
        return ret;
    }

    /* 写目标电流 0x6071 (2 字节, inst=0x2B) */
    ret = motor_hal_sdo_write(m_hal, id, 0x6071, 0, (uint32_t)ma, 2);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} sdo_write 0x6071 failed: {}", id, ret);
        return ret;
    }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {}: torque={}mA (current mode)", id, ma);
    return 0;
}

int StarkMotorCtrl::Speed(uint8_t id, int32_t rpm,
                         int32_t acc, int32_t dec)
{
    uint16_t accel = (uint16_t)acc;
    uint16_t decel = (uint16_t)dec;

    if (accel > 10000 || decel > 10000) {
        ECO_ERROR_NEW("[StarkMotorCtrl] accel/decel out of range: {}/{}", accel, decel);
        return -EINVAL;
    }

    int ret;

    /* 使能 (幂等) */
    motor_state_t st = motor_hal_get_state(m_hal, id);
    if (st != MOTOR_STATE_OP_ENABLED) {
        ret = motor_hal_enable(m_hal, id);
        if (ret != 0) {
            ECO_ERROR_NEW("[StarkMotorCtrl] motor {} enable failed: {}", id, ret);
            return ret;
        }
        memset(m_mode_cache, 0xFF, sizeof(m_mode_cache));
    }

    /* Halt 当前运动 */
    _halt_motion(id);

    /* 切 PV 模式 */
    ret = _set_mode_cached(id, MOTOR_MODE_PROFILE_VEL);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} set_mode(PV) failed: {}", id, ret);
        return ret;
    }

    /* 加减速 0x6083/0x6084 */
    ret = motor_hal_set_accel_decel(m_hal, id, accel, decel);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} set_accel_decel failed: {}", id, ret);
        return ret;
    }

    /* Profile velocity 0x6081 */
    motor_hal_set_profile_velocity(m_hal, id, m_abs_speed);

    /* 目标速度 0x60FF (4 字节) */
    ret = motor_hal_sdo_write(m_hal, id, 0x60FF, 0, (uint32_t)rpm, 4);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} write 0x60FF failed: {}", id, ret);
        return ret;
    }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {}: speed={}RPM accel={} decel={} (PV mode)",
                 id, rpm, accel, decel);
    return 0;
}

int StarkMotorCtrl::SpeedEx(uint8_t id, int32_t rpm, int32_t accel, int32_t decel)
{
    uint16_t a = (accel > 0) ? (uint16_t)accel : m_abs_accel;
    uint16_t d = (decel > 0) ? (uint16_t)decel : m_abs_accel;

    if (a > 10000 || d > 10000) {
        ECO_ERROR_NEW("[StarkMotorCtrl] accel/decel out of range: {}/{}", a, d);
        return -EINVAL;
    }

    motor_state_t st = motor_hal_get_state(m_hal, id);
    if (st != MOTOR_STATE_OP_ENABLED) {
        int ret = motor_hal_enable(m_hal, id);
        if (ret != 0) { ECO_ERROR_NEW("[StarkMotorCtrl] motor {} enable failed: {}", id, ret); return ret; }
        memset(m_mode_cache, 0xFF, sizeof(m_mode_cache));
    }

    _halt_motion(id);

    _set_mode_cached(id, MOTOR_MODE_PROFILE_VEL);

    motor_hal_set_accel_decel(m_hal, id, a, d);
    motor_hal_set_profile_velocity(m_hal, id, m_abs_speed);

    int ret = motor_hal_sdo_write(m_hal, id, 0x60FF, 0, (uint32_t)rpm, 4);
    if (ret != 0) { ECO_ERROR_NEW("[StarkMotorCtrl] motor {} write 0x60FF failed: {}", id, ret); return ret; }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {}: speed={}RPM accel={} decel={} (PV mode)",
                 id, rpm, a, d);
    return 0;
}

int StarkMotorCtrl::AbsPosition(uint8_t id, float deg)
{
    int32_t counts = motor_deg_to_counts(deg);

    if (counts < -32767 || counts > 32768) {
        ECO_ERROR_NEW("[StarkMotorCtrl] position {} counts out of range", counts);
        return -EINVAL;
    }

    int ret;

    /* 使能 (幂等) */
    motor_state_t st = motor_hal_get_state(m_hal, id);
    if (st != MOTOR_STATE_OP_ENABLED) {
        ret = motor_hal_enable(m_hal, id);
        if (ret != 0) {
            ECO_ERROR_NEW("[StarkMotorCtrl] motor {} enable failed: {}", id, ret);
            return ret;
        }
        memset(m_mode_cache, 0xFF, sizeof(m_mode_cache));
    }

    /* Halt 当前运动 */
    _halt_motion(id);

    /* 切 PP 模式 */
    ret = _set_mode_cached(id, MOTOR_MODE_PROFILE_POS);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} set_mode(PP) failed: {}", id, ret);
        return ret;
    }

    /* 加减速 */
    motor_hal_set_accel_decel(m_hal, id, m_abs_accel, m_abs_accel);

    /* 轨迹速度 */
    motor_hal_set_profile_velocity(m_hal, id, m_abs_speed);

    /* 目标位置 0x607A */
    ret = motor_hal_sdo_write(m_hal, id, 0x607A, 0, (uint32_t)counts, 4);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} write 0x607A failed: {}", id, ret);
        return ret;
    }

    /* 启动绝对运动 CW=0x4F */
    ret = motor_hal_sdo_write(m_hal, id, 0x6040, 0, 0x004F, 2);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} start motion (0x4F) failed: {}", id, ret);
        return ret;
    }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {}: abs {:.2f}deg (counts={}, accel={}, vel={})",
                 id, deg, counts, m_abs_accel, m_abs_speed);
    return 0;
}

int StarkMotorCtrl::AbsPositionEx(uint8_t id, float deg, uint16_t accel, uint16_t vel)
{
    int32_t counts = motor_deg_to_counts(deg);
    if (counts < -32767 || counts > 32768) {
        ECO_ERROR_NEW("[StarkMotorCtrl] position {} counts out of range", counts);
        return -EINVAL;
    }

    motor_state_t st = motor_hal_get_state(m_hal, id);
    if (st != MOTOR_STATE_OP_ENABLED) {
        int ret = motor_hal_enable(m_hal, id);
        if (ret != 0) { ECO_ERROR_NEW("[StarkMotorCtrl] motor {} enable failed: {}", id, ret); return ret; }
        memset(m_mode_cache, 0xFF, sizeof(m_mode_cache));
    }

    _halt_motion(id);

    int ret = _set_mode_cached(id, MOTOR_MODE_PROFILE_POS);
    if (ret != 0) { ECO_ERROR_NEW("[StarkMotorCtrl] motor {} set_mode(PP) failed: {}", id, ret); return ret; }

    uint16_t a = (accel > 0) ? accel : m_abs_accel;
    uint16_t v = (vel > 0)   ? vel   : m_abs_speed;

    motor_hal_set_accel_decel(m_hal, id, a, a);
    motor_hal_set_profile_velocity(m_hal, id, v);

    ret = motor_hal_sdo_write(m_hal, id, 0x607A, 0, (uint32_t)counts, 4);
    if (ret != 0) { ECO_ERROR_NEW("[StarkMotorCtrl] motor {} write 0x607A failed: {}", id, ret); return ret; }

    ret = motor_hal_sdo_write(m_hal, id, 0x6040, 0, 0x004F, 2);
    if (ret != 0) { ECO_ERROR_NEW("[StarkMotorCtrl] motor {} start motion failed: {}", id, ret); return ret; }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {}: abs {:.2f}deg accel={} vel={}", id, deg, a, v);
    return 0;
}

void StarkMotorCtrl::AbsStop(uint8_t id)
{
    motor_hal_sdo_write(m_hal, id, 0x6040, 0, 0x000F, 2);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} motion stopped", id);
}

int StarkMotorCtrl::SetZero(uint8_t id)
{
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} zero calibration...", id);

    /* 失能 */
    int ret = motor_hal_disable(m_hal, id);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} disable failed: {}", id, ret);
        return ret;
    }

    /* 写零位 0x2531=1 */
    ret = motor_hal_set_zero(m_hal, id);
    if (ret != 0) {
        ECO_ERROR_NEW("[StarkMotorCtrl] motor {} set_zero failed: {}", id, ret);
        return ret;
    }

    ECO_INFO_NEW("[StarkMotorCtrl] motor {} zero calibrated (re-enable manually)", id);
    return 0;
}

int StarkMotorCtrl::SetPosLimit(uint8_t id, float deg)
{
    int32_t counts = motor_deg_to_counts(deg);

    ECO_INFO_NEW("[StarkMotorCtrl] motor {} pos limit = {:.2f}° ({} counts)", id, deg, counts);

    int ret = motor_hal_disable(m_hal, id);
    if (ret != 0) return ret;

    ret = motor_hal_sdo_write(m_hal, id, 0x607D, 0x02, (uint32_t)counts, 4);
    if (ret != 0) return ret;

    ret = motor_hal_save_flash(m_hal, id);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} pos limit saved to Flash", id);
    return ret;
}

int StarkMotorCtrl::SetNegLimit(uint8_t id, float deg)
{
    int32_t counts = motor_deg_to_counts(deg);

    ECO_INFO_NEW("[StarkMotorCtrl] motor {} neg limit = {:.2f}° ({} counts)", id, deg, counts);

    int ret = motor_hal_disable(m_hal, id);
    if (ret != 0) return ret;

    ret = motor_hal_sdo_write(m_hal, id, 0x607D, 0x01, (uint32_t)counts, 4);
    if (ret != 0) return ret;

    ret = motor_hal_save_flash(m_hal, id);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} neg limit saved to Flash", id);
    return ret;
}

int StarkMotorCtrl::ReadPosLimit(uint8_t id, float* out_deg)
{
    uint32_t val = 0;
    int ret = motor_hal_sdo_read_u32(m_hal, id, 0x607D, 0x02, &val);
    if (ret == 0 && out_deg) {
        *out_deg = (float)(int32_t)val * 360.0f / 65536.0f;
    }
    return ret;
}

int StarkMotorCtrl::ReadNegLimit(uint8_t id, float* out_deg)
{
    uint32_t val = 0;
    int ret = motor_hal_sdo_read_u32(m_hal, id, 0x607D, 0x01, &val);
    if (ret == 0 && out_deg) {
        *out_deg = (float)(int32_t)val * 360.0f / 65536.0f;
    }
    return ret;
}

int StarkMotorCtrl::SaveFlash(uint8_t id)
{
    int ret = motor_hal_save_flash(m_hal, id);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} save flash: {}", id, (ret == 0 ? "OK" : "FAIL"));
    return ret;
}

int StarkMotorCtrl::SetPid(uint8_t id, uint16_t cp, uint16_t ci,
                          uint16_t vp, uint16_t vi, uint16_t pp, uint16_t pi)
{
    motor_pid_t pid = { cp, ci, vp, vi, pp, pi };
    int ret = motor_hal_set_pid(m_hal, id, &pid);
    ECO_INFO_NEW("[StarkMotorCtrl] motor {} PID set: cp={} ci={} vp={} vi={} pp={} pi={} ,  {}",
                 id, cp, ci, vp, vi, pp, pi, (ret == 0 ? "OK" : "FAIL"));
    return ret;
}

int StarkMotorCtrl::ReadPid(uint8_t id, motor_pid_t* out_pid)
{
    return motor_hal_read_pid(m_hal, id, out_pid);
}

/* 通用 SDO */

int StarkMotorCtrl::SdoRead(uint8_t id, uint16_t index, uint8_t subidx, uint32_t* out_val)
{
    return motor_hal_sdo_read_u32(m_hal, id, index, subidx, out_val);
}

int StarkMotorCtrl::SdoWrite(uint8_t id, uint16_t index, uint8_t subidx,
                            uint32_t value, uint8_t size)
{
    return motor_hal_sdo_write(m_hal, id, index, subidx, value, size);
}

/* 读取命令 (SDO 路径, 非 RT) */

float StarkMotorCtrl::ReadAngle(uint8_t id)
{
    int32_t pos = motor_hal_get_position(m_hal, id);
    return motor_counts_to_deg((int16_t)pos);
}

int32_t StarkMotorCtrl::ReadSpeed(uint8_t id)
{
    return motor_hal_get_velocity(m_hal, id);
}

int32_t StarkMotorCtrl::ReadCurrent(uint8_t id)
{
    return motor_hal_get_current(m_hal, id);
}

float StarkMotorCtrl::ReadTemp(uint8_t id)
{
    int32_t raw = 0;
    motor_hal_get_motor_temp(m_hal, id, &raw);
    return motor_temp_to_c((int16_t)raw);
}

int StarkMotorCtrl::ReadState(uint8_t id)
{
    return (int)motor_hal_get_state(m_hal, id);
}

int StarkMotorCtrl::ReadError(uint8_t id, uint16_t* out_err)
{
    return motor_hal_get_fault_code(m_hal, id, out_err);
}

int StarkMotorCtrl::ReadVersion(uint8_t id, uint32_t* out_ver)
{
    return motor_hal_sdo_read_u32(m_hal, id, 0x100A, 0x00, out_ver);
}

int StarkMotorCtrl::ReadMode(uint8_t id, uint8_t* out_mode)
{
    uint32_t mode = 0;
    int ret = motor_hal_sdo_read_u32(m_hal, id, 0x6061, 0x00, &mode);
    if (ret == 0 && out_mode) *out_mode = (uint8_t)mode;
    return ret;
}

/* PDO 控制 (非阻塞, RT 安全) */

void StarkMotorCtrl::PdoCtrl(uint8_t id, motor_mode_t mode, float target)
{
    switch (mode) {
    case MOTOR_MODE_CURRENT:
        motor_hal_set_torque(m_hal, id, (int16_t)target);
        break;
    case MOTOR_MODE_PROFILE_VEL:
        motor_hal_set_velocity(m_hal, id, target);
        break;
    case MOTOR_MODE_CSP:
        motor_hal_set_position(m_hal, id, target);
        break;
    default:
        break;
    }
}

void StarkMotorCtrl::PdoMultiCtrl(const multi_axis_cmd_t* cmds, uint8_t count)
{
    motor_hal_multi_ctrl(m_hal, cmds, count);
}

void StarkMotorCtrl::PdoMitCtrl(uint8_t id, float pos, float vel,
                               int16_t kp, int16_t kd, int16_t torque)
{
    motor_hal_mit_control(m_hal, id, pos, vel, kp, kd, torque);
}

/* PDO 映射 */

int StarkMotorCtrl::PdoMap(uint8_t id, pdo_type_t type,
                          const pdo_map_entry_cfg_t* entries, uint8_t count,
                          uint32_t cob_id, uint8_t trans_type)
{
    return motor_hal_pdo_map(m_hal, id, entries, count, 0,
                             type, cob_id, trans_type);
}

int StarkMotorCtrl::RpdoSend(uint8_t id, const uint8_t* data, uint8_t dlc)
{
    return motor_hal_rpdo_send(m_hal, id, data, dlc);
}

/* 传感器/反馈 */

int StarkMotorCtrl::GetFeedback(uint8_t id, motor_feedback_t* out_fb)
{
    return motor_hal_get_feedback(m_hal, id, out_fb);
}

int StarkMotorCtrl::GetSensor(uint8_t id, motor_sensor_t* out_sensor)
{
    return motor_hal_get_sensor(m_hal, id, out_sensor);
}

int StarkMotorCtrl::SensorConfig(uint8_t id, uint16_t period_div, uint8_t bus_format)
{
    return motor_hal_sensor_config(m_hal, id, period_div, bus_format);
}

int StarkMotorCtrl::SensorStop(uint8_t id)
{
    return motor_hal_sensor_stop(m_hal, id);
}

/* 辅助 */

bool StarkMotorCtrl::IsOnline(uint8_t id)
{
    motor_feedback_t fb;
    int ret = motor_hal_get_feedback(m_hal, id, &fb);
    if (ret != 0) return false;

    motor_state_t state = motor_hal_get_state(m_hal, id);
    return (state >= MOTOR_STATE_SWITCH_ON_DIS && state != MOTOR_STATE_UNKNOWN);
}

}  /* namespace stark_periph_manager_node */
