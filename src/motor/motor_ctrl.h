/*
 * stark_motor_ctrl.h — SDO/PDO/OD 控制封装
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 封装 motor_tool 中验证过的所有控制路径:
 *   系统: enable / disable / fault_reset / reboot
 *   SDO:  torque / speed / abs / setzero / limit / pid / save
 *   PDO:  pdo_ctrl / multi_ctrl / mit_ctrl
 *   读取: read_angle / read_speed / read_current / read_temp 等
 */
#pragma once

#include <cstdint>
#include <string>

extern "C" {
#include "motor_hal.h"
}

namespace stark_periph_manager_node {

class StarkMotorCtrl {
public:
    explicit StarkMotorCtrl(motor_hal_t* hal);
    ~StarkMotorCtrl() = default;

    /* 系统命令 (SDO 路径, 非 RT) */

    /** @brief 手动启动电机 (Bootup ,  NMT ,  DS402 enable) */
    int  Startup(uint8_t id, uint32_t timeout_ms = 5000);

    /** @brief DS402 使能 */
    int  Enable(uint8_t id);

    /** @brief DS402 脱使能 */
    int  Disable(uint8_t id);

    /** @brief 故障复位 */
    int  FaultReset(uint8_t id);

    /** @brief NMT 重启节点 */
    int  Reboot(uint8_t id);

    /** @brief NMT 广播 Stop */
    void NmtStopAll();

    /* SDO 控制命令 (SDO 路径, 非 RT) */

    /** @brief 力矩控制 (使能, 切电流模式, 写0x6071), mA [-20000,20000] */
    int  Torque(uint8_t id, int32_t ma);

    /** @brief 速度控制 (使能, 切PV模式, 设加减速, 写0x60FF), RPM */
    int  Speed(uint8_t id, int32_t rpm, int32_t acc = 1000, int32_t dec = 1000);

    /** @brief 速度控制扩展版 (使能, halt, 切PV, accel/decel/vel可配) */
    int  SpeedEx(uint8_t id, int32_t rpm, int32_t accel, int32_t decel);

    /** @brief 绝对位置控制 (使能, 切PP模式, 设参, 目标, 启动), deg=角度° */
    int  AbsPosition(uint8_t id, float deg);

    /** @brief 绝对位置控制扩展版 (使能, halt, 切PP, accel/vel可配) */
    int  AbsPositionEx(uint8_t id, float deg, uint16_t accel, uint16_t vel);

    /** @brief 停止位置运动 (CW=0x0F) */
    void AbsStop(uint8_t id);

    /** @brief 零位标定 (自动失能, 写0x2531) */
    int  SetZero(uint8_t id);

    /** @brief 正限位 (失能, 写0x607D/02, save_flash) */
    int  SetPosLimit(uint8_t id, float deg);

    /** @brief 负限位 (失能, 写0x607D/01, save_flash) */
    int  SetNegLimit(uint8_t id, float deg);

    /** @brief 读正限位 */
    int  ReadPosLimit(uint8_t id, float* out_deg);

    /** @brief 读负限位 */
    int  ReadNegLimit(uint8_t id, float* out_deg);

    /** @brief 保存参数到 Flash */
    int  SaveFlash(uint8_t id);

    /** @brief 设置 PID 参数 */
    int  SetPid(uint8_t id, uint16_t cp, uint16_t ci,
                uint16_t vp, uint16_t vi, uint16_t pp, uint16_t pi);

    /** @brief 读取 PID 参数 */
    int  ReadPid(uint8_t id, motor_pid_t* out_pid);

    /* 通用 SDO 读写 */

    /** @brief SDO 读 U32 */
    int  SdoRead(uint8_t id, uint16_t index, uint8_t subidx, uint32_t* out_val);

    /** @brief SDO 写 */
    int  SdoWrite(uint8_t id, uint16_t index, uint8_t subidx, uint32_t value, uint8_t size);

    /* 读取命令 (从反馈缓存, 非阻塞) */

    /** @brief 读编码器角度 (°) via SDO */
    float ReadAngle(uint8_t id);

    /** @brief 读转速 (RPM) via SDO */
    int32_t ReadSpeed(uint8_t id);

    /** @brief 读 Q 轴电流 (mA) via SDO */
    int32_t ReadCurrent(uint8_t id);

    /** @brief 读电机温度 (°C) via SDO */
    float ReadTemp(uint8_t id);

    /** @brief 读电机状态 (motor_state_t) */
    int ReadState(uint8_t id);

    /** @brief 读故障码 */
    int ReadError(uint8_t id, uint16_t* out_err);

    /** @brief 读固件版本 */
    int ReadVersion(uint8_t id, uint32_t* out_ver);

    /** @brief 读当前模式 */
    int ReadMode(uint8_t id, uint8_t* out_mode);

    /* PDO 控制 (非阻塞, RT 安全) */

    /** @brief PDO 单轴控制 (自定义 PDO 0x100+ID), 位置/速度单位同 motor_hal API */
    void PdoCtrl(uint8_t id, motor_mode_t mode, float target);

    /** @brief PDO 多轴广播 (0x200, 64B CANFD) */
    void PdoMultiCtrl(const multi_axis_cmd_t* cmds, uint8_t count);

    /** @brief MIT 阻抗控制 (PDO 0x110+ID) */
    void PdoMitCtrl(uint8_t id, float pos, float vel,
                    int16_t kp, int16_t kd, int16_t torque);

    /* PDO 映射 (SDO 路径, 非 RT) */

    /** @brief 通用 PDO 映射 */
    int PdoMap(uint8_t id, pdo_type_t type,
               const pdo_map_entry_cfg_t* entries, uint8_t count,
               uint32_t cob_id, uint8_t trans_type);

    /** @brief 发送标准 RPDO */
    int RpdoSend(uint8_t id, const uint8_t* data, uint8_t dlc);

    /* 传感器/反馈 (非阻塞) */

    /** @brief 从反馈缓存读电机反馈, 0=成功 */
    int GetFeedback(uint8_t id, motor_feedback_t* out_fb);

    /** @brief 从传感器缓存读透传数据, 0=成功 */
    int GetSensor(uint8_t id, motor_sensor_t* out_sensor);

    /** @brief 配置传感器透传 */
    int SensorConfig(uint8_t id, uint16_t period_div, uint8_t bus_format);

    /** @brief 停止传感器透传 */
    int SensorStop(uint8_t id);

    /* 辅助 */

    /** @brief 获取 HAL 原始指针 (高级用途) */
    motor_hal_t* GetHal() { return m_hal; }

    /** @brief 检查电机是否在线 (从反馈缓存 + 状态) */
    bool IsOnline(uint8_t id);

private:
    motor_hal_t* m_hal;

    /* 位置控制参数 */
    uint16_t m_abs_accel;   /* RPM/s, 默认 2000 */
    uint16_t m_abs_speed;   /* RPM 输出端, 默认 10 */

    /* 模式缓存 (幂等优化) */
    uint8_t m_mode_cache[128];  /* 按 node_id 索引 */

    int _set_mode_cached(uint8_t id, motor_mode_t mode);
    void _halt_motion(uint8_t id);
};

}  /* namespace stark_periph_manager_node */
