/*
 * motor_canfd.h — 巨蟹 CANFD 电机设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 包装 motor_hal, 实现 IMotorDevice。电机数量由配置决定, 不写死。
 * 换电机厂商 = 写一个新的 IMotorDevice 实现 + REGISTER_DEVICE 注册,
 * framework 层零改动。
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "framework/device.h"
#include "motor_hal.h"

namespace stark {

class MotorCanfd : public IMotorDevice {
public:
    static Device* create() { return new MotorCanfd(); }

    MotorCanfd() = default;
    ~MotorCanfd() override;

    /* Device */
    const char* name() const override { return m_name.c_str(); }
    const char* type() const override { return "motor"; }
    bool initialize(const nlohmann::json& config) override;
    bool start() override;
    void stop() override;

    /* IMotorDevice */
    int  motorCount() const override { return (int)m_node_ids.size(); }
    bool readFeedback(int index, MotorFeedback& fb) override;
    bool writeCommand(int index, const MotorCommand& cmd) override;
    bool writeMitCommand(int index, float position, float velocity,
                         float kp, float kd, float torque) override;
    bool enable(int index) override;
    bool disable(int index) override;
    bool clearFault(int index) override;

    /* 非RT操作访问底层 motor_hal (SDO/标定/sync/recv 配置) */
    motor_hal_t* hal() const { return m_hal; }

private:
    bool validIndex(int index) const {
        return index >= 0 && index < (int)m_node_ids.size();
    }

    std::string    m_name;
    motor_hal_t*   m_hal = nullptr;
    std::vector<uint8_t> m_node_ids;   /* index → node_id */
};

}  // namespace stark
