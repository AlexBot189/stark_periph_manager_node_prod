/*
 * motor_canfd.h — 巨蟹 CANFD 电机设备 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 包装 motor_hal, 电机数量由配置决定, 不写死。
 * RT 数据路径直接用 hal() 返回的 motor_hal_t*, 不走虚接口 (零虚调用/零延时)。
 * 换电机厂商 = 写一个新的 Device 实现 + REGISTER_DEVICE 注册, 上层零改动。
 */
#pragma once

#include <cstddef>   /* size_t */
#include <string>

#include "framework/device.h"
#include "motor_hal.h"

namespace stark {

class MotorCanfd : public Device {
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

    /* 非RT操作访问底层 motor_hal (SDO/标定/sync/recv 配置; RT 直连此指针) */
    motor_hal_t* hal() const { return m_hal; }

private:
    std::string  m_name;
    motor_hal_t* m_hal = nullptr;
    size_t       m_motor_count = 0;   /* 已注册电机数 */
};

}  // namespace stark
