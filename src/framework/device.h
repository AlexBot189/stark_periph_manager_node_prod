/*
 * device.h — 设备抽象接口 (L1)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 借鉴内核 device model + Android HAL: 接口稳定, 实现可换。
 *
 * 生命周期: initialize -> start -> [运行] -> stop
 *   initialize/start/stop 由非 RT 线程调用 (可阻塞、可分配)。
 *
 * 设计约束 (RT 铁律):
 *   RT 数据路径 (1kHz 控制循环) 不走本接口的虚函数 — 各 L0 设备暴露底层实现
 *   指针 (hal()/source()/sensor()), RT 线程直接调用, 零虚调用/零延时。
 *
 * 加设备 = 继承 Device + 实现 4 方法 + REGISTER_DEVICE 注册;
 * 换设备 = 换 L0 驱动实现, 上层零改动。
 */
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>

namespace stark {

/*
 * 设备基类 — 通用标识 + 生命周期
 */
class Device {
public:
    virtual ~Device() = default;

    virtual const char* name() const = 0;    /* 实例名, 如 "motor_hip_right" */
    virtual const char* type() const = 0;    /* 类别, 如 "motor"/"imu" */

    virtual bool initialize(const nlohmann::json& config) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
};

}  // namespace stark
