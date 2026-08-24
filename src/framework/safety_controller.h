/*
 * safety_controller.h — 安全控制器接口 (L4, 预留)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 安全兜底必须紧贴硬件、RT 域运行、独立于算法。
 * 电机内部(驱动板固件)已实现过流/过温/堵转等硬件保护, host 侧只负责
 * 电机覆盖不到的场景(算法故障/通信超时/系统级急停)。
 * 本文件只定义接口 + 空实现, 具体检查逻辑确认后再实现, 不改框架。
 */
#pragma once

#include <cstdint>

namespace stark {

/* 安全上下文: RT 循环每周期传入 */
struct SafetyContext {
    uint32_t cycle;              /* 当前周期 */
    uint32_t algo_heartbeat_ms;  /* 算法心跳年龄 (ms, 0=未启用) */
    int      motor_count;        /* 电机数 */
};

/*
 * 安全控制器接口 (预留)
 *
 * RT 循环每周期调用 check(), 返回 false 表示触发脱使能;
 * emergencyStop() 供急停路径调用 (可能 RT 调用, 必须非阻塞)。
 */
class ISafetyController {
public:
    virtual ~ISafetyController() = default;
    virtual bool check(const SafetyContext& ctx) = 0;
    virtual void emergencyStop() = 0;
};

/* 空实现 (默认): 不检查, 始终安全 */
class NullSafetyController : public ISafetyController {
public:
    bool check(const SafetyContext& ctx) override {
        (void)ctx;
        return true;
    }
    void emergencyStop() override {}
};

}  // namespace stark
