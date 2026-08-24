/*
 * safety_controller.h — 安全控制器接口 (L4, 预留)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 安全兜底必须紧贴硬件、RT 域运行、独立于算法。
 * 本文件只定义接口, 具体检查逻辑后续按需实现, 上层 RT 循环已预留调用点,
 * 后续新增安全项 = 实现接口方法, 不改框架。
 */
#pragma once

#include <cstdint>

namespace stark {

/* 安全上下文: RT 循环每周期传入的安全相关数据 */
struct SafetyContext {
    uint32_t cycle;              /* 当前周期 */
    uint32_t algo_heartbeat_ms;  /* 算法心跳 (ms) */
    int      motor_count;        /* 电机数 */
};

/*
 * 安全控制器接口
 *
 * RT 循环每周期调用 check(), 返回 false 表示需要脱使能;
 * emergencyStop() 供急停路径调用 (可能 RT 调用, 必须非阻塞)。
 */
class ISafetyController {
public:
    virtual ~ISafetyController() = default;

    /* RT: 每周期安全检查, 返回 false = 触发脱使能 */
    virtual bool check(const SafetyContext& ctx) = 0;

    /* RT: 紧急停止, 立即脱使能 */
    virtual void emergencyStop() = 0;
};

/*
 * 空实现 (默认): 不检查, 始终安全。
 * 后续实现真实安全控制器时替换。
 */
class NullSafetyController : public ISafetyController {
public:
    bool check(const SafetyContext& ctx) override {
        (void)ctx;
        return true;
    }
    void emergencyStop() override {}
};

}  // namespace stark
