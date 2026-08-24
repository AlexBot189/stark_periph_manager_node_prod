/*
 * safety_controller.h — 安全控制器接口 + 具体实现 (L4)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 安全兜底必须紧贴硬件、RT 域运行、独立于算法。
 * 算法挂/超时/发异常命令, 安全控制器要能兜底 (脱使能)。
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <vector>

#include "framework/device.h"

namespace stark {

/* 安全上下文: RT 循环每周期传入 */
struct SafetyContext {
    uint32_t cycle;              /* 当前周期 */
    uint32_t algo_heartbeat_ms;  /* 算法心跳年龄 (ms, 0=未启用) */
    int      motor_count;        /* 电机数 */
};

/* 安全阈值 */
struct SafetyLimits {
    uint32_t heartbeat_timeout_ms = 0;      /* 算法心跳超时 (0=不检查, 默认关避免误脱使能) */
    int32_t  overtemp_celsius     = 80;     /* 过温阈值 (°C) */
    int32_t  overcurrent_ma       = 0;      /* 过流阈值 (mA, 0=不检查, 电机型号相关) */
    uint32_t encoder_stall_s      = 3;      /* 编码器停滞 (s, 0=不检查) */
};

/*
 * 安全控制器接口
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

/* 单调时钟 us */
static inline uint64_t _safety_now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/*
 * 具体安全控制器: 过流 / 过温 / 心跳超时 / 编码器停滞 → 脱使能
 */
class SafetyController : public ISafetyController {
public:
    SafetyController(IMotorDevice* motor, const SafetyLimits& limits)
        : m_motor(motor), m_limits(limits) {}

    bool check(const SafetyContext& ctx) override;
    void emergencyStop() override;

private:
    struct MotorSafetyState {
        int32_t  last_position  = 0;
        uint64_t stall_since_us = 0;  /* 停滞起始时刻 (0=未停滞) */
    };

    IMotorDevice*             m_motor;
    SafetyLimits              m_limits;
    std::vector<MotorSafetyState> m_states;
};

}  // namespace stark
