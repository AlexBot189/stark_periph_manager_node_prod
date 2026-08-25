/*
 * rt_log_events.h — RT 日志事件表 (单一来源)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * X-Macro: 事件名 + 格式串。编译期同时生成「事件号枚举」和「格式串表」,
 * 加/改一条日志只动这一处, 不会漏、不会错位。
 *
 * 格式串 specifier 约束:
 *   - %u / %d / %x / %c   各占 1 个 uint32 参数槽
 *   - %llu / %lld         各占 2 个 uint32 参数槽 (低32 + 高32)
 *   - %%                  字面百分号
 *   RT 日志禁用 %f / %s: 浮点会引入额外开销, 字符串跨进程无指针语义。
 */
#pragma once

#define RT_LOG_EVENTS(X) \
    X(CYCLE_JITTER,      "CycleJitter min=%u avg=%u max=%u samples=%u") \
    X(CYCLE_OVERRUN,     "CycleJitter overrun=%llu") \
    X(SENSOR_NOCFG,      "sensor not configured motor=%u ret=%d") \
    X(MULTI_INVALID,     "M%u multi_mode=%u invalid, skip") \
    X(SCHED_FIFO_FAIL,   "SCHED_FIFO failed (need root/CAP_SYS_NICE)") \
    X(MLOCKALL_FAIL,     "mlockall failed (page faults possible)") \
    X(CPU_AFFINITY_FAIL, "CPU affinity failed")
