/*
 * stark_rt_log.h — RT 线程安全日志
 * Copyright (c) 2026 zhiqiang.yang
 *
 * RT 工作线程不能直接调写盘/控制台 (可能阻塞).
 * 用 lock-free ring buffer 缓冲, 由独立日志线程 drain.
 *
 * ring buffer 单写单读, 不需要锁.
 */
#pragma once

#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#define RT_LOG_BUF_SIZE   256     /* 单条消息最大字节 */
#define RT_LOG_RING_SIZE  64      /* 环形缓冲区条数   */

namespace stark_periph_manager_node {

class StarkRtLog {
public:
    StarkRtLog()
        : m_wr(0), m_rd(0), m_overflow(0)
    {
        memset(m_ring, 0, sizeof(m_ring));
    }

    /* RT 线程调用: lock-free push (< 1μs) */
    void Push(const char* fmt, ...) __attribute__((format(printf, 2, 3)))
    {
        char buf[RT_LOG_BUF_SIZE];

        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);

        uint32_t next = (m_wr + 1) % RT_LOG_RING_SIZE;
        if (next == m_rd) {
            /* 满: 覆盖最老的消息 (RT 不能阻塞等) */
            m_rd = (m_rd + 1) % RT_LOG_RING_SIZE;
            m_overflow++;
        }

        memcpy(m_ring[m_wr], buf, RT_LOG_BUF_SIZE);
        __atomic_store_n(&m_wr, next, __ATOMIC_RELAXED);
    }

    /* 非 RT 日志线程调用: drain ,  callback */
    void Drain(void (*output_fn)(const char* msg))
    {
        uint32_t w = __atomic_load_n(&m_wr, __ATOMIC_RELAXED);
        while (m_rd != w) {
            if (output_fn) {
                output_fn(m_ring[m_rd]);
            }
            m_rd = (m_rd + 1) % RT_LOG_RING_SIZE;
        }
    }

    uint32_t GetOverflow() const { return m_overflow; }

private:
    char     m_ring[RT_LOG_RING_SIZE][RT_LOG_BUF_SIZE];
    uint32_t m_wr;          /* RT 线程写索引 */
    uint32_t m_rd;          /* 日志线程读索引 (单消费者, 不 atomic) */
    uint32_t m_overflow;    /* 溢出次数 (调试用) */
};

}  /* namespace stark_periph_manager_node */

/* ── 便捷宏 ── */

/** @brief RT 线程日志 (va_args 会被 Snprintf 到 ring buffer) */
#define RT_LOG(fmt, ...) \
    do { if (g_rt_log) g_rt_log->Push(fmt, ##__VA_ARGS__); } while(0)

extern stark_periph_manager_node::StarkRtLog* g_rt_log;
