/*
 * rt_log.h — RT 线程零耗时日志接口
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 设计原则 (RT 铁律):
 *   RT 线程 (1kHz) 内不能有锁 / 大块内存拷贝 / 字符串格式化 / syscall。
 *   日志用「结构化事件」(level + event_id + 4×uint32 参数 + ns 时间戳) 直接写
 *   共享内存无锁 ring buffer (几条 store), 由非 RT drain 线程查表格式化后
 *   输出到 loghelper。
 *
 *   - 零锁:     每生产者独立 SPSC ring, 无 CAS 竞争
 *   - 零拷贝:   固定 32 字节事件, 几条 store
 *   - 零格式化: RT 侧只写原始参数, drain 侧查表格式化
 *   - 可关闭:   shm->enabled atomic 开关
 *   - 可调等级: shm->max_level atomic 阈值 (对齐 loghelper)
 *   - 跨进程:   POSIX 共享内存, 多生产者分 ring, 算法进程调用同一接口即可
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <time.h>   /* clock_gettime (vDSO, 无 syscall), struct timespec */

#include "rt_log_events.h"

/* ── 事件号枚举 (由 X-Macro 编译期生成) ── */
#define RT_LOG_ENUM(name, fmt) EV_##name,
enum {
    RT_LOG_EVENTS(RT_LOG_ENUM)
    EV_MAX
};
#undef RT_LOG_ENUM

/* ── 日志等级 (对齐 loghelper/spdlog, 数值越大越重要) ── */
enum {
    RT_LOG_TRACE = 0,
    RT_LOG_DEBUG = 1,
    RT_LOG_INFO  = 2,
    RT_LOG_WARN  = 3,
    RT_LOG_ERROR = 4,
};

/* ── SHM 布局 ── */
#define RT_LOG_SHM_NAME       "/stark_rtlog_shm"
#define RT_LOG_SHM_SIZE       (64 * 1024)
#define RT_LOG_MAX_PRODUCERS  8
#define RT_LOG_RING_SIZE      128

#define RT_LOG_MAGIC          0x52544C47U  /* "RTLG" */
#define RT_LOG_VERSION        1

/* 32 字节事件 (2 条占 1 个 64B cache line) */
typedef struct {
    uint32_t level;      /* RT_LOG_* */
    uint32_t event_id;   /* EV_* */
    uint32_t arg[4];     /* 参数槽 (uint32, 或拼 uint64) */
    uint64_t ts_ns;      /* CLOCK_MONOTONIC 纳秒 */
} rt_log_event_t;

/* 每生产者一个 SPSC 无锁 ring */
typedef struct {
    uint32_t used;       /* 1=已占用 */
    uint32_t pid;        /* 生产者 PID */
    uint32_t wr;         /* 写索引 (生产者写, drain 读) */
    uint32_t rd;         /* 读索引 (drain 写, 生产者读) */
    uint32_t overflow;   /* 溢出计数 */
    rt_log_event_t events[RT_LOG_RING_SIZE];
} rt_log_ring_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t enabled;         /* 全局开关 (atomic) */
    uint32_t max_level;       /* 等级阈值 (atomic), >= 此等级才记录 */
    uint32_t producer_count;  /* 活跃生产者数 */
    rt_log_ring_t rings[RT_LOG_MAX_PRODUCERS];
} rt_log_shm_t;

/* ── 进程级上下文 (每进程一个) ── */
typedef struct {
    rt_log_shm_t* shm;
    int           producer;
} rt_log_ctx_t;

extern rt_log_ctx_t g_rt_log_ctx;

/* 初始化: 打开/创建 SHM + 绑定 producer 槽位 (进程启动时调用一次) */
int rt_log_init(const char* shm_name, int producer_index);

/* 当前单调时钟纳秒 (vDSO, 无 syscall) */
static inline uint64_t rt_log_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/*
 * RT 线程调用: 写一条事件
 * 无锁 / 无 memcpy / 无格式化 / 无 syscall, 约几十 ns.
 */
static inline void rt_log_emit(uint32_t level, uint32_t event_id,
                               uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4)
{
    rt_log_shm_t* shm = g_rt_log_ctx.shm;
    if (!shm) return;

    /* 可关闭 + 可调等级 (两个 atomic load) */
    if (!__atomic_load_n(&shm->enabled, __ATOMIC_RELAXED)) return;
    if (level < __atomic_load_n(&shm->max_level, __ATOMIC_RELAXED)) return;

    rt_log_ring_t* ring = &shm->rings[g_rt_log_ctx.producer];
    uint32_t wr = __atomic_load_n(&ring->wr, __ATOMIC_RELAXED);
    uint32_t next = (wr + 1) % RT_LOG_RING_SIZE;

    rt_log_event_t* ev = &ring->events[wr];
    ev->level    = level;
    ev->event_id = event_id;
    ev->arg[0]   = a1;
    ev->arg[1]   = a2;
    ev->arg[2]   = a3;
    ev->arg[3]   = a4;
    ev->ts_ns    = rt_log_now_ns();

    /* 满: 覆盖最老 (RT 不能阻塞等) */
    uint32_t rd = __atomic_load_n(&ring->rd, __ATOMIC_RELAXED);
    if (next == rd) {
        __atomic_store_n(&ring->rd, (rd + 1) % RT_LOG_RING_SIZE, __ATOMIC_RELAXED);
        __atomic_fetch_add(&ring->overflow, 1, __ATOMIC_RELAXED);
    }

    __atomic_store_n(&ring->wr, next, __ATOMIC_RELAXED);
}

/*
 * 便捷宏: 固定 4 个参数槽, 不足补 0。
 * 例: RT_LOG(RT_LOG_WARN, MULTI_INVALID, c.motor_id, (uint32_t)m, 0, 0);
 *     RT_LOG(RT_LOG_WARN, SCHED_FIFO_FAIL, 0, 0, 0, 0);
 */
#define RT_LOG(level, event, a1, a2, a3, a4) \
    do { rt_log_emit((level), EV_##event, \
                     (uint32_t)(a1), (uint32_t)(a2), (uint32_t)(a3), (uint32_t)(a4)); } while(0)

/* ── drain 端 (非 RT) ── */

/* 按事件号返回格式串 (drain 侧用, 未知返回 NULL) */
const char* rt_log_event_fmt(uint32_t event_id);

/* 格式化一条事件到 out (drain 侧用, 非 RT, 支持 %u/%d/%x/%c/%llu/%lld/%%) */
int rt_log_format(const rt_log_event_t* ev, char* out, size_t n);

/* 初始化文件 sink (读 ECO_RT_LOG_* 环境变量 + 打开文件), 0=成功 */
int rt_log_sink_init(void);

/* 轮询所有 producer ring, 格式化 + 写文件/console (非 RT, drain 线程调用) */
void rt_log_drain(void);

/* 关闭文件 sink (flush 后关闭) */
void rt_log_sink_close(void);
