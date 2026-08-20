/*
 * stark_latency_trace.h — 耗时追踪 (宏开关控制)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 启用:   #define STARK_LATENCY_TRACE  1
 * 关闭:   #define STARK_LATENCY_TRACE  0  (零开销)
 *
 * 追踪节点 (RT线程内):
 *   反馈路径: T1 fb_read_start ,  T4 shm_write_done
 *   控制路径: T5 mailbox_read ,  T6 pdo_sent
 *
 * 输出: 每1000周期统计一次 (min/avg/max),
 *       同时写入 SHM 供 perf_test 读取.
 */
#pragma once

#define STARK_LATENCY_TRACE  1    /* 1=启用耗时追踪, 0=零开销 */

#include <cstdint>
#include <cstring>
#include <ctime>
#include <algorithm>

namespace stark_periph_manager_node {

/* 追踪数据结构 */

struct latency_sample_t {
    uint64_t t1_fb_read_start;
    uint64_t t2_fb_read_done;
    uint64_t t3_mock_sensor_done;
    uint64_t t4_shm_write_done;
    uint64_t t5_mailbox_read;
    uint64_t t6_pdo_sent;
    uint64_t ctrl_origin_us;   /* 算法写 mailbox 时刻 (CLOCK_MONOTONIC) */
    uint64_t fb_origin_us;     /* CAN 收帧时刻 (CLOCK_MONOTONIC) */
};

struct latency_stats_t {
    /* 反馈路径 (μs) */
    uint32_t fb_read_min;
    uint32_t fb_read_avg;
    uint32_t fb_read_max;
    uint32_t mock_sensor_min;
    uint32_t mock_sensor_avg;
    uint32_t mock_sensor_max;
    uint32_t shm_write_min;
    uint32_t shm_write_avg;
    uint32_t shm_write_max;
    uint32_t fb_total_min;        /* T1, T4 */
    uint32_t fb_total_avg;
    uint32_t fb_total_max;

    /* 控制路径 (μs) */
    uint32_t ctrl_total_min;      /* T5, T6 */
    uint32_t ctrl_total_avg;
    uint32_t ctrl_total_max;

    /* 端到端 (μs) */
    uint32_t ctrl_e2e_min;        /* 算法下发 → PDO 发出 */
    uint32_t ctrl_e2e_avg;
    uint32_t ctrl_e2e_max;
    uint32_t fb_e2e_min;          /* CAN 收帧 → SHM 写完 */
    uint32_t fb_e2e_avg;
    uint32_t fb_e2e_max;

    /* 全局 */
    uint32_t cycle_count;         /* 已采样的周期数 */
    uint32_t overrun_count;       /* 周期超限次数 */
};

/* 追踪器 (RT线程内使用, 全部非阻塞) */

class StarkLatencyTracer {
public:
    StarkLatencyTracer() : m_sample_idx(0), m_stat_cycle(0) {
        reset_stats();
    }

    /* 运行时开关: 关闭后所有打点/统计/输出均为零开销 (由 config rt.perf_trace 控制) */
    void SetEnabled(bool en) { m_enabled = en; }
    bool IsEnabled() const   { return m_enabled; }

#if STARK_LATENCY_TRACE

    /*
     * 记录时间戳 — 在 RT 线程各阶段调用.
     * clock_gettime(CLOCK_MONOTONIC) 是 vDSO 系统调用, ~20ns.
     */

    static uint64_t now_us() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    }

    void mark_fb_read_start()    { if (!m_enabled) return; m_cur.t1_fb_read_start    = now_us(); }
    void mark_fb_read_done()     { if (!m_enabled) return; m_cur.t2_fb_read_done     = now_us(); }
    void mark_mock_sensor_done() { if (!m_enabled) return; m_cur.t3_mock_sensor_done = now_us(); }
    void mark_shm_write_done()   { if (!m_enabled) return; m_cur.t4_shm_write_done   = now_us(); }
    void mark_mailbox_read()     { if (!m_enabled) return; m_cur.t5_mailbox_read     = now_us(); }
    void mark_pdo_sent()         { if (!m_enabled) return; m_cur.t6_pdo_sent         = now_us(); }
    void mark_ctrl_origin(uint64_t us) { if (!m_enabled) return; m_cur.ctrl_origin_us = us; }
    void mark_fb_origin(uint64_t us)   { if (!m_enabled) return; m_cur.fb_origin_us   = us; }

    /*
     * 提交当前周期样本到环形缓冲区, 每1000周期输出统计.
     */
    void commit_sample() {
        if (!m_enabled) return;

        uint32_t idx = m_sample_idx % LATENCY_RING_SIZE;
        m_samples[idx] = m_cur;

        /* 计算各阶段延迟 (μs) */
        uint32_t fb_read    = (uint32_t)(m_cur.t2_fb_read_done - m_cur.t1_fb_read_start);
        uint32_t mock_snsr  = (uint32_t)(m_cur.t3_mock_sensor_done - m_cur.t2_fb_read_done);
        uint32_t shm_write  = (uint32_t)(m_cur.t4_shm_write_done - m_cur.t3_mock_sensor_done);
        uint32_t fb_total   = (uint32_t)(m_cur.t4_shm_write_done - m_cur.t1_fb_read_start);

        /* 仅当本周期实际执行了控制命令时才统计算 */
        uint32_t ctrl_total = 0;
        bool     ctrl_valid = (m_cur.t5_mailbox_read > 0);
        if (ctrl_valid) {
            ctrl_total = (uint32_t)(m_cur.t6_pdo_sent - m_cur.t5_mailbox_read);
        }

        /* 端到端: 算法下发 → PDO 发出 (需算法侧写入 origin 时间戳) */
        uint32_t ctrl_e2e = 0;
        bool ctrl_e2e_valid = (m_cur.ctrl_origin_us > 0 &&
                               m_cur.t6_pdo_sent >= m_cur.ctrl_origin_us);
        if (ctrl_e2e_valid) {
            ctrl_e2e = (uint32_t)(m_cur.t6_pdo_sent - m_cur.ctrl_origin_us);
        }

        /* 端到端: CAN 收帧 → SHM 写完 */
        uint32_t fb_e2e = 0;
        bool fb_e2e_valid = (m_cur.fb_origin_us > 0 &&
                             m_cur.t4_shm_write_done >= m_cur.fb_origin_us);
        if (fb_e2e_valid) {
            fb_e2e = (uint32_t)(m_cur.t4_shm_write_done - m_cur.fb_origin_us);
        }

        /* 累计统计 */
        m_fb_read_acc    += fb_read;    m_fb_read_sq  += (uint64_t)fb_read  * fb_read;
        m_mock_snsr_acc  += mock_snsr;  m_mock_snsr_sq  += (uint64_t)mock_snsr * mock_snsr;
        m_shm_write_acc  += shm_write;  m_shm_write_sq  += (uint64_t)shm_write * shm_write;
        m_fb_total_acc   += fb_total;   m_fb_total_sq   += (uint64_t)fb_total * fb_total;
        if (ctrl_valid) {
            m_ctrl_total_acc += ctrl_total; m_ctrl_total_sq += (uint64_t)ctrl_total * ctrl_total;
            m_ctrl_sample_count++;
        }
        if (ctrl_e2e_valid) {
            m_ctrl_e2e_acc += ctrl_e2e; m_ctrl_e2e_sq += (uint64_t)ctrl_e2e * ctrl_e2e;
            m_ctrl_e2e_sample_count++;
        }
        if (fb_e2e_valid) {
            m_fb_e2e_acc += fb_e2e; m_fb_e2e_sq += (uint64_t)fb_e2e * fb_e2e;
            m_fb_e2e_sample_count++;
        }

        if (fb_read    > m_fb_read_max)    m_fb_read_max    = fb_read;
        if (fb_read    < m_fb_read_min)    m_fb_read_min    = fb_read;
        if (mock_snsr  > m_mock_snsr_max)  m_mock_snsr_max  = mock_snsr;
        if (mock_snsr  < m_mock_snsr_min)  m_mock_snsr_min  = mock_snsr;
        if (shm_write  > m_shm_write_max)  m_shm_write_max  = shm_write;
        if (shm_write  < m_shm_write_min)  m_shm_write_min  = shm_write;
        if (fb_total   > m_fb_total_max)   m_fb_total_max   = fb_total;
        if (fb_total   < m_fb_total_min)   m_fb_total_min   = fb_total;
        if (ctrl_valid) {
            if (ctrl_total > m_ctrl_total_max) m_ctrl_total_max = ctrl_total;
            if (ctrl_total < m_ctrl_total_min) m_ctrl_total_min = ctrl_total;
        }
        if (ctrl_e2e_valid) {
            if (ctrl_e2e > m_ctrl_e2e_max) m_ctrl_e2e_max = ctrl_e2e;
            if (ctrl_e2e < m_ctrl_e2e_min) m_ctrl_e2e_min = ctrl_e2e;
        }
        if (fb_e2e_valid) {
            if (fb_e2e > m_fb_e2e_max) m_fb_e2e_max = fb_e2e;
            if (fb_e2e < m_fb_e2e_min) m_fb_e2e_min = fb_e2e;
        }

        m_sample_idx++;

        /* 每1000周期: 推统计到 ring buffer (RT_LOG), 不直接 printf */
        m_stat_cycle++;
        if (m_stat_cycle >= 1000) {
            uint32_t n = (m_sample_idx < 1000) ? m_sample_idx : 1000;
            flush_stats_to_ring(n);
            reset_stats();
            m_stat_cycle = 0;
        }

        /* 清空 origin 时间戳, 避免下一周期无数据时沿用旧值 */
        m_cur.ctrl_origin_us = 0;
        m_cur.fb_origin_us   = 0;
    }

    /*
     * 填充 SHM 中的 stats 字段 (perf_test 可读).
     */
    void fill_shm_stats(latency_stats_t& out) {
        if (!m_enabled) return;

        uint32_t n = (m_sample_idx < 1000) ? m_sample_idx : 1000;
        if (n == 0) return;

        out.fb_read_min  = m_fb_read_min;
        out.fb_read_avg  = (uint32_t)(m_fb_read_acc  / n);
        out.fb_read_max  = m_fb_read_max;
        out.mock_sensor_min = m_mock_snsr_min;
        out.mock_sensor_avg = (uint32_t)(m_mock_snsr_acc / n);
        out.mock_sensor_max = m_mock_snsr_max;
        out.shm_write_min = m_shm_write_min;
        out.shm_write_avg = (uint32_t)(m_shm_write_acc / n);
        out.shm_write_max = m_shm_write_max;
        out.fb_total_min = m_fb_total_min;
        out.fb_total_avg = (uint32_t)(m_fb_total_acc / n);
        out.fb_total_max = m_fb_total_max;
        out.ctrl_total_min = m_ctrl_total_min;
        out.ctrl_total_avg = (m_ctrl_sample_count > 0)
            ? (uint32_t)(m_ctrl_total_acc / m_ctrl_sample_count) : 0;
        out.ctrl_total_max = m_ctrl_total_max;
        out.ctrl_e2e_min = m_ctrl_e2e_min;
        out.ctrl_e2e_avg = (m_ctrl_e2e_sample_count > 0)
            ? (uint32_t)(m_ctrl_e2e_acc / m_ctrl_e2e_sample_count) : 0;
        out.ctrl_e2e_max = m_ctrl_e2e_max;
        out.fb_e2e_min = m_fb_e2e_min;
        out.fb_e2e_avg = (m_fb_e2e_sample_count > 0)
            ? (uint32_t)(m_fb_e2e_acc / m_fb_e2e_sample_count) : 0;
        out.fb_e2e_max = m_fb_e2e_max;
        out.cycle_count = m_sample_idx;
    }

    /* 清零所有本轮累计 max (web 下发"清空/刷新"时调用), 重新开始新一轮统计 */
    void reset_run_max() {
        m_fb_read_max    = 0;
        m_mock_snsr_max  = 0;
        m_shm_write_max  = 0;
        m_fb_total_max   = 0;
        m_ctrl_total_max = 0;
        m_ctrl_e2e_max   = 0;
        m_fb_e2e_max     = 0;
    }

#else  /* STARK_LATENCY_TRACE == 0 — 全部内联为空, 零开销 */

    static uint64_t now_us() { return 0; }
    void mark_fb_read_start()    {}
    void mark_fb_read_done()     {}
    void mark_mock_sensor_done() {}
    void mark_shm_write_done()   {}
    void mark_mailbox_read()     {}
    void mark_pdo_sent()         {}
    void mark_ctrl_origin(uint64_t) {}
    void mark_fb_origin(uint64_t)   {}
    void commit_sample()         {}
    void fill_shm_stats(latency_stats_t&) {}
    void reset_run_max()         {}

#endif  /* STARK_LATENCY_TRACE */

private:
    bool m_enabled = true;    /* 运行时总开关 (默认开, 保持原行为) */

#if STARK_LATENCY_TRACE
    static constexpr int LATENCY_RING_SIZE = 1024;

    void flush_stats_to_ring(uint32_t n);
    void reset_stats();

    latency_sample_t m_cur;
    latency_sample_t m_samples[LATENCY_RING_SIZE];
    uint32_t m_sample_idx;
    uint32_t m_stat_cycle;

    /* 累计值 */
    uint64_t m_fb_read_acc,   m_fb_read_sq;
    uint32_t m_fb_read_min = UINT32_MAX, m_fb_read_max = 0;
    uint64_t m_mock_snsr_acc, m_mock_snsr_sq;
    uint32_t m_mock_snsr_min = UINT32_MAX, m_mock_snsr_max = 0;
    uint64_t m_shm_write_acc, m_shm_write_sq;
    uint32_t m_shm_write_min = UINT32_MAX, m_shm_write_max = 0;
    uint64_t m_fb_total_acc,  m_fb_total_sq;
    uint32_t m_fb_total_min = UINT32_MAX,  m_fb_total_max = 0;
    uint64_t m_ctrl_total_acc,m_ctrl_total_sq;
    uint32_t m_ctrl_total_min = UINT32_MAX, m_ctrl_total_max = 0;
    uint32_t m_ctrl_sample_count = 0;
    uint64_t m_ctrl_e2e_acc, m_ctrl_e2e_sq;
    uint32_t m_ctrl_e2e_min = UINT32_MAX, m_ctrl_e2e_max = 0;
    uint32_t m_ctrl_e2e_sample_count = 0;
    uint64_t m_fb_e2e_acc, m_fb_e2e_sq;
    uint32_t m_fb_e2e_min = UINT32_MAX, m_fb_e2e_max = 0;
    uint32_t m_fb_e2e_sample_count = 0;
#endif
};

}  /* namespace stark_periph_manager_node */
