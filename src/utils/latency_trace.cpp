/*
 * stark_latency_trace.cpp — 耗时追踪实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * RT 线程内不做 printf, 统计摘要通过 RT_LOG ring buffer 输出.
 */
#include "utils/latency_trace.h"
#include "utils/rt_log.h"

namespace stark_periph_manager_node {

#if STARK_LATENCY_TRACE

void StarkLatencyTracer::reset_stats()
{
    m_fb_read_min  = UINT32_MAX; m_fb_read_max  = 0; m_fb_read_acc  = 0; m_fb_read_sq  = 0;
    m_mock_snsr_min = UINT32_MAX; m_mock_snsr_max = 0; m_mock_snsr_acc = 0; m_mock_snsr_sq = 0;
    m_shm_write_min = UINT32_MAX; m_shm_write_max = 0; m_shm_write_acc = 0; m_shm_write_sq = 0;
    m_fb_total_min  = UINT32_MAX; m_fb_total_max  = 0; m_fb_total_acc  = 0; m_fb_total_sq  = 0;
    m_ctrl_total_min = UINT32_MAX; m_ctrl_total_max = 0; m_ctrl_total_acc = 0; m_ctrl_total_sq = 0;
    m_ctrl_sample_count = 0;
}

/*
 * flush_stats_to_ring — RT线程调用, 通过RT_LOG ring buffer输出
 * (不调printf, 不阻塞, <2μs)
 */
void StarkLatencyTracer::flush_stats_to_ring(uint32_t n)
{
    uint32_t fb_read_avg   = (uint32_t)(m_fb_read_acc / n);
    uint32_t mock_avg      = (uint32_t)(m_mock_snsr_acc / n);
    uint32_t shm_avg       = (uint32_t)(m_shm_write_acc / n);
    uint32_t fb_total_avg  = (uint32_t)(m_fb_total_acc / n);
    uint32_t ctrl_sample = (m_ctrl_sample_count > 0) ? m_ctrl_sample_count : 1;
    uint32_t ctrl_total_avg = (uint32_t)(m_ctrl_total_acc / ctrl_sample);

    if (m_ctrl_sample_count > 0) {
        RT_LOG("LatencyTrace[%u] | fb_read: min=%u avg=%u max=%u us | "
               "mock: avg=%u us | SHM: avg=%u us | fb_total: min=%u avg=%u max=%u us | "
               "ctrl: min=%u avg=%u max=%u us (samples=%u)",
               n,
               m_fb_read_min, fb_read_avg, m_fb_read_max,
               mock_avg, shm_avg,
               m_fb_total_min, fb_total_avg, m_fb_total_max,
               m_ctrl_total_min, ctrl_total_avg, m_ctrl_total_max,
               m_ctrl_sample_count);
    } else {
        RT_LOG("LatencyTrace[%u] | fb_read: min=%u avg=%u max=%u us | "
               "mock: avg=%u us | SHM: avg=%u us | fb_total: min=%u avg=%u max=%u us | "
               "ctrl: (no samples)",
               n,
               m_fb_read_min, fb_read_avg, m_fb_read_max,
               mock_avg, shm_avg,
               m_fb_total_min, fb_total_avg, m_fb_total_max);
    }
}

#endif  /* STARK_LATENCY_TRACE */

}  /* namespace stark_periph_manager_node */
