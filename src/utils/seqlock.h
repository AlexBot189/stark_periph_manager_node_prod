/*
 * seqlock.h — 顺序锁 (单生产者单消费者)
 *
 * 传感器缓存的读写原语: 后台线程写, RT 线程读, 无锁无阻塞。
 * 读者检测到写者"写中"(序号奇数)或序号变化时重读, 保证拿到完整一致的快照。
 *
 * 约束:
 *   - 严格单生产者单消费者 (一个写者 + 一个读者)
 *   - T 为 POD 结构体 (imu_data_t / foot_pressure_data_t / barometer_data_t)
 */
#pragma once

#include <atomic>
#include <cstdint>

namespace stark_periph_manager_node {

template <typename T>
class Seqlock {
public:
    /* 写者调用 (单生产者) */
    void store(const T& v)
    {
        uint32_t s = m_seq.load(std::memory_order_relaxed);
        m_seq.store(s + 1, std::memory_order_relaxed);   /* 奇数 = 写中 */
        std::atomic_thread_fence(std::memory_order_release);
        m_data = v;
        std::atomic_thread_fence(std::memory_order_release);
        m_seq.store(s + 2, std::memory_order_relaxed);   /* 偶数 = 写完 */
    }

    /* 读者调用 (无锁, 撞上写者则重读) */
    void load(T& out) const
    {
        uint32_t s1, s2;
        do {
            s1 = m_seq.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            out = m_data;
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = m_seq.load(std::memory_order_relaxed);
        } while (s1 != s2 || (s1 & 1));
    }

private:
    std::atomic<uint32_t> m_seq{0};
    T m_data{};
};

}  /* namespace stark_periph_manager_node */
