/*
 * FootPressureSensor.h -- 足底压力传感器封装
 * Copyright (c) 2026 zhiqiang.yang
 *
 * MCU 通过 UART /dev/ttyS7 以 1000Hz 主动上报 20 字节固定帧 (含 1 字节累加和校验)。
 * 内部串口线程阻塞读取 + 帧解析, 对外 Read() 非阻塞返回缓存。
 *
 * 故障不影响电机状态机: Init 失败时 IsReady()=false, Read() 返回全零。
 */
#pragma once

#include <atomic>
#include <thread>
#include <cstdint>
#include <pthread.h>

extern "C" {
#include "stark_shm.h"
}

namespace stark_periph_manager_node {

class FootPressureSensor {
public:
    FootPressureSensor();
    ~FootPressureSensor();

    /* 禁用拷贝 */
    FootPressureSensor(const FootPressureSensor&) = delete;
    FootPressureSensor& operator=(const FootPressureSensor&) = delete;

    /*
     * 初始化传感器
     *
     * 打开串口并启动后台读线程。失败时 IsReady() 返回 false。
     *
     * @param uart_dev   串口设备路径, 如 "/dev/ttyS7"
     * @param baud_rate  波特率, 如 460800
     * @param timeout_ms 超时判定离线, 默认 10ms
     * @return true 成功, false 失败
     */
    bool Init(const char* uart_dev, int baud_rate, int timeout_ms);

    /*
     * 反初始化
     *
     * 停止后台线程并关闭串口。
     */
    void Deinit();

    /*
     * 读取最新足底压力数据 (非阻塞)
     *
     * 从 mutex 保护的缓存中读取, 不触发 I/O。
     * 硬件未初始化或离线时 out 清零。
     *
     * @param out [out] 足底压力数据结构体
     */
    void Read(foot_pressure_data_t* out) const;

    /*
     * 检查是否已成功初始化
     */
    bool IsReady() const { return m_ready.load(std::memory_order_acquire); }

    /*
     * 检查传感器是否在线
     *
     * timeout_ms 窗口内收到过有效帧则在线。
     */
    bool IsOnline() const;

    /*
     * 获取帧统计
     *
     * @param frames [out] 总成功帧数
     * @param errors [out] 总错误帧数
     */
    void GetStats(uint32_t* frames, uint32_t* errors) const;

private:
    /*
     * 串口读线程 — 阻塞 read + 环形缓冲 + 逐帧解析
     */
    void _ReaderThread();

    /*
     * 帧解析: 移植 BatteryFrame::Unpack 模式, 含累加和校验
     *
     * 搜索 0xF2 帧头 → 校验和 → 帧尾 0xF1 → 帧长 20 → SRC=0x01
     * 提取 6 个 uint16 大端数据并记录 timestamp_us
     *
     * @param buf      原始缓冲区
     * @param len      缓冲区有效数据长度
     * @param out      [输出] 解析成功后的足底压力数据
     * @param consumed [输出] 消耗的字节数, 0=数据不完整需继续接收
     * @return true=解析成功, false=数据不完整或帧无效
     */
    bool _ParseFrame(const uint8_t* buf, size_t len,
                     foot_pressure_data_t& out, size_t& consumed);

    int                  m_fd = -1;
    std::atomic<bool>    m_running{false};
    std::thread          m_thread;
    int                  m_timeout_ms = 10;

    /* 缓存保护 */
    mutable pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
    foot_pressure_data_t    m_cached;
    std::atomic<bool>       m_ready{false};

    /* 帧统计 */
    uint32_t m_frame_count     = 0;
    uint32_t m_error_count     = 0;

    /* FPS 统计: 每秒打印实际接收频率 + 最近一帧 AD 值 */
    uint32_t             m_fps_count      = 0;
    uint64_t             m_fps_last_sec   = 0;
    foot_pressure_data_t m_fps_last_frame;
};

} /* namespace stark_periph_manager_node */
