/*
 * barometer_sensor.h — BMP581 气压计 HAL 传感器封装
 *
 * 读取内核 input 驱动的 sysfs sensor_data
 *   ("Pressure: X Pa Temperature: Y deg C")。
 * 后台线程按 sample_period_ms 轮询, 解析 Pa→hPa、°C, 算海拔, 缓存。
 * 硬件未接入时 Read() 返回全零, 不阻塞, 不影响电机状态机。
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <atomic>
#include <thread>
#include <string>

#include "sensor/barometer/barometer_source.h"
#include "utils/seqlock.h"

namespace stark_periph_manager_node {

class BarometerSensor : public IBarometerSource {
public:
    BarometerSensor();
    ~BarometerSensor() override;

    /* 禁用拷贝 */
    BarometerSensor(const BarometerSensor&) = delete;
    BarometerSensor& operator=(const BarometerSensor&) = delete;

    /*
     * 初始化: 定位 sysfs 目录 → 写初始化序列 → 起后台线程
     *
     * 设备不存在时返回 false (上层跳过该设备, 不崩溃)。
     *
     * @param cfg 气压计配置
     * @return true 设备存在并启动成功
     */
    bool Init(const BarometerConfig& cfg) override;

    /*
     * 反初始化: 停止后台线程
     */
    void Deinit() override;

    /*
     * 读取最新数据 (非阻塞)
     *
     * 从顺序锁保护的缓存读取, 不触发 I/O。
     * 未就绪时 out 为全零。
     */
    void Read(barometer_data_t* out) const override;

    /* 是否已成功读到第一帧有效数据 */
    bool IsReady() const override { return m_ready.load(std::memory_order_acquire); }

private:
    /* 后台读取线程 */
    void _ReaderThread();

    /* 写 sysfs (初始化序列用) */
    bool _WriteSysfs(const std::string& path, const std::string& value);

    BarometerConfig m_cfg;
    std::string     m_sysfs_dir;   /* /sys/class/input/inputN (定位后) */

    std::atomic<bool> m_running{false};
    std::atomic<bool> m_ready{false};
    std::thread       m_thread;

    Seqlock<barometer_data_t> m_data;
};

}  /* namespace stark_periph_manager_node */
