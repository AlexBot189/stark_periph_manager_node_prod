/*
 * bhi360_imu_sensor.h — BHI360 IIO 传感器封装
 *
 * 通过 IIO sysfs 读取 BHI360+BMM350 的加速度/陀螺仪/磁力计/四元数/欧拉角数据。
 * 后台线程轮询 sysfs，解析后存入 Seqlock，RT 线程零锁读取。
 * 硬件未接入时 Read() 返回全零，不阻塞，不影响电机状态机。
 *
 * 数据来源: 内核 bhi360 IIO 驱动 (内核 .ko 已加载)
 * sysfs 路径: /sys/bus/iio/devices/iio:deviceN/
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include "sensor/imu/imu_source.h"
#include "sensor/imu/imu_mount.h"
#include "utils/seqlock.h"

#include <atomic>
#include <thread>
#include <string>
#include <cstdint>

namespace stark_periph_manager_node {

class Bhi360IioSensor : public IImuSource {
public:
    Bhi360IioSensor();
    ~Bhi360IioSensor() override;

    /* 禁用拷贝 */
    Bhi360IioSensor(const Bhi360IioSensor&) = delete;
    Bhi360IioSensor& operator=(const Bhi360IioSensor&) = delete;

    bool Init(const ImuConfig& cfg) override;
    void Deinit() override;
    void Read(imu_data_t* out) const override;
    bool IsReady() const override { return m_ready.load(std::memory_order_acquire); }

private:
    void _ReaderThread();
    bool _FindIioDevice(const std::string& name, std::string& out_path);
    bool _SysfsWrite(const std::string& path, const std::string& value);
    float _SysfsReadFloat(const std::string& path);
    int _SysfsReadInt(const std::string& path);

    std::string m_iio_path;           /* /sys/bus/iio/devices/iio:deviceN */
    uint32_t    m_sample_period_ms = 2;   /* 轮询周期 (默认 2ms = 500Hz) */

    /* 坐标轴映射 */
    int8_t m_mount_axis[3] = {0, 1, 2};  /* 默认单位矩阵 */
    int8_t m_mount_sign[3] = {1, 1, 1};

    std::atomic<bool> m_running{false};
    std::atomic<bool> m_ready{false};
    std::thread       m_thread;

    Seqlock<imu_data_t> m_data;
};

}  /* namespace stark_periph_manager_node */
