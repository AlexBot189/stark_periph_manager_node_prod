/*
 * stark_imu_sensor.h — IMU HAL 传感器封装
 *
 * 封装 libimu_hal.so，提供 ICM45608 eDMP GAF 9轴融合数据。
 * 硬件未接入时 Read() 返回全零，不阻塞。
 *
 * 双通道数据:
 *   - accel/gyro/temp: 通过 notify_raw_data 回调以 sensor ODR 更新
 *   - quat/mag/heading: 通过 emd_gaf_get_output 以 GAF ODR 更新 (保留最近有效值)
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <pthread.h>
#include "stark_shm.h"

struct emd_gaf; /* opaque, defined in emd_gaf.h */

#ifdef __cplusplus
extern "C" {
#endif
#include "emd_gaf_types.h"
#ifdef __cplusplus
}
#endif

namespace stark_periph_manager_node {

class ImuHALSensor {
public:
    ImuHALSensor();
    ~ImuHALSensor();

    /* 禁用拷贝 */
    ImuHALSensor(const ImuHALSensor&) = delete;
    ImuHALSensor& operator=(const ImuHALSensor&) = delete;

    /*
     * 初始化 IMU HAL
     *
     * 创建 emd_gaf 实例并启动后台采集线程。
     * 失败时 handle 保持 NULL，Read() 返回全零，不影响系统运行。
     *
     * @param i2c_dev   I2C 设备路径，如 "/dev/i2c-3"
     * @param gpio_chip GPIO 芯片名，如 "gpiochip4"
     * @param gpio_line GPIO 中断线编号
     * @param op_mode   操作模式 0-9 (推荐 5: GAF 50Hz 融合)
     * @return true 成功，false 失败
     */
    bool Init(const char* i2c_dev, const char* gpio_chip,
              unsigned int gpio_line, int op_mode);

    /*
     * 反初始化
     *
     * 停止后台线程并释放 HAL 资源。
     */
    void Deinit();

    /*
     * 读取最新融合数据 (非阻塞)
     *
     * 从 libimu_hal 后台线程缓存中读取，不触发 I/O。
     * 硬件未初始化时 out 清零。
     *
     * @param out [out] IMU 数据结构体
     */
    void Read(imu_data_t* out) const;

    /*
     * 检查是否已成功初始化
     */
    bool IsReady() const { return m_handle != nullptr; }

private:
    emd_gaf* m_handle = nullptr; /* emd_gaf_t*, 不透明指针 */

    /* 原始数据回调缓冲 (sensor ODR, 回调线程写入, Read 线程读取) */
    mutable pthread_mutex_t m_raw_mutex;
    emd_raw_sensor_t        m_cached_raw;

    /* 融合数据缓存 (GAF ODR, 无新数据时保留上次有效值) */
    mutable pthread_mutex_t m_fused_mutex;
    mutable emd_output_t    m_cached_fused;
    mutable bool            m_fused_valid = false;

    static void _RawDataCb(const emd_raw_sensor_t *data, void *user_data);
};

} /* namespace stark_periph_manager_node */
