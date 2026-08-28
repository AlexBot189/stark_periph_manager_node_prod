/*
 * stark_imu_sensor.h — IMU HAL 传感器封装
 *
 * 封装 libimu_hal.so，提供 ICM45608 eDMP GAF 9轴融合数据。
 * 硬件未接入时 Read() 返回全零，不阻塞。
 *
 * 双通道数据:
 *   - accel/gyro/temp: 通过 notify_raw_data 回调以 sensor ODR 更新
 *   - quat/mag/heading: 通过 fused_data 回调以 GAF ODR 更新 (frame_complete)
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include "sensor/imu/imu_source.h"
#include "utils/seqlock.h"

struct emd_gaf; /* opaque, defined in emd_gaf.h */

#ifdef __cplusplus
extern "C" {
#endif
#include "emd_gaf_types.h"
#ifdef __cplusplus
}
#endif

namespace stark_periph_manager_node {

class ImuHALSensor : public IImuSource {
public:
    ImuHALSensor();
    ~ImuHALSensor() override;

    /* 禁用拷贝 */
    ImuHALSensor(const ImuHALSensor&) = delete;
    ImuHALSensor& operator=(const ImuHALSensor&) = delete;

    /*
     * 初始化 IMU HAL
     *
     * 创建 emd_gaf 实例并启动后台采集线程 + 融合数据轮询线程。
     * 失败时 handle 保持 NULL，Read() 返回全零，不影响系统运行。
     *
     * @param cfg IMU 配置 (driver/interface/gpio/op_mode 等)
     * @return true 成功，false 失败
     */
    bool Init(const ImuConfig& cfg) override;

    /*
     * 反初始化
     *
     * 停止后台线程并释放 HAL 资源。
     */
    void Deinit() override;

    /*
     * 读取最新融合数据 (非阻塞)
     *
     * 从顺序锁保护的缓存读取，不触发 I/O、不拿锁。
     * 硬件未初始化时 out 清零。
     *
     * @param out [out] IMU 数据结构体
     */
    void Read(imu_data_t* out) const override;

    /*
     * 检查是否已成功初始化
     */
    bool IsReady() const override { return m_handle != nullptr; }

private:
    emd_gaf* m_handle = nullptr; /* emd_gaf_t*, 不透明指针 */

    /* 原始数据缓存 (回调线程写, RT 线程读) */
    Seqlock<emd_raw_sensor_t> m_raw;

    /* 融合数据缓存 (回调线程写, RT 线程读) */
    Seqlock<emd_output_t> m_fused;

    static void _RawDataCb(const emd_raw_sensor_t *data, void *user_data);
    static void _FusedDataCb(const emd_output_t *output, void *user_data);
};

} /* namespace stark_periph_manager_node */
