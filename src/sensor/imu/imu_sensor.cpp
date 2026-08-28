/*
 * stark_imu_sensor.cpp — IMU HAL 传感器实现
 *
 * 封装 libimu_hal.so (emd_gaf API)。
 * 硬件未接入时所有 Read() 返回零，不阻塞不崩溃。
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "sensor/imu/imu_sensor.h"

#include <cstring>
#include <cstdio>
#include <cmath>

extern "C" {
#include "emd_gaf.h"
}

namespace stark_periph_manager_node {

ImuHALSensor::ImuHALSensor()
{
}

ImuHALSensor::~ImuHALSensor()
{
    Deinit();
}

void ImuHALSensor::_RawDataCb(const emd_raw_sensor_t *data, void *user_data)
{
    ImuHALSensor *self = static_cast<ImuHALSensor*>(user_data);
    self->m_raw.store(*data);
}

void ImuHALSensor::_FusedDataCb(const emd_output_t *output, void *user_data)
{
    ImuHALSensor *self = static_cast<ImuHALSensor*>(user_data);
    self->m_fused.store(*output);
}

bool ImuHALSensor::Init(const ImuConfig& cfg)
{
    if (m_handle) {
        return true; /* 已初始化 */
    }

    /* 1. 创建实例 */
    m_handle = (struct emd_gaf*)emd_gaf_create();
    if (!m_handle) {
        fprintf(stderr, "[ImuHALSensor] emd_gaf_create() failed\n");
        return false;
    }

    /* 2. 初始化 (打开 I2C/SPI, 配置 GPIO, 载入 eDMP) */
    emd_gaf_cfg_t gaf_cfg;
    memset(&gaf_cfg, 0, sizeof(gaf_cfg));
    gaf_cfg.if_type      = (cfg.interface == "spi") ? EMD_GAF_IF_SPI : EMD_GAF_IF_I2C;
    gaf_cfg.i2c_dev      = cfg.i2c_dev.c_str();
    gaf_cfg.spi_dev      = cfg.spi_dev.c_str();
    gaf_cfg.spi_speed_hz = cfg.spi_speed_hz;
    gaf_cfg.spi_mode     = cfg.spi_mode;
    gaf_cfg.gpio_chip    = cfg.gpio_chip.c_str();
    gaf_cfg.gpio_line    = cfg.gpio_line;
    gaf_cfg.op_mode      = cfg.op_mode;
    gaf_cfg.mount_axis[0] = cfg.mount_axis[0];
    gaf_cfg.mount_axis[1] = cfg.mount_axis[1];
    gaf_cfg.mount_axis[2] = cfg.mount_axis[2];
    gaf_cfg.mount_sign[0] = cfg.mount_sign[0];
    gaf_cfg.mount_sign[1] = cfg.mount_sign[1];
    gaf_cfg.mount_sign[2] = cfg.mount_sign[2];

    const char* dev_name = (gaf_cfg.if_type == EMD_GAF_IF_SPI)
                         ? cfg.spi_dev.c_str() : cfg.i2c_dev.c_str();

    int ret = emd_gaf_init(m_handle, &gaf_cfg);
    if (ret != 0) {
        fprintf(stderr, "[ImuHALSensor] emd_gaf_init(%s, %s:%u, mode=%d) failed: %d\n",
                dev_name, cfg.gpio_chip.c_str(), cfg.gpio_line, cfg.op_mode, ret);
        emd_gaf_destroy((emd_gaf_t*)m_handle);
        m_handle = nullptr;
        return false;
    }

    /* 3. 启动后台采集线程 */
    ret = emd_gaf_start((emd_gaf_t*)m_handle);
    if (ret != 0) {
        fprintf(stderr, "[ImuHALSensor] emd_gaf_start() failed: %d\n", ret);
        emd_gaf_destroy((emd_gaf_t*)m_handle);
        m_handle = nullptr;
        return false;
    }

    /* 4. 注册原始数据回调 (notify_raw_data 等价, sensor ODR) */
    emd_gaf_set_raw_data_callback((emd_gaf_t*)m_handle, _RawDataCb, this);

    /* 5. 注册融合数据回调 (GAF ODR, frame_complete 时触发) */
    emd_gaf_set_fused_data_callback((emd_gaf_t*)m_handle, _FusedDataCb, this);

    printf("[ImuHALSensor] initialized: %s %s:%u mode=%d\n",
           dev_name, cfg.gpio_chip.c_str(), cfg.gpio_line, cfg.op_mode);
    return true;
}

void ImuHALSensor::Deinit()
{
    if (!m_handle) return;

    emd_gaf_stop((emd_gaf_t*)m_handle);
    emd_gaf_destroy((emd_gaf_t*)m_handle);
    m_handle = nullptr;

    printf("[ImuHALSensor] deinit done\n");
}

void ImuHALSensor::Read(imu_data_t* out) const
{
    if (!out) return;

    memset(out, 0, sizeof(imu_data_t));

    if (!m_handle) return;

    /* 原始 accel/gyro/temp (回调线程写, 顺序锁读) */
    emd_raw_sensor_t raw;
    m_raw.load(raw);
    out->acc_x       = raw.accel_x;
    out->acc_y       = raw.accel_y;
    out->acc_z       = raw.accel_z;
    out->gyro_x      = raw.gyro_x;
    out->gyro_y      = raw.gyro_y;
    out->gyro_z      = raw.gyro_z;
    out->temp_c      = raw.temp_c;
    out->timestamp_us = raw.timestamp_us;

    /* 融合数据 quat/mag/heading (回调线程写, 顺序锁读, 无新数据保留上次有效值) */
    emd_output_t imu;
    m_fused.load(imu);

    out->quat_w      = imu.quat_w;
    out->quat_x      = imu.quat_x;
    out->quat_y      = imu.quat_y;
    out->quat_z      = imu.quat_z;

    /* 四元数 -> 欧拉角 (ZYX: Yaw-Pitch-Roll, rad -> °) */
    {
        float qw = imu.quat_w, qx = imu.quat_x, qy = imu.quat_y, qz = imu.quat_z;
        out->yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * 57.29578f;
        float sp    = 2.0f*(qw*qy - qz*qx);
        if (sp > 1.0f) sp = 1.0f; else if (sp < -1.0f) sp = -1.0f;
        out->pitch = asinf(sp) * 57.29578f;
        out->roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 57.29578f;
    }
    out->mag_x       = imu.mag_x;
    out->mag_y       = imu.mag_y;
    out->mag_z       = imu.mag_z;
    out->heading_deg = imu.heading_deg;
    out->stationary  = imu.stationary;
    out->gyr_accuracy = imu.gyr_accuracy;
    out->mag_accuracy = imu.mag_accuracy;
}

} /* namespace stark_periph_manager_node */
