/**
 * @file emd_gaf_types.h
 * @brief IMU HAL — 公开数据类型定义
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#ifndef EMD_GAF_TYPES_H
#define EMD_GAF_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 9 轴融合输出
 *
 * 数据来源: ICM45608 eDMP GAF 引擎。
 * emd_gaf_get_output() 非阻塞读取。
 */
typedef struct {
    /** 9 轴四元数 (归一化到 [-1, 1]) */
    float quat_w, quat_x, quat_y, quat_z;
    /** 航向角 (度) */
    float heading_deg;
    /** 校准加速度 (g) */
    float accel_x, accel_y, accel_z;
    /** 校准角速度 (dps) */
    float gyro_x, gyro_y, gyro_z;
    /** 校准磁力计 (uT) */
    float mag_x, mag_y, mag_z;
    /** 温度 (°C) */
    float temp_c;
    /** 静止检测: 0 运动, 2 静止 */
    int   stationary;
    /** 陀螺仪零偏校准精度: 0 未校准, 3 精校准 */
    int   gyr_accuracy;
    /** 磁力计校准精度: 0 未校准, 3 精校准 */
    int   mag_accuracy;
    /** 时间戳 (us, CLOCK_MONOTONIC) */
    uint64_t timestamp_us;
} emd_output_t;

/**
 * @brief 原始 IMU 数据
 *
 * 未经 eDMP 融合的加速度计/陀螺仪原始值，
 * 采样率取决于工作模式 (200~800 Hz)。
 */
typedef struct {
    /** 原始加速度 (g) */
    float accel_x, accel_y, accel_z;
    /** 原始角速度 (dps) */
    float gyro_x, gyro_y, gyro_z;
    /** 时间戳 (us, CLOCK_MONOTONIC) */
    uint64_t timestamp_us;
} emd_imu_data_t;

/**
 * @brief 原始 IMU 传感器数据 (sensor ODR, 可达 800Hz)
 *
 * notify_raw_data 回调携带的单帧原始数据。
 * 不含四元数/磁力计等融合数据。
 */
typedef struct {
    float     accel_x, accel_y, accel_z;   /* g, 已校准 */
    float     gyro_x, gyro_y, gyro_z;      /* dps, 已校准 */
    float     temp_c;                      /* °C */
    uint64_t  timestamp_us;                /* us, CLOCK_MONOTONIC */
} emd_raw_sensor_t;

/**
 * @brief 原始数据回调 (sensor ODR)
 *
 * 在 sensor_event_cb 内调用, 每帧 IMU 数据触发一次。
 * 回调中应快速拷贝数据, 不要做耗时操作。
 */
typedef void (*emd_raw_data_cb_t)(const emd_raw_sensor_t *data, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* EMD_GAF_TYPES_H */
