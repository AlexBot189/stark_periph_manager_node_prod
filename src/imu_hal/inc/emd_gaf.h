/**
 * @file emd_gaf.h
 * @brief IMU HAL — ICM45608 9轴融合动态库公共API
 *
 * libimu_hal.so 封装 InvenSense ICM45608 eDMP GAF (Gyro-Assisted Fusion)
 * 引擎的 Linux userspace 驱动，提供 9 轴 IMU 数据采集和融合。
 *
 * 线程模型:
 *   后台线程通过 GPIO 中断 + I2C FIFO 读取传感器数据，
 *   用户线程通过 get_output/get_imu 从缓存读取，不触发 I/O。
 *
 * 操作模式 (op_mode 0-9):
 *
 *   | 模式 | 描述                    | ODR    | 融合 |
 *   |------|-------------------------|--------|------|
 *   | 0    | HRC ALN GLN, MAG 100Hz  | 200Hz  | 否   |
 *   | 1    | HRC ALP GLP, MAG 50Hz   | 100Hz  | 否   |
 *   | 2    | HRC ALP, GYRO OFF       | 100Hz  | 否   |
 *   | 3    | ALN GLN, MAG 50Hz       | 400Hz  | 是   |
 *   | 4    | ALP GLP, MAG 50Hz       | 100Hz  | 是   |
 *   | 5    | ALN GLN, MAG 50Hz       | 100Hz  | 是   |
 *   | 6    | ALN GLN, MAG 50Hz       | 400Hz  | 是   |
 *   | 7    | ALN GLN, MAG 50Hz       | 800Hz  | 是   |
 *   | 8    | ALP GLP, MAG OFF        | 50Hz   | 是   |
 *   | 9    | ALP, GYRO OFF, MAG 50Hz | 100Hz  | 是   |
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#ifndef EMD_GAF_H
#define EMD_GAF_H

#include "emd_gaf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 不透明句柄 */
typedef struct emd_gaf emd_gaf_t;

/*
 * 生命周期
 */

/**
 * @brief 创建 IMU HAL 实例
 * @return 成功返回非 NULL，内存不足返回 NULL
 */
emd_gaf_t *emd_gaf_create(void);

/**
 * @brief 销毁实例，释放所有资源
 *
 * 自动停止后台线程（如已启动），释放 I2C/GPIO 资源，
 * 保存校准偏置到文件。
 *
 * @param handle 实例句柄
 */
void emd_gaf_destroy(emd_gaf_t *handle);

/**
 * @brief 初始化 IMU 并配置 HAL
 *
 * 打开 I2C 设备，配置 GPIO 中断线，初始化 ICM45608。
 * 必须在 emd_gaf_start() 之前调用。
 *
 * @param handle    实例句柄
 * @param i2c_dev   I2C 设备路径，如 "/dev/i2c-3"
 * @param gpio_chip GPIO 芯片名，如 "gpiochip4"
 * @param gpio_line GPIO 中断线编号
 * @param op_mode   操作模式 0-9
 * @return 0 成功，<0 失败
 */
int emd_gaf_init(emd_gaf_t *handle, const char *i2c_dev,
                 const char *gpio_chip, unsigned int gpio_line,
                 int op_mode);

/*
 * 采集控制
 */

/**
 * @brief 启动后台采集线程
 * @param handle 实例句柄
 * @return 0 成功，<0 失败
 */
int emd_gaf_start(emd_gaf_t *handle);

/**
 * @brief 停止后台采集线程，保存偏置
 * @param handle 实例句柄
 * @return 0 成功
 */
int emd_gaf_stop(emd_gaf_t *handle);

/*
 * 数据读取（非阻塞，线程安全）
 */

/**
 * @brief 获取最近一次 9 轴融合输出
 *
 * 从后台线程维护的缓存中拷贝，不触发 I/O。
 *
 * @param handle 实例句柄
 * @param output [out] 输出数据
 * @return 0 有新数据，1 无新数据，<0 错误
 */
int emd_gaf_get_output(emd_gaf_t *handle, emd_output_t *output);

/**
 * @brief 获取最近一次原始 IMU 数据
 *
 * 返回加速度计和陀螺仪原始数据（未融合）。
 *
 * @param handle 实例句柄
 * @param accel  [out] 加速度 (g)，可为 NULL
 * @param gyro   [out] 角速度 (dps)，可为 NULL
 * @return 0 成功，<0 错误
 */
int emd_gaf_get_imu(emd_gaf_t *handle, emd_imu_data_t *accel, emd_imu_data_t *gyro);

/**
 * @brief 注册原始传感器数据回调
 *
 * 在 emd_gaf_start 之前调用。回调在 sensor_event_cb 内触发,
 * 以 sensor ODR 速率提供已校准的 accel/gyro/temp 数据,
 * 不含四元数/磁力计等融合数据。
 *
 * @param handle    实例句柄
 * @param cb        回调函数指针, NULL 取消注册
 * @param user_data 用户自定义参数, 回调时透传
 */
void emd_gaf_set_raw_data_callback(emd_gaf_t *handle,
                                   emd_raw_data_cb_t cb, void *user_data);

/**
 * @brief 查询后台线程状态
 * @return 1 运行中，0 已停止
 */
int emd_gaf_is_running(emd_gaf_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* EMD_GAF_H */
