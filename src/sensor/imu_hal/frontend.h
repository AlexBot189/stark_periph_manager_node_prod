/**
 * @file frontend.h
 * @brief eDMP GAF 前端数据结构 (Q30 格式)
 *
 * 提供兼容的:
 *   - INV_MSG 日志宏
 *   - MASK_NOTIFY 标志位
 *   - inv_edmp_gaf_outputs_t (应用层, Q30 定标)
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#ifndef FRONTEND_H
#define FRONTEND_H

#include <stdio.h>
#include <stdint.h>

/* Access to inv_imu_edmp_gaf_outputs_t data type */
#include "imu/inv_imu_edmp.h"

/* 日志级别 */
#define INV_MSG_LEVEL_ERROR   0
#define INV_MSG_LEVEL_WARNING 1
#define INV_MSG_LEVEL_INFO    2
#define INV_MSG_LEVEL_DEBUG   3
#define INV_MSG_LEVEL_VERBOSE 4

#define MSG_LEVEL INV_MSG_LEVEL_INFO

#define INV_MSG(level, fmt, ...) \
    fprintf(stderr, "[%d] " fmt "\n", level, ##__VA_ARGS__)

/* 数据就绪掩码 */
#define MASK_NOTIFY_RAW_ACC_DATA 0x01
#define MASK_NOTIFY_RAW_GYR_DATA 0x02

/* 前端功能配置位 */
#define FRONTEND_CONFIG_GAF      0x00000001
#define FRONTEND_CONFIG_SIF      0x00000002
#define FRONTEND_CONFIG_TAP      0x00000004
#define FRONTEND_CONFIG_FREEFALL 0x00000008
#define FRONTEND_CONFIG_B2S      0x00000010
#define FRONTEND_CONFIG_GAF_CFG  0x00000020
#define FRONTEND_CONFIG_GAF_RMAG 0x00000040

/*
 * 应用层 GAF 输出 (Q30 定标)。
 * 由 sensor_event_cb 从 inv_imu_edmp_gaf_outputs_t (Q14/Q12) 转换而来。
 */
typedef struct {
	int32_t grv_quat_q30[4];
	uint8_t grv_quat_valid;

	int32_t gmrv_quat_q30[4];
	int32_t gmrv_heading_q27;
	uint8_t gmrv_quat_valid;

	int32_t rv_quat_q30[4];
	int32_t rv_heading_q27;
	uint8_t rv_quat_valid;

	/* 校准加速度 (1g = 1<<16) */
	int32_t acc_cal_q16[3];
	uint8_t acc_cal_valid;

	/* 校准角速度 (1dps = 1<<16) */
	int32_t gyr_cal_q16[3];
	/* 陀螺零偏 (1dps = 1<<16) */
	int32_t gyr_bias_q16[3];
	/* 陀螺校准精度: 0 未校准 ~ 3 精校准 */
	int8_t  gyr_accuracy_flag;
	/* 静止检测 */
	int8_t  stationary_flag;
	uint8_t gyr_bias_valid;
	uint8_t gyr_flags_valid;

	int16_t raw_mag[3];
	uint8_t rmag_valid;

	/* 校准磁力计 (1uT = 1<<16) */
	int32_t mag_cal_q16[3];
	/* 磁力计零偏 (1uT = 1<<16) */
	int32_t mag_bias_q16[3];
	/* 磁力计校准精度: 0 未校准 ~ 3 精校准 */
	int8_t  mag_accuracy_flag;
	int8_t  mag_anomaly;
	uint8_t mag_bias_valid;

	inv_imu_auto_mrm_state_t mrm_state;
	uint8_t                  mrm_evt_chg_st;
	uint8_t                  mrm_evt_exe_mrm;
	uint8_t                  mrm_evt_exc_thr;
	uint8_t                  mrm_state_valid;

	/* 温度 (1°C = 1<<16) */
	int32_t temp_degC_q16;
	uint8_t temperature_valid;
} inv_edmp_gaf_outputs_t;

#endif /* FRONTEND_H */
