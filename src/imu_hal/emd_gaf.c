/**
 * @file emd_gaf.c
 * @brief IMU HAL — libimu_hal.so 核心实现
 *
 * 封装 ICM45608 eDMP GAF 引擎:
 *   - 后台线程自动采集 (gpio poll + FIFO read + decode)
 *   - 输出缓存 (pthread_mutex 保护)
 *   - 非阻塞 get_output / get_imu API
 *
 * Copyright (c) 2026 zhiqiang.yang
 */

#include "emd_gaf.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <math.h>

/* eMD IMU Drivers */
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"

/* Magnetometer drivers */
#include "invn_mag.h"

/* HAL */
#include "system_interface.h"

/* Frontend */
#include "frontend.h"

/* 常量定义 */

/*
 * IMU 坐标系变换开关. 芯片 eDMP 按传感器物理坐标输出融合数据,
 * 本机需转到标准机器人坐标(前-左-上 FLU, 右手系). 换硬件朝向时改此宏或 _remap_* 公式.
 * 1=启用变换, 0=保持芯片原始坐标.
 */
#define EMD_IMU_AXIS_REMAP 0

/*
 * 芯片绕 X 轴 +90 安装 + 转 FLU. 异于 REMAP 的 Rz(-90).
 * 变换公式: rx=-cz, ry=-cx, rz=+cy (右手系 FLU).
 * 1=启用, 0=关闭. 与 EMD_IMU_AXIS_REMAP 互斥: 只开一个.
 */
#define EMD_IMU_AXIS_REMAP_2 1

#define SERIF_TYPE UI_I2C

#define ACCEL_FSR_ENUM (ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G)
#define ACCEL_FSR_G    (4)
#define RAW_ACC_SCALE  (ACCEL_FSR_G * 2)

/* High FSR not supported by ICM45608, use ±2000dps */
#define GYRO_FSR_ENUM (GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS)
#define GYRO_FSR_DPS  (2000)
#define RAW_GYR_SCALE    (GYRO_FSR_DPS * 2)
#define RAW_GYR_SCALE_HR (4000 / 8)

#define RAW_MAG_SCALE (4915)

#define DEFAULT_WOM_THS_MG (52 >> 2)

#define LOW_POWER_MODE 0
#define LOW_NOISE_MODE 1

#define MASK_NOTIFY_RAW_ACC_DATA 0x01
#define MASK_NOTIFY_RAW_GYR_DATA 0x02

#define DEFAULT_I2C_DEV   "/dev/i2c-3"
#define DEFAULT_GPIO_CHIP "gpiochip4"
#define DEFAULT_GPIO_LINE 2
#define DEFAULT_IMU_ADDR  0x68

/* 操作模式参数结构 */

typedef struct op_mode {
    uint32_t dmp_sensor_odr_us;
    uint32_t gaf_pdr_us;
    uint8_t  fusion_enabled;
    struct {
        pwr_mgmt0_accel_mode_t pm;
        union { uint32_t bw; uint32_t avg; } lpf;
    } acc;
    struct {
        pwr_mgmt0_gyro_mode_t pm;
        union { uint32_t bw; uint32_t avg; } lpf;
    } gyr;
    struct {
        uint8_t  is_on;
        uint32_t mag_odr_us;
    } mag;
} op_mode_t;

static const op_mode_t supported_cfg[] = {
    { /* 0: HRC ALN GLN BYPASS 200Hz MAG 100Hz HRC 100Hz */
        .dmp_sensor_odr_us = 5000,  .gaf_pdr_us = 10000, .fusion_enabled = 0,
        .acc = { PWR_MGMT0_ACCEL_MODE_LN, .lpf = { .bw = 0 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LN,  .lpf = { .bw = 0 } },
        .mag = { 1, 10000 },
    },
    { /* 1: HRC ALP GLP AVG 1x 100Hz MAG 50Hz HRC 50Hz */
        .dmp_sensor_odr_us = 10000, .gaf_pdr_us = 20000, .fusion_enabled = 0,
        .acc = { PWR_MGMT0_ACCEL_MODE_LP, .lpf = { .avg = 1 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LP,  .lpf = { .avg = 1 } },
        .mag = { 1, 20000 },
    },
    { /* 2: HRC ALP AVG 1x 100Hz GYRO OFF MAG 50Hz HRC 50Hz */
        .dmp_sensor_odr_us = 10000, .gaf_pdr_us = 20000, .fusion_enabled = 0,
        .acc = { PWR_MGMT0_ACCEL_MODE_LP, .lpf = { .avg = 1 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_OFF, },
        .mag = { 1, 20000 },
    },
    { /* 3: ALN GLN BYPASS 400Hz MAG 50Hz GAF 200Hz */
        .dmp_sensor_odr_us = 2500,  .gaf_pdr_us = 5000,  .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LN, .lpf = { .bw = 0 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LN,  .lpf = { .bw = 0 } },
        .mag = { 1, 20000 },
    },
    { /* 4: ALP GLP AVG 1x 100Hz MAG 50Hz GAF 50Hz */
        .dmp_sensor_odr_us = 10000, .gaf_pdr_us = 20000, .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LP, .lpf = { .avg = 1 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LP,  .lpf = { .avg = 1 } },
        .mag = { 1, 20000 },
    },
    { /* 5: ALN GLN BYPASS 100Hz MAG 50Hz GAF 50Hz */
        .dmp_sensor_odr_us = 10000, .gaf_pdr_us = 20000, .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LN, .lpf = { .bw = 0 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LN,  .lpf = { .bw = 0 } },
        .mag = { 1, 20000 },
    },
    { /* 6: ALN GLN BYPASS 400Hz MAG 50Hz GAF 50Hz */
        .dmp_sensor_odr_us = 2500,  .gaf_pdr_us = 20000, .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LN, .lpf = { .bw = 0 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LN,  .lpf = { .bw = 0 } },
        .mag = { 1, 20000 },
    },
    { /* 7: ALN GLN BYPASS 800Hz MAG 50Hz GAF 50Hz */
        .dmp_sensor_odr_us = 1250,  .gaf_pdr_us = 20000, .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LN, .lpf = { .bw = 0 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LN,  .lpf = { .bw = 0 } },
        .mag = { 1, 20000 },
    },
    { /* 8: ALP GLP AVG 1x 50Hz MAG OFF GAF 50Hz */
        .dmp_sensor_odr_us = 20000, .gaf_pdr_us = 20000, .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LP, .lpf = { .avg = 1 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_LP,  .lpf = { .avg = 1 } },
        .mag = { 0, },
    },
    { /* 9: ALP AVG 1x 100Hz GYRO OFF MAG 50Hz GAF 50Hz */
        .dmp_sensor_odr_us = 10000, .gaf_pdr_us = 20000, .fusion_enabled = 1,
        .acc = { PWR_MGMT0_ACCEL_MODE_LP, .lpf = { .avg = 1 } },
        .gyr = { PWR_MGMT0_GYRO_MODE_OFF, },
        .mag = { 1, 20000 },
    },
};
#define NUM_OP_MODES (sizeof(supported_cfg) / sizeof(supported_cfg[0]))

/* 内部结构体 (opaque handle) */

struct emd_gaf {
    /* 初始化参数 */
    char     i2c_dev[64];
    char     gpio_chip[64];
    unsigned int gpio_line;
    uint8_t  imu_addr;
    uint8_t  op_mode_idx;
    int      initialized;

    /* IMU 设备 */
    inv_imu_device_t imu_dev;

    /* 中断 / 时间戳 */
    volatile int      int1_flag;
    volatile uint64_t int1_timestamp;
    uint64_t          timestamp;

    /* 当前操作模式 */
    uint32_t dmp_odr_us;
    uint32_t mag_odr_us;
    uint32_t gaf_pdr_us;
    uint8_t  fusion_enabled;
    int      power_save_en;

    /* 安装矩阵 / 软铁矩阵 */
    int8_t  mounting_matrix[9];
    int32_t soft_iron_matrix[3][3];

    /* FIFO */
    uint8_t fifo_data[FIFO_MIRRORING_SIZE];

    /* 传感器状态 */
    uint8_t accel_en;
    uint8_t gyro_en;
    int32_t acc_bias_q16[3];
    uint8_t gyro_is_on;
    uint8_t mag_is_on;

    /* 高分辨率 / MRM / FIFO push */
    uint32_t high_res_en;
    uint32_t mrm_auto_is_on;
    uint8_t  fifo_push_en;

    /* MRM 事件 */
    inv_imu_edmp_int_state_t mrm_event;

    /* 偏置 / 精度 */
    int16_t gyr_bias_q12[3];
    uint8_t gyr_accuracy;
    int32_t gyr_bias_temperature;
    int32_t mag_bias_q16[3];
    uint8_t mag_accuracy;
    int32_t freeze_mag_bias;
    int32_t frozen_bias_mag[3];

    /* 融合输出 */
    inv_edmp_gaf_outputs_t edmp_outputs;

    /* 后台线程 */
    volatile int    running;
    pthread_t       thread;
    pthread_mutex_t output_mutex;

    /* 输出缓存 */
    emd_output_t  cached_output;
    emd_imu_data_t cached_accel;
    emd_imu_data_t cached_gyro;
    int           output_updated;
    int           imu_updated;

    /* 原始数据回调 (notify_raw_data 等价) */
    emd_raw_data_cb_t raw_data_cb;
    void              *raw_data_user;
};

/* 内部函数声明 */

static int  _setup_imu(emd_gaf_t *g);
static int  _stop_algo(emd_gaf_t *g);
static int  _set_operation_mode(emd_gaf_t *g, const op_mode_t *op_mode);
static int  _start_algo(emd_gaf_t *g);
static void _sensor_event_cb(inv_imu_sensor_event_t *event);
static void _int_cb(void *context, unsigned int int_num);
static int  _init_imu_biases(emd_gaf_t *g);
static void _convert_output(const inv_edmp_gaf_outputs_t *in, uint64_t ts, emd_output_t *out);
static void *_thread_main(void *arg);

/* 回调需要访问的当前实例 (单实例场景) */
static emd_gaf_t *g_active_instance = NULL;

#if EMD_IMU_AXIS_REMAP
/*
 * 坐标系变换: 传感器 -> 标准机器人 FLU(前-左-上), 绕Z轴 Rz(-90).
 *   前 = +Y_sensor,  左 = -X_sensor,  上 = Z_sensor
 * 静态标定: 某轴朝天读 +1g, 朝地读 -1g (比力约定).
 * 加速度/角速度/磁力计同一套变换.
 */
static inline void _remap_vec(float *x, float *y, float *z)
{
    float sx = *x, sy = *y;
    *x =  sy;   /* 前 = +Y_sensor */
    *y = -sx;   /* 左 = -X_sensor */
    (void)z;    /* Z 不变 */
}

/*
 * 四元数变换: 与向量同一旋转 Rz(-90), 右乘 q_r=(sqrt2/2, 0, 0, sqrt2/2).
 * 注意: eDMP GAF 的 rv_quat convention 为芯片黑盒, 若实测 roll/pitch/yaw
 * 符号不对, 改为左乘或取共轭.
 */
static void _remap_quat(float *w, float *x, float *y, float *z)
{
    const float s = 0.70710678f;
    float qw = *w, qx = *x, qy = *y, qz = *z;
    *w = s * (qw - qz);
    *x = s * (qx + qy);
    *y = s * (qy - qx);
    *z = s * (qw + qz);
}
#endif

#if EMD_IMU_AXIS_REMAP_2
/*
 * 坐标系变换: 芯片绕 X 轴 +90 安装, 转标准 FLU.
 *   前 = -Z_sensor,  左 = -X_sensor,  上 = +Y_sensor
 * 静态标定: 某轴朝天读 +1g, 朝地读 -1g (比力约定).
 */
static inline void _remap_vec_2(float *x, float *y, float *z)
{
    float sx = *x, sy = *y, sz = *z;
    *x = -sz;   /* 前 = -Z_sensor */
    *y = -sx;   /* 左 = -X_sensor */
    *z =  sy;   /* 上 = +Y_sensor */
}

/*
 * 四元数变换: 旋转矩阵 R=[0,0,-1; -1,0,0; 0,1,0] -> q_R=[0.5,0.5,-0.5,-0.5]
 * q_robot = q_chip    q_R^(-1) = q_chip    [0.5, -0.5, 0.5, 0.5]
 * 展开: *w=0.5*(qw+qx-qy-qz)  *x=0.5*(-qw+qx+qy-qz)
 *       *y=0.5*(qw-qx+qy-qz)  *z=0.5*(qw+qx+qy+qz)
 */
static void _remap_quat_2(float *w, float *x, float *y, float *z)
{
    const float s = 0.5f;
    float qw = *w, qx = *x, qy = *y, qz = *z;
    *w = s * ( qw + qx - qy - qz);
    *x = s * (-qw + qx + qy - qz);
    *y = s * ( qw - qx + qy - qz);
    *z = s * ( qw + qx + qy + qz);
}
#endif

/*
 * 公共 API — 生命周期
 */

emd_gaf_t *emd_gaf_create(void)
{
    emd_gaf_t *g = (emd_gaf_t *)calloc(1, sizeof(emd_gaf_t));
    if (!g) return NULL;

    pthread_mutex_init(&g->output_mutex, NULL);

    /* 默认值 */
    strncpy(g->i2c_dev, DEFAULT_I2C_DEV, sizeof(g->i2c_dev) - 1);
    strncpy(g->gpio_chip, DEFAULT_GPIO_CHIP, sizeof(g->gpio_chip) - 1);
    g->gpio_line = DEFAULT_GPIO_LINE;
    g->imu_addr  = DEFAULT_IMU_ADDR;
    g->op_mode_idx = 0;

    /* 单位安装矩阵 */
    g->mounting_matrix[0] = 1; g->mounting_matrix[1] = 0; g->mounting_matrix[2] = 0;
    g->mounting_matrix[3] = 0; g->mounting_matrix[4] = 1; g->mounting_matrix[5] = 0;
    g->mounting_matrix[6] = 0; g->mounting_matrix[7] = 0; g->mounting_matrix[8] = 1;

    /* 单位软铁矩阵 (Q30) */
    g->soft_iron_matrix[0][0] = (1 << 30); g->soft_iron_matrix[0][1] = 0; g->soft_iron_matrix[0][2] = 0;
    g->soft_iron_matrix[1][0] = 0; g->soft_iron_matrix[1][1] = (1 << 30); g->soft_iron_matrix[1][2] = 0;
    g->soft_iron_matrix[2][0] = 0; g->soft_iron_matrix[2][1] = 0; g->soft_iron_matrix[2][2] = (1 << 30);

    g->high_res_en    = 1;
    g->mrm_auto_is_on = 1;
    g->fifo_push_en   = 1;
    g->freeze_mag_bias = 0;

    g->running = 0;

    return g;
}

void emd_gaf_destroy(emd_gaf_t *handle)
{
    if (!handle) return;

    /* 确保线程已停止 */
    if (g_active_instance == handle) {
        emd_gaf_stop(handle);
    }

    pthread_mutex_destroy(&handle->output_mutex);
    free(handle);
}

int emd_gaf_init(emd_gaf_t *handle, const char *i2c_dev,
                 const char *gpio_chip, unsigned int gpio_line,
                 int op_mode)
{
    if (!handle) return -1;
    if (op_mode < 0 || (size_t)op_mode >= NUM_OP_MODES) return -1;

    int rc = 0;

    /* 保存参数 */
    strncpy(handle->i2c_dev, i2c_dev, sizeof(handle->i2c_dev) - 1);
    strncpy(handle->gpio_chip, gpio_chip, sizeof(handle->gpio_chip) - 1);
    handle->gpio_line   = gpio_line;
    handle->op_mode_idx = (uint8_t)op_mode;

    /* 重置状态 */
    handle->accel_en = 0;
    handle->gyro_en  = 0;
    handle->power_save_en = 0;
    handle->acc_bias_q16[0] = 0;
    handle->acc_bias_q16[1] = 0;
    handle->acc_bias_q16[2] = 0;

    memset(&handle->mrm_event, 0, sizeof(handle->mrm_event));
    memset(handle->mag_bias_q16, 0, sizeof(handle->mag_bias_q16));

    /* 从 NVM 恢复偏置 */
    rc |= _init_imu_biases(handle);

    handle->edmp_outputs.gyr_bias_q16[0]   = handle->gyr_bias_q12[0] << 4;
    handle->edmp_outputs.gyr_bias_q16[1]   = handle->gyr_bias_q12[1] << 4;
    handle->edmp_outputs.gyr_bias_q16[2]   = handle->gyr_bias_q12[2] << 4;
    handle->edmp_outputs.gyr_accuracy_flag = handle->gyr_accuracy;
    handle->edmp_outputs.mag_bias_q16[0]   = handle->mag_bias_q16[0];
    handle->edmp_outputs.mag_bias_q16[1]   = handle->mag_bias_q16[1];
    handle->edmp_outputs.mag_bias_q16[2]   = handle->mag_bias_q16[2];
    handle->edmp_outputs.mag_accuracy_flag = handle->mag_accuracy;
    SI_CHECK_RC(rc);

    /* 1. Init HAL */
    rc |= emd_hal_init(handle->i2c_dev, handle->imu_addr,
                       handle->gpio_chip, handle->gpio_line);
    SI_CHECK_RC(rc);

    /* 2. 配置 GPIO 中断回调 */
    g_active_instance = handle;
    rc |= emd_hal_gpio_init(1, (emd_gpio_cb_t)_int_cb);
    SI_CHECK_RC(rc);

    /* 3. 初始化 IMU */
    rc |= _setup_imu(handle);
    SI_CHECK_RC(rc);

    /* 4. 配置传感器 + 启动算法 */
    rc |= _set_operation_mode(handle, &supported_cfg[handle->op_mode_idx]);
    rc |= _start_algo(handle);
    SI_CHECK_RC(rc);

    /* 重置中断标志 */
    handle->int1_flag      = 0;
    handle->int1_timestamp = 0;

    handle->initialized = 1;
    return rc;
}

/*
 * 公共 API — 采集控制
 */

int emd_gaf_start(emd_gaf_t *handle)
{
    if (!handle || !handle->initialized) return -1;
    if (handle->running) return 0;

    handle->running = 1;
    int rc = pthread_create(&handle->thread, NULL, _thread_main, handle);
    if (rc != 0) {
        handle->running = 0;
        return -1;
    }
    return 0;
}

int emd_gaf_stop(emd_gaf_t *handle)
{
    if (!handle) return -1;
    if (!handle->running) return 0;

    handle->running = 0;
    pthread_join(handle->thread, NULL);

    _stop_algo(handle);
    emd_hal_deinit();

    g_active_instance = NULL;
    handle->initialized = 0;

    return 0;
}

/*
 * 公共 API — 数据读取
 */

int emd_gaf_get_output(emd_gaf_t *handle, emd_output_t *output)
{
    if (!handle || !output) return -1;

    pthread_mutex_lock(&handle->output_mutex);
    memcpy(output, &handle->cached_output, sizeof(emd_output_t));
    int updated = handle->output_updated;
    handle->output_updated = 0;
    pthread_mutex_unlock(&handle->output_mutex);

    return updated ? 0 : 1;
}

int emd_gaf_get_imu(emd_gaf_t *handle, emd_imu_data_t *accel, emd_imu_data_t *gyro)
{
    if (!handle) return -1;

    pthread_mutex_lock(&handle->output_mutex);
    if (accel) memcpy(accel, &handle->cached_accel, sizeof(emd_imu_data_t));
    if (gyro)  memcpy(gyro,  &handle->cached_gyro,  sizeof(emd_imu_data_t));
    int updated = handle->imu_updated;
    handle->imu_updated = 0;
    pthread_mutex_unlock(&handle->output_mutex);

    return updated ? 0 : 1;
}

int emd_gaf_is_running(emd_gaf_t *handle)
{
    if (!handle) return 0;
    return handle->running;
}

void emd_gaf_set_raw_data_callback(emd_gaf_t *handle,
                                   emd_raw_data_cb_t cb, void *user_data)
{
    if (!handle) return;
    handle->raw_data_cb   = cb;
    handle->raw_data_user = user_data;
}

/*
 * 后台线程
 */

static void *_thread_main(void *arg)
{
    emd_gaf_t *g = (emd_gaf_t *)arg;
    int rc = 0;

    /* 设置 RT 调度 (SCHED_FIFO 50), 低于 stark RT 线程的 90, 保证可抢占 */
    {
        struct sched_param param;
        param.sched_priority = 50;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            fprintf(stderr, "[W] IMU HAL: SCHED_FIFO 50 failed (need root/CAP_SYS_NICE)\n");
        }
    }

    fprintf(stderr, "[I] IMU HAL background thread started (mode=%d)\n", g->op_mode_idx);

    do {
        int ret = emd_hal_gpio_wait(100);
        if (ret < 0) break;
        if (ret == 0) continue;

        if (g->int1_flag) {
            inv_imu_int_state_t int_state;

            si_disable_irq();
            g->int1_flag = 0;
            g->timestamp = g->int1_timestamp;
            si_enable_irq();

            rc |= inv_imu_get_int_status(&g->imu_dev, INV_IMU_INT1, &int_state);
            if (rc) break;

            if (int_state.INV_EDMP_EVENT) {
                inv_imu_edmp_int_state_t apex_state = { 0 };
                rc |= inv_imu_edmp_get_int_apex_status(&g->imu_dev, &apex_state);
                if (rc) break;
                g->mrm_event.INV_GAF_MRM_CHG = apex_state.INV_GAF_MRM_CHG;
                g->mrm_event.INV_GAF_MRM_RUN = apex_state.INV_GAF_MRM_RUN;
                g->mrm_event.INV_GAF_MRM_THR = apex_state.INV_GAF_MRM_THR;
            }

            if (int_state.INV_FIFO_THS) {
                uint16_t fifo_count;
                rc |= inv_imu_adv_get_data_from_fifo(&g->imu_dev, g->fifo_data, &fifo_count);
                if (rc) break;
                rc |= inv_imu_adv_parse_fifo_data(&g->imu_dev, g->fifo_data, fifo_count);
                if (rc) break;
            }
        }
    } while (rc == 0 && g->running);

    fprintf(stderr, "[I] IMU HAL background thread exiting (rc=%d)\n", rc);
    return NULL;
}

/*
 * IMU 初始化
 */

static int _setup_imu(emd_gaf_t *g)
{
    int rc = 0;
    inv_imu_int_pin_config_t  int_pin_config;
    inv_imu_adv_var_t        *e = (inv_imu_adv_var_t *)g->imu_dev.adv_var;
    inv_imu_int_state_t       int_config;
    inv_imu_adv_fifo_config_t fifo_config;

    /* 传输层 */
    g->imu_dev.transport.read_reg   = si_io_imu_read_reg;
    g->imu_dev.transport.write_reg  = si_io_imu_write_reg;
    g->imu_dev.transport.serif_type = SERIF_TYPE;
    g->imu_dev.transport.sleep_us   = si_sleep_us;

    /* 传感器事件回调 */
    e->sensor_event_cb = _sensor_event_cb;

    si_sleep_us(3000);

    rc |= inv_imu_adv_init(&g->imu_dev);
    SI_CHECK_RC(rc);

    /* 配置中断引脚 */
    int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
    int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
    rc |= inv_imu_set_pin_config_int(&g->imu_dev, INV_IMU_INT1, &int_pin_config);
    SI_CHECK_RC(rc);

    /* 设置传感器量程 */
    rc |= inv_imu_set_accel_fsr(&g->imu_dev, ACCEL_FSR_ENUM);
    rc |= inv_imu_set_gyro_fsr(&g->imu_dev, GYRO_FSR_ENUM);
    SI_CHECK_RC(rc);

    /* 自动 MRM 中断配置 */
    if (g->mrm_auto_is_on) {
        inv_imu_edmp_int_state_t apex_int_config;
        memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));
        apex_int_config.INV_GAF_MRM_CHG = INV_IMU_ENABLE;
        apex_int_config.INV_GAF_MRM_RUN = INV_IMU_ENABLE;
        apex_int_config.INV_GAF_MRM_THR = INV_IMU_ENABLE;
        rc |= inv_imu_edmp_set_config_int_apex(&g->imu_dev, &apex_int_config);
    }

    /* 中断: FIFO + EDMP */
    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_FIFO_THS = INV_IMU_ENABLE;
    if (g->mrm_auto_is_on)
        int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int(&g->imu_dev, INV_IMU_INT1, &int_config);
    SI_CHECK_RC(rc);

    /* FIFO 配置 */
    rc |= inv_imu_adv_get_fifo_config(&g->imu_dev, &fifo_config);
    fifo_config.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
    fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
    fifo_config.base_conf.fifo_wm_th = 1;
    fifo_config.base_conf.hires_en   = INV_IMU_DISABLE;
    fifo_config.base_conf.gyro_en    = INV_IMU_ENABLE;
    fifo_config.base_conf.accel_en   = INV_IMU_ENABLE;
    fifo_config.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
    fifo_config.es1_en               = INV_IMU_ENABLE;
    fifo_config.es0_en               = INV_IMU_ENABLE;
    fifo_config.comp_en              = INV_IMU_DISABLE;
    fifo_config.tmst_fsync_en        = INV_IMU_ENABLE;
    fifo_config.es0_6b_9b            = FIFO_CONFIG4_FIFO_ES0_9B;
    rc |= inv_imu_adv_set_fifo_config(&g->imu_dev, &fifo_config);
    SI_CHECK_RC(rc);

    return rc;
}

/*
 * 停止算法
 */

static int _stop_algo(emd_gaf_t *g)
{
    int rc = 0;

    rc |= inv_imu_edmp_disable(&g->imu_dev);

    /* 获取最新计算的偏置 */
    rc |= inv_imu_edmp_get_gaf_gyr_bias(&g->imu_dev, g->gyr_bias_q12,
                                         &g->gyr_bias_temperature, &g->gyr_accuracy);
    rc |= inv_imu_edmp_get_gaf_mag_bias(&g->imu_dev, g->mag_bias_q16, &g->mag_accuracy);

    return rc;
}

/*
 * 设置操作模式
 */

static int _set_operation_mode(emd_gaf_t *g, const op_mode_t *op_mode)
{
    int rc = 0;

    g->dmp_odr_us     = op_mode->dmp_sensor_odr_us;
    g->gaf_pdr_us     = op_mode->gaf_pdr_us;
    g->mag_odr_us     = op_mode->mag.mag_odr_us;
    g->fusion_enabled = op_mode->fusion_enabled;
    g->gyro_is_on     = op_mode->gyr.pm == PWR_MGMT0_GYRO_MODE_OFF ? 0 : 1;
    g->mag_is_on      = op_mode->mag.is_on;

    rc |= inv_imu_set_accel_mode(&g->imu_dev, PWR_MGMT0_ACCEL_MODE_OFF);
    rc |= inv_imu_set_gyro_mode(&g->imu_dev, PWR_MGMT0_GYRO_MODE_OFF);
    /* 清空 FIFO */
    rc |= inv_imu_flush_fifo(&g->imu_dev);
    g->int1_timestamp = 0;
    g->int1_flag      = 0;
    SI_CHECK_RC(rc);

    if (g->mag_is_on) {
        if (invn_mag_init(&g->imu_dev))
            g->mag_is_on = 0;
    }

    switch (g->dmp_odr_us) {
    case 20 * 1000:
        rc |= inv_imu_edmp_set_frequency(&g->imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ);
        rc |= inv_imu_set_accel_frequency(&g->imu_dev, ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
        rc |= inv_imu_set_gyro_frequency(&g->imu_dev, GYRO_CONFIG0_GYRO_ODR_50_HZ);
        break;
    case 10 * 1000:
        rc |= inv_imu_edmp_set_frequency(&g->imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ);
        rc |= inv_imu_set_accel_frequency(&g->imu_dev, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
        rc |= inv_imu_set_gyro_frequency(&g->imu_dev, GYRO_CONFIG0_GYRO_ODR_100_HZ);
        break;
    case 5 * 1000:
        rc |= inv_imu_edmp_set_frequency(&g->imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ);
        rc |= inv_imu_set_accel_frequency(&g->imu_dev, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
        rc |= inv_imu_set_gyro_frequency(&g->imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
        break;
    case 2500:
        rc |= inv_imu_edmp_set_frequency(&g->imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ);
        rc |= inv_imu_set_accel_frequency(&g->imu_dev, ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
        rc |= inv_imu_set_gyro_frequency(&g->imu_dev, GYRO_CONFIG0_GYRO_ODR_400_HZ);
        break;
    case 1250:
        rc |= inv_imu_edmp_set_frequency(&g->imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ);
        rc |= inv_imu_set_accel_frequency(&g->imu_dev, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
        rc |= inv_imu_set_gyro_frequency(&g->imu_dev, GYRO_CONFIG0_GYRO_ODR_800_HZ);
        break;
    default:
        fprintf(stderr, "[E] Unknown dmp_odr_us %d, force to default 200Hz\n", (int)g->dmp_odr_us);
        g->dmp_odr_us = 5 * 1000;
        rc |= inv_imu_edmp_set_frequency(&g->imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ);
        rc |= inv_imu_set_accel_frequency(&g->imu_dev, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
        rc |= inv_imu_set_gyro_frequency(&g->imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
        break;
    }

    switch (op_mode->acc.lpf.bw) {
    case 0:   rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER); break;
    case 4:   rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4); break;
    case 8:   rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8); break;
    case 16:  rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16); break;
    case 32:  rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32); break;
    case 64:  rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_64); break;
    case 128: rc |= inv_imu_set_accel_ln_bw(&g->imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_128); break;
    default: break;
    }
    switch (op_mode->gyr.lpf.bw) {
    case 0:   rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER); break;
    case 4:   rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4); break;
    case 8:   rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8); break;
    case 16:  rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_16); break;
    case 32:  rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_32); break;
    case 64:  rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_64); break;
    case 128: rc |= inv_imu_set_gyro_ln_bw(&g->imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_128); break;
    default: break;
    }

    switch (op_mode->acc.lpf.avg) {
    case 1: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1); break;
    case 2: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_2); break;
    case 4: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_4); break;
    case 5: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_5); break;
    case 7: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_7); break;
    case 8: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_8); break;
    case 10: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_10); break;
    case 11: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_11); break;
    case 16: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_16); break;
    case 18: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_18); break;
    case 20: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_20); break;
    case 32: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_32); break;
    case 64: rc |= inv_imu_set_accel_lp_avg(&g->imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_64); break;
    default: break;
    }
    switch (op_mode->gyr.lpf.avg) {
    case 1: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_1); break;
    case 2: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_2); break;
    case 4: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_4); break;
    case 5: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_5); break;
    case 7: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_7); break;
    case 8: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_8); break;
    case 10: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_10); break;
    case 11: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_11); break;
    case 16: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_16); break;
    case 18: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_18); break;
    case 20: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_20); break;
    case 32: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_32); break;
    case 64: rc |= inv_imu_set_gyro_lp_avg(&g->imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_64); break;
    default: break;
    }

    /* 强制时钟配置以满足 I2C master 需求 */
    if ((op_mode->acc.pm == PWR_MGMT0_ACCEL_MODE_LP) && (g->gyro_is_on == 0))
        rc |= inv_imu_select_accel_lp_clk(&g->imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
    else
        rc |= inv_imu_select_accel_lp_clk(&g->imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);

    if (op_mode->acc.pm == PWR_MGMT0_ACCEL_MODE_LN)
        rc |= inv_imu_set_accel_mode(&g->imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    else
        rc |= inv_imu_set_accel_mode(&g->imu_dev, PWR_MGMT0_ACCEL_MODE_LP);

    if (g->gyro_is_on) {
        if (op_mode->gyr.pm == PWR_MGMT0_GYRO_MODE_LN)
            rc |= inv_imu_set_gyro_mode(&g->imu_dev, PWR_MGMT0_GYRO_MODE_LN);
        else
            rc |= inv_imu_set_gyro_mode(&g->imu_dev, PWR_MGMT0_GYRO_MODE_LP);
        si_sleep_us(GYR_STARTUP_TIME_US);
    }
    SI_CHECK_RC(rc);

    return rc;
}

/*
 * 启动算法
 */

static int _start_algo(emd_gaf_t *g)
{
    int rc = 0;
    inv_imu_edmp_gaf_parameters_t       gaf_params;
    inv_imu_edmp_powersave_parameters_t apex_parameters;

    memset(&g->edmp_outputs, 0, sizeof(g->edmp_outputs));

    rc |= inv_imu_edmp_init(&g->imu_dev);
    SI_CHECK_RC(rc);

    /* GAF 参数 */
    rc |= inv_imu_edmp_get_gaf_parameters(&g->imu_dev, &gaf_params);
    gaf_params.pdr_us        = g->gaf_pdr_us;
    gaf_params.run_spherical = g->fusion_enabled;

    /* 高分辨率 gyro 与 GAF 融合不兼容, 且 FIFO hires 已禁用, 统一关闭 */
    g->high_res_en = 0;

    if (g->mag_is_on)
        gaf_params.mag_dt_us = g->mag_odr_us;
    else
        gaf_params.mag_dt_us = 0;
    rc |= inv_imu_read_reg(&g->imu_dev, SW_PLL1_TRIM, 1, (uint8_t *)&gaf_params.clock_variation);

    rc |= inv_imu_edmp_set_gaf_acc_bias(&g->imu_dev, g->acc_bias_q16);
    rc |= inv_imu_edmp_set_gaf_gyr_bias(&g->imu_dev, g->gyr_bias_q12,
                                         g->gyr_bias_temperature, g->gyr_accuracy);
    g->mag_accuracy = 0;
    rc |= inv_imu_edmp_set_gaf_mag_bias(&g->imu_dev, g->mag_bias_q16, g->mag_accuracy);

    rc |= inv_imu_edmp_set_gaf_parameters(&g->imu_dev, &gaf_params);
    SI_CHECK_RC(rc);

    /* 配置 GAF 6 或 9 轴: AG, AM 或 AGM */
    rc |= inv_imu_edmp_set_gaf_mode(&g->imu_dev, g->gyro_is_on, g->mag_is_on);

    /* 省电模式 */
    rc |= inv_imu_edmp_get_powersave_parameters(&g->imu_dev, &apex_parameters);
    if (g->power_save_en) {
        apex_parameters.power_save_en = INV_IMU_ENABLE;
        rc |= inv_imu_adv_configure_wom(&g->imu_dev, DEFAULT_WOM_THS_MG, DEFAULT_WOM_THS_MG,
                                        DEFAULT_WOM_THS_MG, TMST_WOM_CONFIG_WOM_INT_MODE_ANDED,
                                        TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
        rc |= inv_imu_adv_enable_wom(&g->imu_dev);
    } else {
        apex_parameters.power_save_en = INV_IMU_DISABLE;
        rc |= inv_imu_adv_disable_wom(&g->imu_dev);
    }
    rc |= inv_imu_edmp_set_powersave_parameters(&g->imu_dev, &apex_parameters);
    SI_CHECK_RC(rc);

    /* 设置安装矩阵 */
    rc |= inv_imu_edmp_set_mounting_matrix(&g->imu_dev, g->mounting_matrix);
    if (g->mag_is_on) {
        rc |= inv_imu_edmp_set_gaf_soft_iron_cor_matrix(&g->imu_dev, g->soft_iron_matrix);
        rc |= inv_imu_edmp_enable_gaf_soft_iron_cor(&g->imu_dev);
    }
    SI_CHECK_RC(rc);

    rc |= invn_mag_load_ram_image(&g->imu_dev, INVN_MAG_USECASE_IMG_OVER_SIF);
    if (g->mrm_auto_is_on) {
        rc |= invn_mag_enable_automrm(&g->imu_dev);
    } else {
        rc |= invn_mag_disable_automrm(&g->imu_dev);
    }

    if (g->fifo_push_en)
        rc |= inv_imu_edmp_start_gaf_fifo_push(&g->imu_dev);
    else
        rc |= inv_imu_edmp_stop_gaf_fifo_push(&g->imu_dev);

    /* 重置 FIFO */
    rc |= inv_imu_adv_reset_fifo(&g->imu_dev);

    si_disable_irq();
    g->int1_flag = 0;
    si_enable_irq();

    /* 使能 GAF */
    rc |= inv_imu_edmp_enable_gaf(&g->imu_dev);
    SI_CHECK_RC(rc);

    /* 使能 eDMP */
    rc |= inv_imu_edmp_enable(&g->imu_dev);

    /* 让 dmp 看到 ISR0 上的 accel/gyro 数据就绪 */
    rc |= inv_imu_edmp_unmask_int_src(&g->imu_dev, INV_IMU_EDMP_INT0,
                                      EDMP_INT_SRC_ACCEL_DRDY_MASK | EDMP_INT_SRC_GYRO_DRDY_MASK);

    return rc;
}

/*
 * FIFO 传感器事件回调
 */

static void _sensor_event_cb(inv_imu_sensor_event_t *event)
{
    emd_gaf_t *g = g_active_instance;
    if (!g) return;

    static int16_t prev_rgyr[3] = { 0 };
    uint8_t        input_mask   = 0;

    if (event->sensor_mask & (1 << INV_SENSOR_ACCEL)) {
        inv_imu_remap_data(event->accel, g->mounting_matrix);
        if (g->accel_en) {
            input_mask |= MASK_NOTIFY_RAW_ACC_DATA;
        }
    }

    if (event->sensor_mask & (1 << INV_SENSOR_GYRO)) {
        inv_imu_remap_data(event->gyro, g->mounting_matrix);
        if (g->gyro_en) {
            input_mask |= MASK_NOTIFY_RAW_GYR_DATA;
        }
        if (g->high_res_en) {
            prev_rgyr[0] = event->gyro[0];
            prev_rgyr[1] = event->gyro[1];
            prev_rgyr[2] = event->gyro[2];
        }
    }

    if (((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)) ==
        (event->sensor_mask & ((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)))) {
        static inv_imu_edmp_gaf_outputs_t gaf_outputs = { 0 };
        int rc;

        rc = inv_imu_edmp_gaf_decode_fifo(&g->imu_dev, (const uint8_t *)event->es0,
                                          (const uint8_t *)event->es1, &gaf_outputs);
        if (g->high_res_en) {
            if (!g->fusion_enabled) {
                static int32_t rgyr_highres[3] = { 0 };

                if (gaf_outputs.hr_g_valid) {
                    rc |= inv_imu_edmp_decode_gaf_rgyr_highres(
                        &g->imu_dev, gaf_outputs.hr_g, GYRO_FSR_ENUM,
                        prev_rgyr, rgyr_highres);
                }
                event->gyro[0] = rgyr_highres[0];
                event->gyro[1] = rgyr_highres[1];
                event->gyro[2] = rgyr_highres[2];
            }
        }

        if (rc == -1) {
            fprintf(stderr, "[E] Error when rebuilding GAF output, unknown FIFO frame received\n");
            si_sleep_us(10 * 1000 * 1000);
        } else if (gaf_outputs.frame_complete) {
            g->edmp_outputs.grv_quat_valid = gaf_outputs.grv_quat_valid;
            if (g->edmp_outputs.grv_quat_valid) {
                g->edmp_outputs.grv_quat_q30[0] = (int32_t)gaf_outputs.grv_quat_q14[0] << 16;
                g->edmp_outputs.grv_quat_q30[1] = (int32_t)gaf_outputs.grv_quat_q14[1] << 16;
                g->edmp_outputs.grv_quat_q30[2] = (int32_t)gaf_outputs.grv_quat_q14[2] << 16;
                g->edmp_outputs.grv_quat_q30[3] = (int32_t)gaf_outputs.grv_quat_q14[3] << 16;
            }
            g->edmp_outputs.gmrv_quat_valid =
                gaf_outputs.gmrv_quat_valid && gaf_outputs.gmrv_heading_valid;
            if (g->edmp_outputs.gmrv_quat_valid) {
                g->edmp_outputs.gmrv_quat_q30[0] = (int32_t)gaf_outputs.gmrv_quat_q14[0] << 16;
                g->edmp_outputs.gmrv_quat_q30[1] = (int32_t)gaf_outputs.gmrv_quat_q14[1] << 16;
                g->edmp_outputs.gmrv_quat_q30[2] = (int32_t)gaf_outputs.gmrv_quat_q14[2] << 16;
                g->edmp_outputs.gmrv_quat_q30[3] = (int32_t)gaf_outputs.gmrv_quat_q14[3] << 16;
                g->edmp_outputs.gmrv_heading_q27 = (int32_t)gaf_outputs.gmrv_heading_q11 << 16;
            }
            g->edmp_outputs.rv_quat_valid =
                gaf_outputs.rv_quat_valid && gaf_outputs.rv_heading_valid;
            if (g->edmp_outputs.rv_quat_valid) {
                g->edmp_outputs.rv_quat_q30[0] = (int32_t)gaf_outputs.rv_quat_q14[0] << 16;
                g->edmp_outputs.rv_quat_q30[1] = (int32_t)gaf_outputs.rv_quat_q14[1] << 16;
                g->edmp_outputs.rv_quat_q30[2] = (int32_t)gaf_outputs.rv_quat_q14[2] << 16;
                g->edmp_outputs.rv_quat_q30[3] = (int32_t)gaf_outputs.rv_quat_q14[3] << 16;
                g->edmp_outputs.rv_heading_q27 = (int32_t)gaf_outputs.rv_heading_q11 << 16;
            }

            g->edmp_outputs.gyr_bias_valid = gaf_outputs.gyr_bias_valid;
            if (g->edmp_outputs.gyr_bias_valid) {
                g->edmp_outputs.gyr_bias_q16[0] = (int32_t)gaf_outputs.gyr_bias_q12[0] << 4;
                g->edmp_outputs.gyr_bias_q16[1] = (int32_t)gaf_outputs.gyr_bias_q12[1] << 4;
                g->edmp_outputs.gyr_bias_q16[2] = (int32_t)gaf_outputs.gyr_bias_q12[2] << 4;
            }

            g->edmp_outputs.gyr_flags_valid = gaf_outputs.gyr_flags_valid;
            if (g->edmp_outputs.gyr_flags_valid) {
                g->edmp_outputs.gyr_accuracy_flag = gaf_outputs.gyr_accuracy_flag;
                g->edmp_outputs.stationary_flag   = gaf_outputs.stationary_flag;
            }

            g->edmp_outputs.mag_bias_valid = gaf_outputs.mag_bias_valid;
            if (g->edmp_outputs.mag_bias_valid) {
                g->edmp_outputs.mag_bias_q16[0] = (int32_t)gaf_outputs.mag_bias_q16[0];
                g->edmp_outputs.mag_bias_q16[1] = (int32_t)gaf_outputs.mag_bias_q16[1];
                g->edmp_outputs.mag_bias_q16[2] = (int32_t)gaf_outputs.mag_bias_q16[2];
                if (g->freeze_mag_bias) {
                    g->edmp_outputs.mag_bias_q16[0] = g->frozen_bias_mag[0];
                    g->edmp_outputs.mag_bias_q16[1] = g->frozen_bias_mag[1];
                    g->edmp_outputs.mag_bias_q16[2] = g->frozen_bias_mag[2];
                }
                g->edmp_outputs.mag_accuracy_flag = gaf_outputs.mag_accuracy_flag;
                g->edmp_outputs.mag_anomaly       = gaf_outputs.mag_anomalies;
            }

            g->edmp_outputs.rmag_valid = gaf_outputs.rmag_valid;
            if (g->edmp_outputs.rmag_valid) {
                g->edmp_outputs.raw_mag[0] = gaf_outputs.rmag[0];
                g->edmp_outputs.raw_mag[1] = gaf_outputs.rmag[1];
                g->edmp_outputs.raw_mag[2] = gaf_outputs.rmag[2];
            }

            g->edmp_outputs.mrm_state_valid = gaf_outputs.mrm_state_valid;
            if (g->edmp_outputs.mrm_state_valid) {
                g->edmp_outputs.mrm_state = gaf_outputs.mrm_state;
                if (g->mrm_event.INV_GAF_MRM_CHG)
                    g->edmp_outputs.mrm_evt_chg_st = 1;
                if (g->mrm_event.INV_GAF_MRM_RUN)
                    g->edmp_outputs.mrm_evt_exe_mrm = 1;
                if (g->mrm_event.INV_GAF_MRM_THR)
                    g->edmp_outputs.mrm_evt_exc_thr = 1;
                memset(&g->mrm_event, 0, sizeof(g->mrm_event));
            }

            if (g->edmp_outputs.mag_bias_valid && g->edmp_outputs.rmag_valid) {
                g->edmp_outputs.mag_cal_q16[0] =
                    (int32_t)g->edmp_outputs.raw_mag[0] * RAW_MAG_SCALE - g->edmp_outputs.mag_bias_q16[0];
                g->edmp_outputs.mag_cal_q16[1] =
                    (int32_t)g->edmp_outputs.raw_mag[1] * RAW_MAG_SCALE - g->edmp_outputs.mag_bias_q16[1];
                g->edmp_outputs.mag_cal_q16[2] =
                    (int32_t)g->edmp_outputs.raw_mag[2] * RAW_MAG_SCALE - g->edmp_outputs.mag_bias_q16[2];
            }

            memset(&gaf_outputs, 0, sizeof(gaf_outputs));
        }
    }

    /* 每帧都更新: accel, gyro, temp, 输出 (sensor ODR 为准) */
    g->edmp_outputs.acc_cal_q16[0] =
        ((int32_t)event->accel[0] * RAW_ACC_SCALE) - g->acc_bias_q16[0];
    g->edmp_outputs.acc_cal_q16[1] =
        ((int32_t)event->accel[1] * RAW_ACC_SCALE) - g->acc_bias_q16[1];
    g->edmp_outputs.acc_cal_q16[2] =
        ((int32_t)event->accel[2] * RAW_ACC_SCALE) - g->acc_bias_q16[2];
    g->edmp_outputs.acc_cal_valid = 1;

    if (g->gyro_is_on) {
        if (g->high_res_en) {
            g->edmp_outputs.gyr_cal_q16[0] =
                (int32_t)event->gyro[0] * RAW_GYR_SCALE_HR - g->edmp_outputs.gyr_bias_q16[0];
            g->edmp_outputs.gyr_cal_q16[1] =
                (int32_t)event->gyro[1] * RAW_GYR_SCALE_HR - g->edmp_outputs.gyr_bias_q16[1];
            g->edmp_outputs.gyr_cal_q16[2] =
                (int32_t)event->gyro[2] * RAW_GYR_SCALE_HR - g->edmp_outputs.gyr_bias_q16[2];
        } else {
            g->edmp_outputs.gyr_cal_q16[0] =
                (int32_t)event->gyro[0] * RAW_GYR_SCALE - g->edmp_outputs.gyr_bias_q16[0];
            g->edmp_outputs.gyr_cal_q16[1] =
                (int32_t)event->gyro[1] * RAW_GYR_SCALE - g->edmp_outputs.gyr_bias_q16[1];
            g->edmp_outputs.gyr_cal_q16[2] =
                (int32_t)event->gyro[2] * RAW_GYR_SCALE - g->edmp_outputs.gyr_bias_q16[2];
        }
        g->edmp_outputs.gyr_flags_valid = 1;
    }

    g->edmp_outputs.temp_degC_q16 =
        (25 * (1UL << 16)) + ((int32_t)event->temperature * 32768);
    g->edmp_outputs.temperature_valid = 1;

    /* 输出缓存 */
    pthread_mutex_lock(&g->output_mutex);
    _convert_output(&g->edmp_outputs, g->timestamp, &g->cached_output);
    g->output_updated = 1;
    pthread_mutex_unlock(&g->output_mutex);

    /* 更新原始 IMU 缓存 */
    pthread_mutex_lock(&g->output_mutex);

    g->cached_accel.accel_x = event->accel[0] * ACCEL_FSR_G / 32768.0f;
    g->cached_accel.accel_y = event->accel[1] * ACCEL_FSR_G / 32768.0f;
    g->cached_accel.accel_z = event->accel[2] * ACCEL_FSR_G / 32768.0f;
    g->cached_accel.timestamp_us = g->timestamp;

    if (g->high_res_en && !g->fusion_enabled) {
        g->cached_gyro.gyro_x = event->gyro[0] * GYRO_FSR_DPS / 32768.0f;
        g->cached_gyro.gyro_y = event->gyro[1] * GYRO_FSR_DPS / 32768.0f;
        g->cached_gyro.gyro_z = event->gyro[2] * GYRO_FSR_DPS / 32768.0f;
    } else {
        g->cached_gyro.gyro_x = event->gyro[0] * GYRO_FSR_DPS / 32768.0f;
        g->cached_gyro.gyro_y = event->gyro[1] * GYRO_FSR_DPS / 32768.0f;
        g->cached_gyro.gyro_z = event->gyro[2] * GYRO_FSR_DPS / 32768.0f;
    }
    g->cached_gyro.timestamp_us = g->timestamp;
#if EMD_IMU_AXIS_REMAP
    _remap_vec(&g->cached_accel.accel_x, &g->cached_accel.accel_y, &g->cached_accel.accel_z);
    _remap_vec(&g->cached_gyro.gyro_x,   &g->cached_gyro.gyro_y,   &g->cached_gyro.gyro_z);
#endif
#if EMD_IMU_AXIS_REMAP_2
    _remap_vec_2(&g->cached_accel.accel_x, &g->cached_accel.accel_y, &g->cached_accel.accel_z);
    _remap_vec_2(&g->cached_gyro.gyro_x,   &g->cached_gyro.gyro_y,   &g->cached_gyro.gyro_z);
#endif
    g->imu_updated = 1;

    pthread_mutex_unlock(&g->output_mutex);

    /* 原始数据回调 (notify_raw_data 等价, sensor ODR) */
    if (g->raw_data_cb) {
        emd_raw_sensor_t raw;
        raw.accel_x = g->edmp_outputs.acc_cal_q16[0] / 65536.0f;
        raw.accel_y = g->edmp_outputs.acc_cal_q16[1] / 65536.0f;
        raw.accel_z = g->edmp_outputs.acc_cal_q16[2] / 65536.0f;
        raw.gyro_x  = g->edmp_outputs.gyr_cal_q16[0] / 65536.0f;
        raw.gyro_y  = g->edmp_outputs.gyr_cal_q16[1] / 65536.0f;
        raw.gyro_z  = g->edmp_outputs.gyr_cal_q16[2] / 65536.0f;
        raw.temp_c  = g->edmp_outputs.temp_degC_q16 / 65536.0f;
        raw.timestamp_us = g->timestamp;
        /* 对原始数据应用坐标变换 */
#if EMD_IMU_AXIS_REMAP
        _remap_vec(&raw.accel_x, &raw.accel_y, &raw.accel_z);
        _remap_vec(&raw.gyro_x,  &raw.gyro_y,  &raw.gyro_z);
#endif
#if EMD_IMU_AXIS_REMAP_2
        _remap_vec_2(&raw.accel_x, &raw.accel_y, &raw.accel_z);
        _remap_vec_2(&raw.gyro_x,  &raw.gyro_y,  &raw.gyro_z);
#endif
        g->raw_data_cb(&raw, g->raw_data_user);
    }

    /* 清空 MRM 一次性事件标志, quat/mag 保留以供下次 _convert_output 复用 */
    g->edmp_outputs.mrm_evt_chg_st  = 0;
    g->edmp_outputs.mrm_evt_exe_mrm = 0;
    g->edmp_outputs.mrm_evt_exc_thr = 0;
    if (0 == g->gyro_is_on) {
        g->edmp_outputs.gyr_bias_valid  = 0;
        g->edmp_outputs.gyr_flags_valid = 0;
    }
}

/*
 * GPIO 中断回调
 */

static void _int_cb(void *context, unsigned int int_num)
{
    (void)context;

    if (int_num == 1) {
        emd_gaf_t *g = g_active_instance;
        if (g) {
            g->int1_timestamp = si_get_time_us();
            g->int1_flag      = 1;
        }
    }
}

/*
 * 从 NVM 恢复偏置
 */

static int _init_imu_biases(emd_gaf_t *g)
{
    int rc;
    uint8_t sensor_bias[84] = { 0 };
    rc = emd_hal_storage_read(sensor_bias, 84);
    if (0 == rc) {
        memcpy(g->gyr_bias_q12, sensor_bias, 3 * sizeof(g->gyr_bias_q12[0]));
        memcpy(&g->gyr_bias_temperature,
               &sensor_bias[3 * sizeof(g->gyr_bias_q12[0])],
               sizeof(g->gyr_bias_temperature));
        memcpy(g->mag_bias_q16,
               &sensor_bias[3 * sizeof(g->gyr_bias_q12[0]) + sizeof(g->gyr_bias_temperature)],
               3 * sizeof(g->mag_bias_q16[0]));

        g->gyr_accuracy = 3;
        g->mag_accuracy = 3;
    } else {
        memset(g->gyr_bias_q12, 0, 3 * sizeof(g->gyr_bias_q12[0]));
        g->gyr_accuracy = 0;
        g->gyr_bias_temperature = GAF_DEFAULT_TEMPERATURE_INIT_Q16;
        memset(g->mag_bias_q16, 0, 3 * sizeof(g->mag_bias_q16[0]));
        g->mag_accuracy = 0;
        rc = INV_IMU_OK;
    }

    return rc;
}

/*
 * Q30/Q16 ,  float 格式转换
 */

static void _convert_output(const inv_edmp_gaf_outputs_t *in, uint64_t ts,
                            emd_output_t *out)
{
    memset(out, 0, sizeof(emd_output_t));

    out->timestamp_us = ts;

    /* 9轴四元数 (Q30 ,  float) */
    if (in->rv_quat_valid) {
        out->quat_w = in->rv_quat_q30[0] / 1073741824.0f;
        out->quat_x = in->rv_quat_q30[1] / 1073741824.0f;
        out->quat_y = in->rv_quat_q30[2] / 1073741824.0f;
        out->quat_z = in->rv_quat_q30[3] / 1073741824.0f;
        /* 从四元数解算航向角 (弧度转度, Q27) */
        float heading_rad = in->rv_heading_q27 / 134217728.0f;
        out->heading_deg  = heading_rad * 57.29578f;
    }

    /* 校准加速度 (Q16 ,  g) */
    if (in->acc_cal_valid) {
        out->accel_x = in->acc_cal_q16[0] / 65536.0f;
        out->accel_y = in->acc_cal_q16[1] / 65536.0f;
        out->accel_z = in->acc_cal_q16[2] / 65536.0f;
    }

    /* 校准角速度 (Q16 ,  dps) */
    if (in->gyr_flags_valid) {
        out->gyro_x = in->gyr_cal_q16[0] / 65536.0f;
        out->gyro_y = in->gyr_cal_q16[1] / 65536.0f;
        out->gyro_z = in->gyr_cal_q16[2] / 65536.0f;
        out->stationary   = (int)in->stationary_flag;
        out->gyr_accuracy = (int)in->gyr_accuracy_flag;
    }

    /* 校准磁力计 (Q16 ,  uT) */
    if (in->mag_bias_valid && in->rmag_valid) {
        out->mag_x = in->mag_cal_q16[0] / 65536.0f;
        out->mag_y = in->mag_cal_q16[1] / 65536.0f;
        out->mag_z = in->mag_cal_q16[2] / 65536.0f;
        out->mag_accuracy = (int)in->mag_accuracy_flag;
    }

    /* 温度 (Q16 ,  °C) */
    if (in->temperature_valid) {
        out->temp_c = in->temp_degC_q16 / 65536.0f;
    }

#if EMD_IMU_AXIS_REMAP
    /* 传感器坐标 -> 标准机器人坐标(FLU), 见 _remap_vec/_remap_quat */
    _remap_vec(&out->accel_x, &out->accel_y, &out->accel_z);
    _remap_vec(&out->gyro_x,  &out->gyro_y,  &out->gyro_z);
    _remap_vec(&out->mag_x,   &out->mag_y,   &out->mag_z);
    _remap_quat(&out->quat_w, &out->quat_x, &out->quat_y, &out->quat_z);
    out->heading_deg += 90.0f;
    if (out->heading_deg > 180.0f)       out->heading_deg -= 360.0f;
    else if (out->heading_deg < -180.0f) out->heading_deg += 360.0f;
#endif
#if EMD_IMU_AXIS_REMAP_2
    /* 芯片绕 X+90 安装 -> FLU, 见 _remap_vec_2/_remap_quat_2 */
    _remap_vec_2(&out->accel_x, &out->accel_y, &out->accel_z);
    _remap_vec_2(&out->gyro_x,  &out->gyro_y,  &out->gyro_z);
    _remap_vec_2(&out->mag_x,   &out->mag_y,   &out->mag_z);
    _remap_quat_2(&out->quat_w, &out->quat_x, &out->quat_y, &out->quat_z);
    out->heading_deg += 90.0f;
    if (out->heading_deg > 180.0f)       out->heading_deg -= 360.0f;
    else if (out->heading_deg < -180.0f) out->heading_deg += 360.0f;
#endif
}
