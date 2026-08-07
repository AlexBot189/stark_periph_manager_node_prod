/**
 * @file motor_calib.h
 * @brief 巨蟹关节电机校准状态机
 *
 * motor.c 的校准逻辑。
 * 校准通过 API 调用 + feedback 缓存读取。
 */

#ifndef MOTOR_CALIB_H
#define MOTOR_CALIB_H

#include "motor_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 校准状态
 * ================================================================ */

typedef enum {
    MOTOR_CALIB_IDLE     = 0,  /* 未启动 */
    MOTOR_CALIB_ZEROING  = 1,  /* 正在设零位 */
    MOTOR_CALIB_CHECKING = 2,  /* 正在检测位置 */
    MOTOR_CALIB_DONE     = 3,  /* 校准成功 */
    MOTOR_CALIB_TIMEOUT  = -1, /* 超时失败 */
} motor_calib_state_t;

/* ================================================================
 * 校准配置
 * ================================================================ */

typedef struct {
    uint8_t   motor_id_r;          /* 右电机 CAN ID (1) */
    uint8_t   motor_id_l;          /* 左电机 CAN ID (2) */
    int       timeout_ms;          /* 总超时 (ms), 默认 10000 */
    float     angle_threshold_deg; /* 位置判定阈值 (°), 默认 1.0 */
    uint8_t   ctrl_mode;           /* 校准后切换的控制模式, 默认 MOTOR_MODE_CURRENT */
    bool      enable_after_done;   /* 校准完成后是否 SDO 使能电机, 默认 true */
} motor_calib_config_t;

/* ================================================================
 * 校准器句柄 (不透明)
 * ================================================================ */

typedef struct motor_calib motor_calib_t;

/* ================================================================
 * API
 * ================================================================ */

/**
 * @brief 创建校准器实例
 * @param hal 已初始化的 motor_hal 句柄
 * @return 校准器指针, NULL=失败
 */
motor_calib_t* motor_calib_create(motor_hal_t *hal);

/** @brief 销毁校准器 */
void motor_calib_destroy(motor_calib_t *cal);

/**
 * @brief 启动校准流程
 *
 * 内部步骤:
 *   1. 发 Shutdown (0x06) 给左右电机
 *   2. 设零位 (SDO写 0x2531=1)
 *   3. 状态切换为 CHECKING
 *
 * @param cal  校准器
 * @param cfg  校准配置 (内部拷贝)
 * @return 0=成功; <0=HAL错误
 */
int motor_calib_start(motor_calib_t *cal, const motor_calib_config_t *cfg);

/**
 * @brief 每轮循环调用一次 (非阻塞)
 *
 * 在校准 CHECKING 阶段, 从 feedback 缓存读左右电机位置。
 * 检测到左右位置均在 ±threshold 范围内 ,  校准成功。
 *
 * @return 当前校准状态 (MOTOR_CALIB_DONE/TIMEOUT/CHECKING/...)
 */
motor_calib_state_t motor_calib_poll(motor_calib_t *cal);

/** @brief 获取当前校准状态 */
motor_calib_state_t motor_calib_get_state(const motor_calib_t *cal);

/** @brief 退出校准 (恢复未校准状态, 关闭透传, 电流置零) */
int motor_calib_exit(motor_calib_t *cal);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CALIB_H */
