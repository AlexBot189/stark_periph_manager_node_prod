/**
 * @file motor_calib.c

 *
 *   motor_para_calibrate: 0, 1(设零位), 2(位置检测±1°成功), 3(失败)
 *   成功后: 使能+电流模式+开透传
 *
 * RV1126B 实现:
 *   - 按键 ,  API 调用
 *   - SDO 轮询角度 ,  motor_hal_get_feedback() 读缓存
 *   - tx_re_flag 同步 ,  motor_hal_c 内部 SDO 队列处理
 */

#include "motor_calib.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#define CALIB_LOG(fmt, ...) \
    fprintf(stderr, "[calib] " fmt "\n", ##__VA_ARGS__)

struct motor_calib {
    motor_hal_t        *hal;        /* HAL 句柄 (不拥有) */
    motor_calib_state_t state;      /* 当前状态 */
    motor_calib_config_t cfg;       /* 校准配置 */
    uint64_t            start_us;   /* 校准开始时间 */
};

/* ---------- 辅助: 获取当前时间 (us) ---------- */
static uint64_t _now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
}

/* ---------- 生命周期 ---------- */

motor_calib_t* motor_calib_create(motor_hal_t *hal)
{
    if (!hal) return NULL;

    motor_calib_t *cal = calloc(1, sizeof(motor_calib_t));
    if (!cal) return NULL;

    cal->hal   = hal;
    cal->state = MOTOR_CALIB_IDLE;

    CALIB_LOG("created");
    return cal;
}

void motor_calib_destroy(motor_calib_t *cal)
{
    if (!cal) return;
    CALIB_LOG("destroyed");
    free(cal);
}

/* ---------- 启动校准 ---------- */

int motor_calib_start(motor_calib_t *cal, const motor_calib_config_t *cfg)
{
    if (!cal || !cfg) return -1;
    if (cal->state != MOTOR_CALIB_IDLE && cal->state != MOTOR_CALIB_TIMEOUT) {
        CALIB_LOG("busy (state=%d)", cal->state);
        return -1;
    }

    cal->cfg = *cfg;
    if (cal->cfg.timeout_ms <= 0)     cal->cfg.timeout_ms = 10000;
    if (cal->cfg.angle_threshold_deg <= 0.0f) cal->cfg.angle_threshold_deg = 1.0f;

    CALIB_LOG("start: R=%d L=%d timeout=%dms thresh=%.1f°",
              cfg->motor_id_r, cfg->motor_id_l,
              cal->cfg.timeout_ms, cal->cfg.angle_threshold_deg);

    /* Step 1: Shutdown 左右电机 (跳过未使用的电机) */
    int ret = 0;
    if (cfg->motor_id_r > 0)
        ret |= motor_hal_sdo_write(cal->hal, cfg->motor_id_r, 0x6040, 0, 0x06, 2);
    if (cfg->motor_id_l > 0)
        ret |= motor_hal_sdo_write(cal->hal, cfg->motor_id_l, 0x6040, 0, 0x06, 2);
    if (ret != 0) {
        CALIB_LOG("shutdown failed");
        return -1;
    }
    usleep(50000); /* 50ms */

    /* Step 2: 设零位 (跳过未使用的电机) */
    ret = 0;
    if (cfg->motor_id_r > 0)
        ret |= motor_hal_set_zero(cal->hal, cfg->motor_id_r);
    if (cfg->motor_id_l > 0)
        ret |= motor_hal_set_zero(cal->hal, cfg->motor_id_l);
    if (ret != 0) {
        CALIB_LOG("set_zero failed");
        return -1;
    }
    usleep(20000); /* 20ms */

    cal->state   = MOTOR_CALIB_CHECKING;
    cal->start_us = _now_us();

    CALIB_LOG("zero set, entering CHECKING");
    return 0;
}

/* ---------- 每轮轮询 ---------- */

motor_calib_state_t motor_calib_poll(motor_calib_t *cal)
{
    if (!cal) return MOTOR_CALIB_TIMEOUT;
    if (cal->state != MOTOR_CALIB_CHECKING) return cal->state;

    /* 检查超时 */
    uint64_t elapsed_us = _now_us() - cal->start_us;
    if (elapsed_us > (uint64_t)cal->cfg.timeout_ms * 1000UL) {
        CALIB_LOG("TIMEOUT after %dms", cal->cfg.timeout_ms);
        cal->state = MOTOR_CALIB_TIMEOUT;
        return MOTOR_CALIB_TIMEOUT;
    }

    /* 从 feedback 缓存读电机状态, 确认数据通道正常 */
    motor_feedback_t fb_r = {0}, fb_l = {0};
    int r_ok = 0, l_ok = 0;
    int need_r = (cal->cfg.motor_id_r > 0) ? 1 : 0;
    int need_l = (cal->cfg.motor_id_l > 0) ? 1 : 0;
    if (need_r) r_ok = motor_hal_get_feedback(cal->hal, cal->cfg.motor_id_r, &fb_r);
    if (need_l) l_ok = motor_hal_get_feedback(cal->hal, cal->cfg.motor_id_l, &fb_l);

    /* 等待所有已配置电机的 feedback 可用 */
    if ((need_r && r_ok != 0) || (need_l && l_ok != 0)) {
        return MOTOR_CALIB_CHECKING;
    }

    /* 验证位置: set_zero 后位置需在 ±threshold 范围内 */
    {
        float thresh = cal->cfg.angle_threshold_deg;
        if (need_r) {
            float angle_r = (float)fb_r.position * 180.0f / 32768.0f;
            if (angle_r < -thresh || angle_r > thresh) {
                return MOTOR_CALIB_CHECKING;
            }
        }
        if (need_l) {
            float angle_l = (float)fb_l.position * 180.0f / 32768.0f;
            if (angle_l < -thresh || angle_l > thresh) {
                return MOTOR_CALIB_CHECKING;
            }
        }
    }

    /* 所有已配置电机位置在阈值内, 校准完成 */
    {
        CALIB_LOG("DONE: R=%d L=%d", need_r, need_l);

        uint8_t id_r = cal->cfg.motor_id_r;
        uint8_t id_l = cal->cfg.motor_id_l;
        int ret = 0;

        /* Step 2.5: 力矩传感器零漂标定 (理论力矩=0, 失能状态, 失败不中断) */
        {
            int tmp_ret;
            if (id_r > 0) {
                tmp_ret = motor_hal_torque_calib(cal->hal, id_r, 0);
                if (tmp_ret != 0) CALIB_LOG("M%d torque zero calib skipped (ret=%d)", id_r, tmp_ret);
            }
            if (id_l > 0) {
                tmp_ret = motor_hal_torque_calib(cal->hal, id_l, 0);
                if (tmp_ret != 0) CALIB_LOG("M%d torque zero calib skipped (ret=%d)", id_l, tmp_ret);
            }
        }

        /* Step 3-5: 使能 + 模式设置 + 透传 (仅 enable_after_done=true) */
        if (cal->cfg.enable_after_done) {
            /* Step 3: DS402 使能 (0x06, 0x07, 0x0F) */
            if (id_r > 0) ret |= motor_hal_sdo_write(cal->hal, id_r, 0x6040, 0, 0x06, 2);
            if (id_l > 0) ret |= motor_hal_sdo_write(cal->hal, id_l, 0x6040, 0, 0x06, 2);
            usleep(20000);
            if (id_r > 0) ret |= motor_hal_sdo_write(cal->hal, id_r, 0x6040, 0, 0x07, 2);
            if (id_l > 0) ret |= motor_hal_sdo_write(cal->hal, id_l, 0x6040, 0, 0x07, 2);
            usleep(20000);
            if (id_r > 0) ret |= motor_hal_sdo_write(cal->hal, id_r, 0x6040, 0, 0x0F, 2);
            if (id_l > 0) ret |= motor_hal_sdo_write(cal->hal, id_l, 0x6040, 0, 0x0F, 2);

            if (ret != 0) {
                CALIB_LOG("enable failed");
                cal->state = MOTOR_CALIB_TIMEOUT;
                return MOTOR_CALIB_TIMEOUT;
            }
            usleep(120000); /* 等抱闸释放 */

            /* Step 4: 设置控制模式 (默认电流模式) */
            if (id_r > 0) motor_hal_set_mode(cal->hal, id_r, cal->cfg.ctrl_mode);
            if (id_l > 0) motor_hal_set_mode(cal->hal, id_l, cal->cfg.ctrl_mode);

            /* Step 5: 启动传感器透传, 0.5ms/2000Hz 最快周期,
             * 与 main_loop 默认一致, 避免校准覆盖成慢周期 */
            if (id_r > 0) motor_hal_sensor_config(cal->hal, id_r, 4, 3);
            if (id_l > 0) motor_hal_sensor_config(cal->hal, id_l, 4, 3);
        }

        cal->state = MOTOR_CALIB_DONE;
        return MOTOR_CALIB_DONE;
    }

    return MOTOR_CALIB_CHECKING;
}

/* ---------- 退出校准 ---------- */

int motor_calib_exit(motor_calib_t *cal)
{
    if (!cal) return -1;
    if (cal->state == MOTOR_CALIB_IDLE) return 0;

    uint8_t id_r = cal->cfg.motor_id_r;
    uint8_t id_l = cal->cfg.motor_id_l;

    CALIB_LOG("exit: stopping motors");

    /* 关闭透传 */
    if (id_r > 0) motor_hal_sensor_stop(cal->hal, id_r);
    if (id_l > 0) motor_hal_sensor_stop(cal->hal, id_l);

    /* 电流置零 (0x6071 2字节 int16) */
    if (id_r > 0) motor_hal_sdo_write(cal->hal, id_r, 0x6071, 0, 0, 2);
    if (id_l > 0) motor_hal_sdo_write(cal->hal, id_l, 0x6071, 0, 0, 2);

    /* 脱使能 */
    if (id_r > 0) motor_hal_sdo_write(cal->hal, id_r, 0x6040, 0, 0x06, 2);
    if (id_l > 0) motor_hal_sdo_write(cal->hal, id_l, 0x6040, 0, 0x06, 2);

    cal->state = MOTOR_CALIB_IDLE;
    return 0;
}

motor_calib_state_t motor_calib_get_state(const motor_calib_t *cal)
{
    return cal ? cal->state : MOTOR_CALIB_TIMEOUT;
}
