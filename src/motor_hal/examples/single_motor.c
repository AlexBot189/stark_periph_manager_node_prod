/**
 * @file single_motor.c
 * @brief 单电机控制示例
 */

#include "motor_hal.h"
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

static volatile int g_running = 1;

static void sig_handler(int sig) { g_running = 0; }

static uint64_t _now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void on_feedback(uint8_t id, const motor_feedback_t *fb, void *ctx)
{
    static int count = 0;
    if (++count % 50 == 0) {
        printf("[Motor %d] pos=%.2f° vel=%dRPM cur=%dmA temp=%.1f°C err=0x%04X %s\n",
               id, motor_counts_to_deg(fb->position),
               fb->velocity, fb->current_iq,
               motor_temp_to_c(fb->temperature),
               fb->error_code,
               (fb->status_byte & 0x20) ? "ERR" : "");
    }
}

int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* 1. 创建 HAL */
    motor_hal_t *hal = motor_hal_create();
    if (!hal) { fprintf(stderr, "create HAL failed\n"); return 1; }

    int ret = motor_hal_init(hal, "can0", 1000000, 5000000);
    if (ret < 0) { fprintf(stderr, "CAN init failed: %d\n", ret); motor_hal_destroy(hal); return 1; }
    printf("CAN opened\n");

    /* 2. 注册电机 */
    motor_config_t cfg = {0};
    cfg.node_id          = 1;
    cfg.heartbeat_ms     = 2000;
    cfg.profile_accel    = 5000;
    cfg.profile_decel    = 5000;
    cfg.profile_velocity = 10;
    cfg.disable_watchdog = true;
    cfg.auto_enable      = true;
    cfg.bootup_timeout_ms = 5000;

    ret = motor_hal_add_motor(hal, &cfg);
    if (ret != 0) { fprintf(stderr, "add motor failed: %d\n", ret); motor_hal_destroy(hal); return 1; }
    motor_hal_set_feedback_cb(hal, 1, on_feedback, NULL);

    /* 3. 启动 */
    printf("Starting motor 1...\n");
    ret = motor_hal_startup(hal, 1, 5000);
    if (ret != 0) { fprintf(stderr, "startup failed: %d\n", ret); motor_hal_destroy(hal); return 1; }
    printf("Motor started\n");

    /* 4. 控制循环 */
    float target = 0.0f;
    int dir = 1;
    uint64_t last = 0;

    while (g_running) {
        motor_hal_poll(hal, 1);

        uint64_t now = _now_us();
        if (now - last > 100000) {  /* 100ms ,  10Hz */
            last = now;
            target += dir * 10.0f;
            if (target > 90.0f)  { target = 90.0f;  dir = -1; }
            if (target < -90.0f) { target = -90.0f; dir = 1;  }
            motor_hal_set_position(hal, 1, target);
        }
    }

    /* 5. 清理 */
    printf("\nShutting down...\n");
    motor_hal_disable(hal, 1);
    motor_hal_destroy(hal);
    return 0;
}
