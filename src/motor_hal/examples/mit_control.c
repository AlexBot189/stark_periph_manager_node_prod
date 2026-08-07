/**
 * @file mit_control.c
 * @brief MIT 阻抗控制示例
 *
 * 力位混合控制, 适用于协作/柔顺场景。
 */

#include "motor_hal.h"
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

static volatile int g_running = 1;
static void sig_handler(int sig) { g_running = 0; }

static uint64_t _now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void on_fb(uint8_t id, const motor_feedback_t *fb, void *ctx)
{
    static int cnt = 0;
    if (++cnt % 20 == 0) {
        printf("[ID%d] pos=%.1f° vel=%dRPM cur=%dmA err=0x%04X\n",
               id, motor_counts_to_deg(fb->position),
               fb->velocity, fb->current_iq, fb->error_code);
    }
}

int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    motor_hal_t *hal = motor_hal_create();
    if (!hal) return 1;
    if (motor_hal_init(hal, "can0", 1000000, 5000000) < 0) {
        fprintf(stderr, "CAN init failed\n");
        motor_hal_destroy(hal); return 1;
    }

    motor_config_t cfg = {0};
    cfg.node_id      = 1;
    cfg.heartbeat_ms = 2000;
    cfg.disable_watchdog = true;
    cfg.auto_enable  = true;
    cfg.bootup_timeout_ms = 5000;

    motor_hal_add_motor(hal, &cfg);
    motor_hal_set_feedback_cb(hal, 1, on_fb, NULL);

    printf("Starting motor...\n");
    if (motor_hal_startup(hal, 1, 5000) != 0) {
        fprintf(stderr, "Startup failed\n");
        motor_hal_destroy(hal); return 1;
    }
    printf("Motor started\n");

    /* MIT 模式控制循环 */
    float kp = 0.5f;
    float kd = 0.1f;
    uint64_t last = 0;

    while (g_running) {
        for (int i = 0; i < 5; i++) motor_hal_poll(hal, 0);

        uint64_t now = _now_us();
        if (now - last < 20000) continue;  /* 50Hz */
        last = now;

        float t = now * 1e-6f;
        float target_pos = 30.0f * sinf(t);
        motor_hal_mit_control(hal, 1, target_pos, 0.0f, kp, kd, 0.0f);
    }

    printf("\nShutdown...\n");
    motor_hal_disable(hal, 1);
    motor_hal_destroy(hal);
    return 0;
}
