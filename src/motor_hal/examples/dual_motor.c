/**
 * @file dual_motor.c
 * @brief 双关节电机控制示例
 *
 * 场景: 双足/双轮机器人, 左右关节同步控制
 * 反馈回调供步态算法用。
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

/* 全局状态 (供步态算法读取) */
static struct {
    float angle_deg;
    float velocity_rpm;
    float current_a;
    float temp_c;
    bool  has_error;
} g_left, g_right;

static void on_left_fb(uint8_t id, const motor_feedback_t *fb, void *ctx)
{
    g_left.angle_deg    = motor_counts_to_deg(fb->position);
    g_left.velocity_rpm = (float)fb->velocity;
    g_left.current_a    = motor_ma_to_a(fb->current_iq);
    g_left.temp_c       = motor_temp_to_c(fb->temperature);
    g_left.has_error    = (fb->status_byte & 0x20) != 0;
}

static void on_right_fb(uint8_t id, const motor_feedback_t *fb, void *ctx)
{
    g_right.angle_deg    = motor_counts_to_deg(fb->position);
    g_right.velocity_rpm = (float)fb->velocity;
    g_right.current_a    = motor_ma_to_a(fb->current_iq);
    g_right.temp_c       = motor_temp_to_c(fb->temperature);
    g_right.has_error    = (fb->status_byte & 0x20) != 0;
}

int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* 1. 初始化 */
    motor_hal_t *hal = motor_hal_create();
    if (!hal) return 1;
    if (motor_hal_init(hal, "can0", 1000000, 5000000) < 0) {
        fprintf(stderr, "CAN init failed\n");
        motor_hal_destroy(hal); return 1;
    }

    /* 2. 注册左右关节 */
    motor_config_t cfg_left = {0};
    cfg_left.node_id          = 1;
    cfg_left.heartbeat_ms     = 2000;
    cfg_left.profile_accel    = 5000;
    cfg_left.profile_decel    = 5000;
    cfg_left.profile_velocity = 10;
    cfg_left.disable_watchdog = true;
    cfg_left.auto_enable      = true;
    cfg_left.bootup_timeout_ms = 5000;

    motor_config_t cfg_right = cfg_left;
    cfg_right.node_id = 2;

    motor_hal_add_motor(hal, &cfg_left);
    motor_hal_add_motor(hal, &cfg_right);
    motor_hal_set_feedback_cb(hal, 1, on_left_fb, NULL);
    motor_hal_set_feedback_cb(hal, 2, on_right_fb, NULL);

    /* 3. 启动 */
    printf("Starting left (ID=1)...\n");
    if (motor_hal_startup(hal, 1, 5000) != 0) { fprintf(stderr, "left fail\n"); return 1; }
    printf("Starting right (ID=2)...\n");
    if (motor_hal_startup(hal, 2, 5000) != 0) { fprintf(stderr, "right fail\n"); return 1; }

    printf("Control loop running...\n");

    /* 4. 控制循环 */
    uint64_t last = 0;
    while (g_running) {
        /* 高频轮询 */
        for (int i = 0; i < 5; i++) motor_hal_poll(hal, 0);

        uint64_t now = _now_us();
        if (now - last < 5000) continue;  /* 200Hz */
        last = now;

        /* 步态算法 */
        float t = now * 1e-6f;
        float left_target  =  30.0f * sinf(t * 2.0f);
        float right_target = -30.0f * sinf(t * 2.0f);

        motor_hal_set_position(hal, 1, left_target);
        motor_hal_set_position(hal, 2, right_target);

        static int cnt = 0;
        if (++cnt % 40 == 0) {
            printf("L:%6.1f° R:%6.1f° | L:%4dRPM R:%4dRPM\n",
                   g_left.angle_deg, g_right.angle_deg,
                   (int)g_left.velocity_rpm, (int)g_right.velocity_rpm);
        }
    }

    /* 5. 清理 */
    printf("\nShutdown...\n");
    motor_hal_disable(hal, 1);
    motor_hal_disable(hal, 2);
    motor_hal_destroy(hal);
    return 0;
}
