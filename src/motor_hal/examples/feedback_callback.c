/**
 * @file feedback_callback.c
 * @brief 外骨骼上层算法示例 — 注册反馈回调, 每 5ms 收到一次电机数据
 *
 * 用法:
 *   ./motor_tool daemon can0 &        # 启动 daemon (自动发 SYNC@5ms)
 *   ./feedback_callback               # 运行本示例 (注册回调)
 *
 * 工作原理:
 *   daemon 以 5ms 周期发 SYNC ,  电机收到 SYNC 后发 TPDO (0x181)
 *   ,  recv 线程收到 TPDO ,  更新缓存 ,  触发回调
 *   ,  回调中做算法分析 (如: 外骨骼力矩估算/姿态解算)
 */

#include "motor_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

static volatile int running = 1;

static void sig_handler(int sig) { (void)sig; running = 0; }

/* ================================================================
 * 反馈回调 — 每 5ms 触发一次 (在 recv 线程上下文)
 *
 * 在这里做:
 *   - 外骨骼关节角度读取
 *   - 力矩估算 (电流 ,  力矩)
 *   - 速度/加速度计算
 *   - 数据推送到共享内存/UDP/ROS2
 *
 * ⚠️ 注意: 回调在 recv 线程中执行, 不要:
 *   - 调用 SDO (会死锁)
 *   - 做耗时操作 (printf 也尽量少用)
 *   - 阻塞 recv 线程
 * ================================================================ */

/* 环形缓冲区 (无锁, 单生产者单消费者) */
#define FB_BUF_SIZE 256
static motor_feedback_t fb_buf[FB_BUF_SIZE];
static volatile int      fb_write_idx = 0;
static int               fb_read_idx  = 0;

static void fb_callback(uint8_t node_id, const motor_feedback_t *fb, void *ctx)
{
    (void)ctx;

    /* 写入环形缓冲区 (无锁, 因为 producer 是单线程) */
    fb_buf[fb_write_idx] = *fb;
    fb_write_idx = (fb_write_idx + 1) % FB_BUF_SIZE;
}

/* ================================================================
 * 上层: 从环形缓冲区取出数据, 做算法
 * ================================================================ */

static void process_feedback(void)
{
    while (fb_read_idx != fb_write_idx) {
        motor_feedback_t fb = fb_buf[fb_read_idx];
        fb_read_idx = (fb_read_idx + 1) % FB_BUF_SIZE;

        /* 这里做外骨骼算法: */
        float angle_deg   = motor_counts_to_deg(fb.position);
        float speed_rpm   = (float)fb.velocity;
        float current_a   = motor_ma_to_a(fb.current_iq);
        float temp_c      = motor_temp_to_c(fb.temperature);

        printf("[FB] pos=%.2f° vel=%.1fRPM cur=%.2fA temp=%.1f°C err=0x%04X\n",
               angle_deg, speed_rpm, current_a, temp_c, fb.error_code);
    }
}

/* ================================================================
 * 主函数
 * ================================================================ */

int main(void)
{
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 1. 创建 HAL 并初始化 CAN (独立进程, 不走 daemon socket) */
    motor_hal_t *hal = motor_hal_create();
    if (!hal) { fprintf(stderr, "create failed\n"); return 1; }

    int ret = motor_hal_init(hal, "can0", 1000000, 5000000);
    if (ret < 0) {
        fprintf(stderr, "CAN init failed (is can0 up?)\n");
        motor_hal_destroy(hal);
        return 1;
    }

    /* 2. 注册电机 (只监听, 不使能 — daemon 已负责使能) */
    motor_config_t cfg = {0};
    cfg.node_id = 1;
    cfg.auto_enable = false;  /* daemon 已使能, 这里只监听 */
    motor_hal_add_motor(hal, &cfg);

    /* 3. 注册反馈回调 */
    motor_hal_set_feedback_cb(hal, 1, fb_callback, NULL);

    /* 4. 启动接收线程 (监听 CAN, 触发回调) */
    motor_hal_recv_start(hal);

    printf("Feedback callback demo started. Press Ctrl+C to stop.\n");
    printf("Receiving TPDO from motor 1 at 5ms intervals...\n\n");

    /* 5. 主循环: 处理反馈数据 */
    while (running) {
        process_feedback();
        usleep(5000);  /* 5ms 处理周期 */
    }

    /* 6. 清理 */
    printf("\nStopping...\n");
    motor_hal_recv_stop(hal);
    motor_hal_destroy(hal);
    printf("Done.\n");

    return 0;
}
