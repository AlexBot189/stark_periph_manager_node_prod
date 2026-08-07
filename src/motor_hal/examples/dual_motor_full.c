/**
 * @file dual_motor_full.c
 * @brief 双关节电机完整控制示例 — CANFD + CANopen 全流程
 *
 * 覆盖:
 *   CANFD 初始化 (仲裁1M + 数据5M)
 *   双电机启动 (Bootup检测 ,  心跳配置 ,  关看门狗 ,  DS402使能)
 *   5ms 控制周期 (PDO 自动触发反馈, 不需要 SYNC)
 *   实时控制 (位置/速度/电流, 参数动态传入)
 *   故障处理 (EMCY + 反馈错误码)
 *   安全停机
 *
 * 硬件: RV1126B + SocketCAN can0 + 巨蟹驱动板
 * 编译: gcc -o dual_full dual_motor_full.c -lmotor_hal -lpthread -lm
 */

#include "motor_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

/* ================================================================
 * 配置常量
 * ================================================================ */

#define CAN_IFACE           "can0"
#define CAN_ARB_BITRATE     1000000   /* 仲裁段 1Mbps */
#define CAN_DATA_BITRATE    5000000   /* 数据段 5Mbps (CANFD) */

#define LEFT_NODE_ID        1
#define RIGHT_NODE_ID       2

#define CTRL_INTERVAL_US    5000      /* 控制周期 5ms (200Hz) */
#define STARTUP_TIMEOUT_MS  5000

/* ================================================================
 * 全局状态
 * ================================================================ */

static volatile int g_running = 1;

/* 双电机反馈数据 (供上层算法读取) */
typedef struct {
    float    angle_deg;       /* 角度 (°) */
    int16_t  velocity_rpm;    /* 速度 (RPM) */
    int16_t  current_ma;      /* 电流 (mA) */
    float    temp_c;           /* 温度 (°C) */
    uint16_t error_code;
    uint8_t  mode;
    bool     enabled;
    bool     brake_released;
    bool     has_error;
    bool     target_reached;
    uint64_t timestamp_us;
    int      update_count;    /* 收到反馈次数 */
} joint_state_t;

static joint_state_t g_left, g_right;

/* ================================================================
 * 反馈回调 — 5ms 一帧, 供步态算法用
 * ================================================================ */

static void on_left_feedback(uint8_t id, const motor_feedback_t *fb, void *ctx)
{
    g_left.angle_deg     = motor_counts_to_deg(fb->position);
    g_left.velocity_rpm  = fb->velocity;
    g_left.current_ma    = fb->current_iq;
    g_left.temp_c        = motor_temp_to_c(fb->temperature);
    g_left.error_code    = fb->error_code;
    g_left.mode          = fb->mode;
    g_left.enabled       = (fb->status_byte & 0x80) != 0;
    g_left.brake_released= (fb->status_byte & 0x40) != 0;
    g_left.has_error     = (fb->status_byte & 0x20) != 0;
    g_left.target_reached= (fb->status_byte & 0x10) != 0;
    g_left.timestamp_us  = fb->timestamp_us;
    g_left.update_count++;

    /* 错误时立即打印 */
    if (g_left.has_error) {
        fprintf(stderr, "[LEFT] ERROR: 0x%04X\n", g_left.error_code);
    }
}

static void on_right_feedback(uint8_t id, const motor_feedback_t *fb, void *ctx)
{
    g_right.angle_deg     = motor_counts_to_deg(fb->position);
    g_right.velocity_rpm  = fb->velocity;
    g_right.current_ma    = fb->current_iq;
    g_right.temp_c        = motor_temp_to_c(fb->temperature);
    g_right.error_code    = fb->error_code;
    g_right.mode          = fb->mode;
    g_right.enabled       = (fb->status_byte & 0x80) != 0;
    g_right.brake_released= (fb->status_byte & 0x40) != 0;
    g_right.has_error     = (fb->status_byte & 0x20) != 0;
    g_right.target_reached= (fb->status_byte & 0x10) != 0;
    g_right.timestamp_us  = fb->timestamp_us;
    g_right.update_count++;

    if (g_right.has_error) {
        fprintf(stderr, "[RIGHT] ERROR: 0x%04X\n", g_right.error_code);
    }
}

/* EMCY 紧急回调 */
static void on_error(uint8_t id, uint16_t code, void *ctx)
{
    fprintf(stderr, "[EMCY] Motor %d emergency: 0x%04X\n", id, code);
}

/* 状态迁移回调 */
static void on_state_change(uint8_t id,
                            motor_state_t old, motor_state_t new_state, void *ctx)
{
    printf("[Motor %d] State: %s ,  %s\n",
           id, motor_state_str(old), motor_state_str(new_state));
}

/* ================================================================
 * 信号处理
 * ================================================================ */

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ================================================================
 * 时间工具
 * ================================================================ */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void sleep_us(uint64_t us) {
    struct timespec ts;
    ts.tv_sec  = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000;
    nanosleep(&ts, NULL);
}

/* ================================================================
 * 初始化: CANFD 接口
 * ================================================================ */

static int canfd_init(motor_hal_t *hal)
{
    int ret;

    printf("=== Step 1: CANFD 初始化 ===\n");
    printf("  Interface: %s\n", CAN_IFACE);
    printf("  Arbitration: %d bps\n", CAN_ARB_BITRATE);
    printf("  Data phase:  %d bps (FD mode)\n", CAN_DATA_BITRATE);

    ret = motor_hal_init(hal, CAN_IFACE, CAN_ARB_BITRATE, CAN_DATA_BITRATE);
    if (ret < 0) {
        fprintf(stderr, "CANFD init failed: %d\n", ret);
        return ret;
    }
    printf("  ✓ CANFD interface up\n");
    return 0;
}

/* ================================================================
 * 电机注册 + 回调
 * ================================================================ */

static int motor_register(motor_hal_t *hal, uint8_t node_id,
                          motor_feedback_cb_t fb_cb)
{
    motor_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.node_id           = node_id;
    cfg.heartbeat_ms      = 2000;         /* 心跳间隔 */
    cfg.profile_accel     = 5000;         /* 加速度 5000 RPM/s (电机端) */
    cfg.profile_decel     = 5000;         /* 减速度 */
    cfg.profile_velocity  = 20;           /* 轨迹速度 20 RPM (输出端) */
    cfg.pos_limit_pos     = 180.0f;       /* 正限位 */
    cfg.pos_limit_neg     = -180.0f;      /* 负限位 */
    cfg.disable_watchdog  = true;         /* 关闭看门狗 (推荐, 避免喂狗开销) */
    cfg.auto_enable       = true;         /* startup 自动使能 */
    cfg.bootup_timeout_ms = 3000;         /* Bootup 超时 */

    int ret = motor_hal_add_motor(hal, &cfg);
    if (ret != 0) {
        fprintf(stderr, "Add motor %d failed: %d\n", node_id, ret);
        return ret;
    }

    /* 注册三重回调 */
    motor_hal_set_feedback_cb(hal, node_id, fb_cb, NULL);
    motor_hal_set_error_cb(hal, node_id, on_error, NULL);
    motor_hal_set_state_cb(hal, node_id, on_state_change, NULL);

    printf("  Motor %d registered: HB=%dms, Accel=%d, V=%d RPM, "
           "Watchdog=OFF\n",
           node_id, cfg.heartbeat_ms, cfg.profile_accel,
           cfg.profile_velocity);
    return 0;
}

/* ================================================================
 * 启动: 等待 Bootup ,  心跳 ,  关看门狗 ,  使能
 * ================================================================ */

static int motor_start(motor_hal_t *hal, uint8_t node_id)
{
    int ret;

    /* 内部流程:
     *  1. 等待 Bootup 帧 (0x700+ID, data[0]=0x00)
     *  2. SDO 写 0x1017=2000ms (心跳配置)
     *  3. SDO 写 0x2650=1 (关看门狗)
     *  4. SDO 读 0x100A 固件版本 (通信验证)
     *  5. DS402: Shutdown(0x06), SwitchOn(0x07), EnableOp(0x0F)
     *  6. 延时 120ms 等待抱闸释放
     */
    ret = motor_hal_startup(hal, node_id, STARTUP_TIMEOUT_MS);
    if (ret != 0) {
        fprintf(stderr, "Motor %d startup failed: %d\n", node_id, ret);
    }
    return ret;
}

/* ================================================================
 * 控制: 动态参数 (角度/速度/电流 全部传参)
 * ================================================================ */

/* 步态算法: 返回左右目标角度 */
static void gait_calc(float *left_deg, float *right_deg, float t)
{
    /* 正弦摆动, 你可以替换成自己的步态算法 */
    float amp = 30.0f;      /* 摆幅 ±30° */
    float freq = 1.0f;      /* 频率 1Hz */
    float phase = 2.0f * M_PI * freq * t;

    *left_deg  =  amp * sinf(phase);
    *right_deg = -amp * sinf(phase);
}

/* ================================================================
 * 打印反馈 (低频, 避免刷屏)
 * ================================================================ */

static void print_feedback(float t)
{
    static int count = 0;
    if (++count % 40 == 0) {  /* 每40个控制周期打印一次 (~5Hz) */
        printf("\n[%.3fs] ================================\n", t);
        printf(" LEFT  | pos=%7.2f° | vel=%5d RPM | cur=%5d mA | "
               "temp=%5.1f°C | err=0x%04X | cnt=%d\n",
               g_left.angle_deg, g_left.velocity_rpm,
               g_left.current_ma, g_left.temp_c,
               g_left.error_code, g_left.update_count);
        printf(" RIGHT | pos=%7.2f° | vel=%5d RPM | cur=%5d mA | "
               "temp=%5.1f°C | err=0x%04X | cnt=%d\n",
               g_right.angle_deg, g_right.velocity_rpm,
               g_right.current_ma, g_right.temp_c,
               g_right.error_code, g_right.update_count);
    }
}

/* ================================================================
 * 主程序
 * ================================================================ */

int main(void)
{
    int ret;

    /* 注册信号 */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* ============================================================
     * Phase 1: CANFD 初始化
     * ============================================================ */
    printf("\n╔══════════════════════════════════════╗\n");
    printf("║  Dual Motor CANopen/CANFD HAL Test  ║\n");
    printf("╚══════════════════════════════════════╝\n\n");

    motor_hal_t *hal = motor_hal_create();
    if (!hal) { fprintf(stderr, "HAL create failed\n"); return 1; }

    ret = canfd_init(hal);
    if (ret < 0) { motor_hal_destroy(hal); return 1; }

    /* ============================================================
     * Phase 2: 注册双电机
     * ============================================================ */
    printf("\n=== Step 2: 注册电机 ===\n");

    ret = motor_register(hal, LEFT_NODE_ID,  on_left_feedback);
    if (ret < 0) { motor_hal_destroy(hal); return 1; }

    ret = motor_register(hal, RIGHT_NODE_ID, on_right_feedback);
    if (ret < 0) { motor_hal_destroy(hal); return 1; }

    /* ============================================================
     * Phase 3: 上电启动 (等 Bootup + 心跳 + 关狗 + 使能)
     * ============================================================ */
    printf("\n=== Step 3: 启动电机 ===\n");

    printf("  Waiting for left motor (ID=%d) bootup + enable...\n", LEFT_NODE_ID);
    ret = motor_start(hal, LEFT_NODE_ID);
    if (ret < 0) {
        fprintf(stderr, "Left motor start FAILED\n");
        motor_hal_destroy(hal); return 1;
    }
    printf("  ✓ Left motor: Operation Enabled\n");

    printf("  Waiting for right motor (ID=%d) bootup + enable...\n", RIGHT_NODE_ID);
    ret = motor_start(hal, RIGHT_NODE_ID);
    if (ret < 0) {
        fprintf(stderr, "Right motor start FAILED\n");
        motor_hal_disable(hal, LEFT_NODE_ID);  /* 回滚左电机 */
        motor_hal_destroy(hal); return 1;
    }
    printf("  ✓ Right motor: Operation Enabled\n");

    /* ============================================================
     * Phase 4: 配置 PDO 映射 (可选, 演示标准 PDO 用法)
     * ============================================================ */
    printf("\n=== Step 4: CANopen 配置 ===\n");

    /* 4a. 设置运行模式为 PP (位置模式) */
    motor_hal_set_mode(hal, LEFT_NODE_ID,  MOTOR_MODE_PROFILE_POS);
    motor_hal_set_mode(hal, RIGHT_NODE_ID, MOTOR_MODE_PROFILE_POS);
    printf("  Mode: Profile Position\n");

    /* 4b. 设置加减速 (可在运行时动态修改) */
    motor_hal_set_accel_decel(hal, LEFT_NODE_ID,  5000, 5000);
    motor_hal_set_accel_decel(hal, RIGHT_NODE_ID, 5000, 5000);
    printf("  Accel/Decel: 5000 RPM/s\n");

    /* 4c. 设置限位 */
    motor_hal_set_limits(hal, LEFT_NODE_ID,  180.0f, -180.0f);
    motor_hal_set_limits(hal, RIGHT_NODE_ID, 180.0f, -180.0f);
    printf("  Limits: ±180°\n");

    printf("  ✓ CANopen configured\n");

    /* 额外延时确保抱闸完全释放 */
    printf("\n  Waiting 200ms for brake release...\n");
    usleep(200000);

    /* ============================================================
     * Phase 5: 主控制循环 (200Hz, 5ms)
     * ============================================================ */
    printf("\n=== Step 5: Control Loop (200Hz) ===\n");
    printf("  Press Ctrl+C to stop\n\n");

    uint64_t t_start  = now_us();
    uint64_t last_ctrl = t_start;
    uint64_t last_print = t_start;

    memset(&g_left,  0, sizeof(g_left));
    memset(&g_right, 0, sizeof(g_right));

    while (g_running) {
        /* ── 高频轮询 CAN 帧 (非阻塞) ── */
        for (int i = 0; i < 5; i++) {
            motor_hal_poll(hal, 0);  /* timeout=0 ,  非阻塞 */
        }

        uint64_t now = now_us();

        /* ── 5ms 控制周期 ── */
        if (now - last_ctrl >= CTRL_INTERVAL_US) {
            last_ctrl = now;
            float t = (now - t_start) * 1e-6f;

            /* 计算目标角度 (可动态修改参数!) */
            float left_deg, right_deg;
            gait_calc(&left_deg, &right_deg, t);

            /* 下发控制 — 角度值动态传参 */
            motor_hal_set_position(hal, LEFT_NODE_ID,  left_deg);
            motor_hal_set_position(hal, RIGHT_NODE_ID, right_deg);

            /*
             * 其他控制示例 (按需切换):
             *
             * // 电流控制:
             * motor_hal_set_torque(hal, LEFT_NODE_ID, 2000);  // 2000mA
             *
             * // 速度控制:
             * motor_hal_set_velocity(hal, LEFT_NODE_ID, 500.0f); // 500RPM
             *
             * // CSP 同步位置:
             * motor_hal_ctrl_raw(hal, LEFT_NODE_ID, MOTOR_MODE_CSP,
             *                    motor_deg_to_counts(left_deg), 0, 0);
             *
             * // MIT 阻抗:
             * motor_hal_mit_control(hal, LEFT_NODE_ID, 30.0f, 0.0f, 0.5f, 0.1f, 0.0f);
             */
        }

        /* ── 打印反馈 (低频) ── */
        if (now - last_print >= 200000) {  /* 200ms ,  5Hz */
            last_print = now;
            float t = (now - t_start) * 1e-6f;
            print_feedback(t);
        }
    }

    /* ============================================================
     * Phase 6: 安全停机
     * ============================================================ */
    printf("\n=== Shutdown ===\n");

    /* 6a. 停止运动 */
    printf("  Stopping motors...\n");
    motor_hal_stop(hal, LEFT_NODE_ID);
    motor_hal_stop(hal, RIGHT_NODE_ID);
    usleep(100000);

    /* 6b. 脱使能 (DS402: Shutdown) */
    printf("  Disabling motors...\n");
    ret = motor_hal_disable(hal, LEFT_NODE_ID);
    printf("  Left motor disable: %s\n", ret == 0 ? "OK" : "FAIL");
    ret = motor_hal_disable(hal, RIGHT_NODE_ID);
    printf("  Right motor disable: %s\n", ret == 0 ? "OK" : "FAIL");

    /* 6c. NMT 停止 (广播) */
    printf("  NMT Stop all nodes...\n");
    motor_hal_nmt_broadcast(hal, NMT_CMD_STOP);

    /* 6d. 清理 */
    printf("\n  Left feedback count:  %d\n", g_left.update_count);
    printf("  Right feedback count: %d\n", g_right.update_count);

    motor_hal_destroy(hal);
    printf("  ✓ HAL destroyed\n");
    printf("\nDone.\n");

    return 0;
}
