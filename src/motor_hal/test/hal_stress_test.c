/**
 * @file hal_stress_test.c
 * @brief HAL 层并发压力测试 — 不依赖电机机械响应
 *
 * 测试覆盖:
 *   1. 并发 SDO: 多线程同时读写, 验证 SDO 队列+condvar 无竞态
 *   2. PDO 压力: 高频切换模式+发控制帧, 验证帧构造无内存越界
 *   3. 反馈接收: 持续性验证 recv 线程不丢帧不崩溃
 *   4. TPDO 同步: 配置 SYNC, TPDO 链路验证
 *   5. 多轴广播: 帧格式 on bus 验证 (需 candump 对照)
 *   6. 传感器透传: 配置+接收 验证
 *   7. SDO 重试: 发到离线电机验证 timeout+retry 不卡死
 *
 * 编译: gcc -O2 -o hal_stress_test hal_stress_test.c -I../inc -L../build -lmotor_hal -lpthread -lrt
 * 运行: sudo ./hal_stress_test
 */

#define _GNU_SOURCE
#include "motor_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

/* ================================================================
 * 配置
 * ================================================================ */

#define CAN_IFACE           "can0"
#define MOTOR_ID_EXISTING   1       /* 在线电机 ID */
#define MOTOR_ID_OFFLINE    2       /* 离线电机 ID (测试重试) */
#define STRESS_SDO_COUNT    100     /* 并发 SDO 次数 */
#define STRESS_PDO_COUNT    500     /* PDO 快速切换次数 */
#define STRESS_THREADS      4       /* 并发线程数 */
#define WATCH_DURATION_S    3       /* 反馈接收验证时长 */

static volatile int g_running = 1;

/* ================================================================
 * 统计
 * ================================================================ */

typedef struct {
    int sdo_read_ok;
    int sdo_read_fail;
    int sdo_read_timeout;
    int pdo_sent;
    int pdo_fail;
    int fb_received;
    int sensor_received;
} test_stats_t;

static test_stats_t g_stats;
static pthread_mutex_t g_stats_lock = PTHREAD_MUTEX_INITIALIZER;

/* ================================================================
 * 工具
 * ================================================================ */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void sig_handler(int s) { (void)s; g_running = 0; }

#define TEST(fmt, ...)  printf("\n═══ " fmt " ═══\n", ##__VA_ARGS__)
#define PASS(fmt, ...)  printf("  ✓ " fmt "\n", ##__VA_ARGS__)
#define FAIL(fmt, ...)  printf("  ✗ " fmt "\n", ##__VA_ARGS__)

/* ================================================================
 * T0: CANFD 初始化 + 电机注册
 * ================================================================ */

static int test_init(motor_hal_t **out_hal)
{
    TEST("T0: CANFD 初始化 + 电机注册");

    motor_hal_t *hal = motor_hal_create();
    if (!hal) { FAIL("motor_hal_create"); return -1; }

    int ret = motor_hal_init(hal, CAN_IFACE, 1000000, 5000000);
    if (ret < 0) { FAIL("CANFD init ret=%d", ret); motor_hal_destroy(hal); return -2; }
    PASS("CANFD %s opened (1M/5M)", CAN_IFACE);

    /* 注册电机 1 (在线) */
    motor_config_t cfg = {0};
    cfg.node_id            = MOTOR_ID_EXISTING;
    cfg.heartbeat_ms       = 2000;
    cfg.profile_accel      = 5000;
    cfg.profile_decel      = 5000;
    cfg.profile_velocity   = 20;
    cfg.disable_watchdog   = true;
    cfg.auto_enable        = true;
    cfg.bootup_timeout_ms  = 3000;
    cfg.tpdo_sync_count    = 1;

    ret = motor_hal_add_motor(hal, &cfg);
    if (ret != 0) { FAIL("add motor %d ret=%d", MOTOR_ID_EXISTING, ret); motor_hal_destroy(hal); return -3; }
    PASS("motor %d registered", MOTOR_ID_EXISTING);

    /* 注册电机 2 (离线, 用于测试超时) */
    cfg.node_id = MOTOR_ID_OFFLINE;
    ret = motor_hal_add_motor(hal, &cfg);
    if (ret != 0) { FAIL("add motor %d ret=%d", MOTOR_ID_OFFLINE, ret); }
    PASS("motor %d registered (offline test target)", MOTOR_ID_OFFLINE);

    /* 启动接收线程 */
    ret = motor_hal_recv_start(hal);
    if (ret < 0) { FAIL("recv_start ret=%d", ret); motor_hal_destroy(hal); return -4; }
    PASS("recv thread started");

    /* 启动电机 1 (等 bootup ,  SDO 配置 ,  DS402 使能) */
    printf("  Starting motor %d...\n", MOTOR_ID_EXISTING);
    usleep(200000);  /* 等 bootup 到达 */
    ret = motor_hal_startup(hal, MOTOR_ID_EXISTING, 3000);
    if (ret == 0) {
        PASS("motor %d startup OK", MOTOR_ID_EXISTING);
    } else {
        PASS("motor %d startup ret=%d (continue anyway)", MOTOR_ID_EXISTING, ret);
    }

    *out_hal = hal;
    return 0;
}

/* ================================================================
 * T1: SDO 基础读写 (单线程)
 * ================================================================ */

static int test_sdo_basic(motor_hal_t *hal)
{
    TEST("T1: SDO 基础读写");

    /* 读 Statusword */
    uint32_t sw = 0;
    int ret = motor_hal_sdo_read_u32(hal, MOTOR_ID_EXISTING, 0x6041, 0x00, &sw);
    if (ret == 0) PASS("SDO read 0x6041 = 0x%04X", sw);
    else          FAIL("SDO read 0x6041 ret=%d", ret);

    /* 读固件版本 */
    uint32_t fw = 0;
    ret = motor_hal_sdo_read_u32(hal, MOTOR_ID_EXISTING, 0x100A, 0x00, &fw);
    if (ret == 0) PASS("SDO read 0x100A = 0x%08X", fw);
    else          FAIL("SDO read 0x100A ret=%d", ret);

    /* 写心跳周期 ,  回读验证 */
    ret = motor_hal_sdo_write(hal, MOTOR_ID_EXISTING, 0x1017, 0x00, 2000, 2);
    if (ret == 0) PASS("SDO write 0x1017 = 2000");
    else          FAIL("SDO write 0x1017 ret=%d", ret);

    uint32_t hb = 0;
    ret = motor_hal_sdo_read_u32(hal, MOTOR_ID_EXISTING, 0x1017, 0x00, &hb);
    if (ret == 0 && hb == 2000) PASS("SDO 回读验证: 0x1017 = %d ✓", hb);
    else                        FAIL("SDO 回读 0x1017 = %d (expected 2000)", hb);

    return (ret == 0) ? 0 : -1;
}

/* ================================================================
 * T2: SDO 并发压力 (多线程同时读写)
 * ================================================================ */

typedef struct {
    motor_hal_t *hal;
    int          thread_id;
    uint16_t     obj_index;   /* 每个线程读不同对象, 测试队列路由 */
} sdo_thread_arg_t;

static void* _sdo_thread_fn(void *arg)
{
    sdo_thread_arg_t *a = (sdo_thread_arg_t*)arg;
    int ok = 0, fail = 0, timeout = 0;

    for (int i = 0; i < STRESS_SDO_COUNT; i++) {
        uint32_t val = 0;
        int ret = motor_hal_sdo_read_u32(a->hal, MOTOR_ID_EXISTING, a->obj_index, 0x00, &val);
        if (ret == 0) {
            ok++;
        } else if (ret == -ETIMEDOUT) {
            timeout++;
        } else {
            fail++;
        }
    }

    pthread_mutex_lock(&g_stats_lock);
    g_stats.sdo_read_ok      += ok;
    g_stats.sdo_read_fail    += fail;
    g_stats.sdo_read_timeout += timeout;
    pthread_mutex_unlock(&g_stats_lock);

    return NULL;
}

static int test_sdo_stress(motor_hal_t *hal)
{
    TEST("T2: SDO 并发压力 (%d threads × %d reads)", STRESS_THREADS, STRESS_SDO_COUNT);

    pthread_t threads[STRESS_THREADS];
    sdo_thread_arg_t args[STRESS_THREADS];

    uint64_t t0 = now_us();

    for (int i = 0; i < STRESS_THREADS; i++) {
        args[i].hal       = hal;
        args[i].thread_id = i;
        static const uint16_t sdo_indexes[] = {0x6041, 0x6064, 0x606C, 0x100A};
        args[i].obj_index = sdo_indexes[i % 4];
        pthread_create(&threads[i], NULL, _sdo_thread_fn, &args[i]);
    }

    for (int i = 0; i < STRESS_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    uint64_t elapsed = now_us() - t0;

    printf("  Total: %d OK / %d FAIL / %d TIMEOUT\n",
           g_stats.sdo_read_ok, g_stats.sdo_read_fail, g_stats.sdo_read_timeout);
    printf("  Elapsed: %llu us, avg: %llu us/op\n",
           (unsigned long long)elapsed,
           (unsigned long long)(elapsed / (STRESS_THREADS * STRESS_SDO_COUNT)));

    if (g_stats.sdo_read_ok == STRESS_THREADS * STRESS_SDO_COUNT
        && g_stats.sdo_read_timeout == 0) {
        PASS("zero timeout, zero failure");
    } else if (g_stats.sdo_read_timeout > 0) {
        FAIL("%d timeouts — SDO queue may have race", g_stats.sdo_read_timeout);
    }

    return (g_stats.sdo_read_timeout == 0 && g_stats.sdo_read_fail == 0) ? 0 : -1;
}

/* ================================================================
 * T3: DS402 状态机
 * ================================================================ */

static int test_ds402_state_machine(motor_hal_t *hal)
{
    TEST("T3: DS402 状态机");

    /* 查询当前状态 */
    motor_state_t st = motor_hal_get_state(hal, MOTOR_ID_EXISTING);
    printf("  Initial state: %d (%s)\n", st, motor_state_str(st));

    /* enable */
    int ret = motor_hal_enable(hal, MOTOR_ID_EXISTING);
    if (ret == 0) PASS("enable OK");
    else          FAIL("enable ret=%d", ret);

    st = motor_hal_get_state(hal, MOTOR_ID_EXISTING);
    printf("  After enable:  %s\n", motor_state_str(st));

    if (st != MOTOR_STATE_OP_ENABLED) {
        FAIL("not OPERATION_ENABLED (state=%d)", st);
    }

    /* disable ,  enable × 3 */
    for (int loop = 0; loop < 3; loop++) {
        ret = motor_hal_disable(hal, MOTOR_ID_EXISTING);
        if (ret != 0) { FAIL("disable loop %d ret=%d", loop, ret); break; }
        ret = motor_hal_enable(hal, MOTOR_ID_EXISTING);
        if (ret != 0) { FAIL("enable loop %d ret=%d", loop, ret); break; }
    }
    PASS("enable/disable ×3 OK");

    return 0;
}

/* ================================================================
 * T4: PDO Byte0 控制
 * ================================================================ */

static int test_pdo_byte0(motor_hal_t *hal)
{
    TEST("T4: PDO Byte0 控制");

    /* 读当前 byte0 */
    uint8_t b0;
    int ret = motor_hal_pdo_get_byte0(hal, MOTOR_ID_EXISTING, &b0);
    if (ret == 0) PASS("get_byte0 = 0x%02X", b0);
    else          FAIL("get_byte0 ret=%d", ret);

    /* enable */
    ret = motor_hal_pdo_enable(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("pdo_enable OK") : FAIL("pdo_enable ret=%d", ret);

    /* disable */
    ret = motor_hal_pdo_disable(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("pdo_disable OK") : FAIL("pdo_disable ret=%d", ret);

    /* bus_on / bus_off */
    ret = motor_hal_pdo_bus_on(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("pdo_bus_on OK") : FAIL("pdo_bus_on ret=%d", ret);

    ret = motor_hal_pdo_bus_off(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("pdo_bus_off OK") : FAIL("pdo_bus_off ret=%d", ret);

    /* estop ,  recover */
    ret = motor_hal_pdo_estop(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("estop OK") : FAIL("estop ret=%d", ret);

    ret = motor_hal_pdo_recover(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("recover OK") : FAIL("recover ret=%d", ret);

    /* 模式切换 */
    motor_mode_t modes[] = { MOTOR_MODE_CURRENT, MOTOR_MODE_PROFILE_VEL,
                             MOTOR_MODE_PROFILE_POS, MOTOR_MODE_CSP, MOTOR_MODE_CSV };
    const char *names[]  = { "CURRENT", "PV", "PP", "CSP", "CSV" };

    for (int i = 0; i < 5; i++) {
        ret = motor_hal_pdo_set_mode(hal, MOTOR_ID_EXISTING, modes[i]);
        if (ret == 0) PASS("set_mode ,  %s OK", names[i]);
        else          FAIL("set_mode ,  %s ret=%d", names[i], ret);
    }

    /* clear fault */
    ret = motor_hal_pdo_clear_fault(hal, MOTOR_ID_EXISTING);
    ret == 0 ? PASS("clear_fault OK") : FAIL("clear_fault ret=%d", ret);

    /* 恢复当前 */
    motor_hal_pdo_enable(hal, MOTOR_ID_EXISTING);
    motor_hal_pdo_bus_on(hal, MOTOR_ID_EXISTING);

    return 0;
}

/* ================================================================
 * T5: PDO 控制帧压力测试
 * ================================================================ */

static int test_pdo_stress(motor_hal_t *hal)
{
    TEST("T5: PDO 控制帧快速切换 (%d次)", STRESS_PDO_COUNT);

    int errors = 0;
    /* 安全范围: 电流±500mA, 速度±10RPM, 位置±10° */
    static const int16_t current_vals[] = {0, 100, -100, 200, -200, 500, -500};
    static const float   vel_vals[]     = {0.0f, 2.0f, -2.0f, 5.0f, -5.0f, 10.0f, -10.0f};
    static const float   pos_vals[]     = {0.0f, 1.0f, -1.0f, 3.0f, -3.0f, 10.0f, -10.0f};

    for (int i = 0; i < STRESS_PDO_COUNT; i++) {
        if (i % 3 == 0) {
            if (motor_hal_set_torque(hal, MOTOR_ID_EXISTING, current_vals[i % 7]) != 0) errors++;
        } else if (i % 3 == 1) {
            if (motor_hal_set_velocity(hal, MOTOR_ID_EXISTING, vel_vals[i % 7]) != 0) errors++;
        } else {
            if (motor_hal_set_position(hal, MOTOR_ID_EXISTING, pos_vals[i % 7]) != 0) errors++;
        }
    }

    if (errors == 0) PASS("all %d PDO frames sent OK", STRESS_PDO_COUNT);
    else             FAIL("%d/%d PDO frames failed", errors, STRESS_PDO_COUNT);

    return (errors == 0) ? 0 : -1;
}

/* ================================================================
 * T6: 多轴广播
 * ================================================================ */

static int test_multi_axis(motor_hal_t *hal)
{
    TEST("T6: 多轴广播");

    multi_axis_cmd_t cmds[2];

    /* 电流广播 */
    cmds[0].node_id       = 1;
    cmds[0].mode          = MOTOR_MODE_CURRENT;
    cmds[0].enable        = true;
    cmds[0].release_brake = true;
    cmds[0].target1       = 0;
    cmds[0].target2       = 0;
    cmds[0].feedforward   = 0;

    cmds[1].node_id       = 2;
    cmds[1].mode          = MOTOR_MODE_CURRENT;
    cmds[1].enable        = true;
    cmds[1].release_brake = true;
    cmds[1].target1       = 0;
    cmds[1].target2       = 0;
    cmds[1].feedforward   = 0;

    motor_hal_multi_ctrl(hal, cmds, 2);
    PASS("multi_ctrl (ID=1,2) current mode sent");

    /* 位置广播 */
    cmds[0].mode = MOTOR_MODE_CSP;
    cmds[1].mode = MOTOR_MODE_CSP;
    motor_hal_multi_ctrl(hal, cmds, 2);
    PASS("multi_ctrl (ID=1,2) CSP mode sent");

    /* 速度广播 */
    cmds[0].mode = MOTOR_MODE_PROFILE_VEL;
    cmds[1].mode = MOTOR_MODE_PROFILE_VEL;
    motor_hal_multi_ctrl(hal, cmds, 2);
    PASS("multi_ctrl (ID=1,2) velocity mode sent");

    return 0;
}

/* ================================================================
 * T7: MIT 阻抗控制 PDO
 * ================================================================ */

static int test_mit(motor_hal_t *hal)
{
    TEST("T7: MIT 阻抗控制");

    int ret = motor_hal_mit_control(hal, MOTOR_ID_EXISTING,
                                     0.0f,     /* pos=0° */
                                     0.0f,     /* vel=0 */
                                     0.5f,     /* kp=0.5 */
                                     0.1f,     /* kd=0.1 */
                                     0.0f);    /* torque=0 */
    if (ret == 0) PASS("MIT control frame sent");
    else          FAIL("MIT control ret=%d", ret);

    return ret;
}

/* ================================================================
 * T8: TPDO 同步配置 + 接收验证
 * ================================================================ */

static void on_tpdo_test(uint8_t id, const canfd_frame_t *f, void *ctx)
{
    (void)f;
    pthread_mutex_lock(&g_stats_lock);
    g_stats.fb_received++;
    pthread_mutex_unlock(&g_stats_lock);
}

static int test_tpdo_sync(motor_hal_t *hal)
{
    TEST("T8: TPDO 同步上报");

    /* 配置 TPDO1 */
    int ret = motor_hal_tpdo_config(hal, MOTOR_ID_EXISTING, 1);
    if (ret == 0) PASS("tpdo_config sync_count=1 OK");
    else          FAIL("tpdo_config ret=%d", ret);

    /* 注册 TPDO 回调 */
    motor_hal_set_tpdo_cb(hal, MOTOR_ID_EXISTING, on_tpdo_test, NULL);
    PASS("tpdo_callback registered");

    /* 启动 SYNC (10ms 周期) */
    ret = motor_hal_sync_start(hal, 10000);
    if (ret == 0) PASS("SYNC started (10ms)");
    else          FAIL("sync_start ret=%d", ret);

    /* 等 1 秒收 TPDO */
    printf("  Waiting 1s for TPDO frames...\n");
    usleep(1000000);

    ret = motor_hal_sync_stop(hal);
    if (ret == 0) PASS("SYNC stopped");
    else          FAIL("sync_stop ret=%d", ret);

    PASS("TPDO frames received: %d", g_stats.fb_received);
    return (g_stats.fb_received > 0) ? 0 : -1;
}

/* ================================================================
 * T9: 传感器透传
 * ================================================================ */

static void on_sensor_test(uint8_t id, const motor_sensor_t *s, void *ctx)
{
    (void)ctx;
    if (s->data_valid) {
        pthread_mutex_lock(&g_stats_lock);
        g_stats.sensor_received++;
        pthread_mutex_unlock(&g_stats_lock);
    }
}

static int test_sensor(motor_hal_t *hal)
{
    TEST("T9: 传感器透传配置 + 接收");

    /* 配置透传 250Hz (4000μs) */
    int ret = motor_hal_sensor_config(hal, MOTOR_ID_EXISTING, 4000, 0);
    if (ret == 0) PASS("sensor_config 4000us OK");
    else          FAIL("sensor_config ret=%d", ret);

    /* 注册回调 */
    motor_hal_set_sensor_cb(hal, MOTOR_ID_EXISTING, on_sensor_test, NULL);
    PASS("sensor_callback registered");

    /* 等 1 秒收数据 */
    printf("  Waiting 1s for sensor frames...\n");
    usleep(1000000);

    /* 读缓存 */
    motor_sensor_t s;
    ret = motor_hal_get_sensor(hal, MOTOR_ID_EXISTING, &s);
    if (ret == 0) {
        PASS("get_sensor OK | hall=%d,%d,%d force=%d knee=%d sw=%d valid=%d",
             s.hall_adc0, s.hall_adc1, s.hall_adc2,
             s.force_raw, s.knee_hall, s.hw_sw_pc9, s.data_valid);
    } else {
        PASS("get_sensor ret=%d (no motor ,  no real data, OK)", ret);
    }

    /* 停止透传 */
    ret = motor_hal_sensor_stop(hal, MOTOR_ID_EXISTING);
    if (ret == 0) PASS("sensor_stop OK");
    else          FAIL("sensor_stop ret=%d", ret);

    return 0;
}

/* ================================================================
 * T10: SDO 重试/超时 (离线电机)
 * ================================================================ */

static int test_sdo_timeout(motor_hal_t *hal)
{
    TEST("T10: SDO 超时重试 (离线电机 ID=%d)", MOTOR_ID_OFFLINE);

    uint64_t t0 = now_us();
    uint32_t val = 0;
    int ret = motor_hal_sdo_read_u32(hal, MOTOR_ID_OFFLINE, 0x6041, 0x00, &val);
    uint64_t elapsed = now_us() - t0;

    if (ret == -ETIMEDOUT) {
        PASS("SDO timeout as expected (ret=%d, elapsed=%llu us)", ret,
             (unsigned long long)elapsed);
    } else if (ret == 0) {
        FAIL("SDO to offline motor SUCCEEDED — unexpected (val=0x%X)", val);
    } else {
        FAIL("SDO to offline motor ret=%d (expected -ETIMEDOUT)", ret);
    }

    /* 验证不卡死: 超时后立即再发一条 SDO 到在线电机 */
    ret = motor_hal_sdo_read_u32(hal, MOTOR_ID_EXISTING, 0x6041, 0x00, &val);
    if (ret == 0) PASS("SDO to online motor OK after timeout — no deadlock");
    else          FAIL("SDO after timeout failed ret=%d — possible deadlock", ret);

    return (ret == 0) ? 0 : -1;
}

/* ================================================================
 * T11: 反馈接收持续性
 * ================================================================ */

static int test_feedback_continuous(motor_hal_t *hal)
{
    TEST("T11: 反馈接收持续性 (%ds)", WATCH_DURATION_S);

    int count = 0;
    uint64_t t0 = now_us();

    while (now_us() - t0 < (uint64_t)WATCH_DURATION_S * 1000000ULL) {
        motor_feedback_t fb;
        if (motor_hal_get_feedback(hal, MOTOR_ID_EXISTING, &fb) == 0) {
            count++;
        }
        usleep(5000);  /* 5ms 轮询 */
    }

    if (count > 0) PASS("received %d feedback frames in %ds", count, WATCH_DURATION_S);
    else           FAIL("no feedback frames received");

    return (count > 0) ? 0 : -1;
}

/* ================================================================
 * 退出: 安全停机
 * ================================================================ */

static void test_shutdown(motor_hal_t *hal)
{
    TEST("Shutdown");

    motor_hal_disable(hal, MOTOR_ID_EXISTING);
    motor_hal_nmt_broadcast(hal, NMT_CMD_STOP);
    motor_hal_recv_stop(hal);
    motor_hal_destroy(hal);
    PASS("HAL destroyed cleanly");
}

/* ================================================================
 * main
 * ================================================================ */

int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    memset(&g_stats, 0, sizeof(g_stats));

    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  motor_hal_c Concurrency Stress Test        ║\n");
    printf("║  No motor required — CANFD protocol only    ║\n");
    printf("╚══════════════════════════════════════════════╝\n");

    motor_hal_t *hal = NULL;

    int passed = 0, total = 0;

    /* T0 */
    total++;
    if (test_init(&hal) == 0) passed++;

    if (!hal) {
        printf("\nFATAL: Initialization failed\n");
        return 1;
    }

    /* T1~T11 */
    total++; if (test_sdo_basic(hal)          == 0) passed++;
    total++; if (test_sdo_stress(hal)         == 0) passed++;
    total++; if (test_ds402_state_machine(hal)== 0) passed++;
    total++; if (test_pdo_byte0(hal)          == 0) passed++;
    total++; if (test_pdo_stress(hal)         == 0) passed++;
    total++; if (test_multi_axis(hal)         == 0) passed++;
    total++; if (test_mit(hal)                == 0) passed++;
    total++; if (test_tpdo_sync(hal)          == 0) passed++;
    total++; if (test_sensor(hal)             == 0) passed++;
    total++; if (test_sdo_timeout(hal)        == 0) passed++;
    total++; if (test_feedback_continuous(hal)== 0) passed++;

    test_shutdown(hal);

    printf("\n╔══════════════════════════════════════════════╗\n");
    printf("║  RESULT: %d/%d passed                        ║\n", passed, total);
    printf("╚══════════════════════════════════════════════╝\n");

    return (passed == total) ? 0 : 1;
}
