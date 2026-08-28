/**
 * @file motor_hal.c
 * @brief 关节模组 HAL 层 - 核心实现
 *
 * 所有公共 API 的入口, 内部管理电机列表和 CAN 帧分发。
 *
 * 依赖模块:
 *   can_driver      ,  SocketCAN 驱动
 *   sdo_client      ,  SDO 读写
 *   pdo_handler     ,  PDO 发送
 *   feedback_parser ,  反馈解析
 *   nmt_master      ,  NMT 命令
 *   heartbeat       ,  心跳管理
 *   motor_hal_startup ,  启动流程
 *   utils           ,  工具函数
 */

#define _GNU_SOURCE
#include "motor_hal.h"

#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include "pdo_mapper.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>
#include <stdio.h>

/* ---------- 诊断: CAN 帧 hex dump (仅调试模式) ---------- */
#ifdef MOTOR_DEBUG_HEX
static void _dump_can_frame(const char *dir, const canfd_frame_t *f)
{
    fprintf(stderr, "[%s] id=0x%03X dlc=%d :", dir, f->id, f->dlc);
    for (int i = 0; i < f->dlc && i < 64; i++) {
        fprintf(stderr, " %02X", f->data[i]);
    }
    fprintf(stderr, "\n");
}
#else
static inline void _dump_can_frame(const char *dir, const canfd_frame_t *f) { (void)dir; (void)f; }
#endif

/* ---------- 协议验收日志桥接 (C → C++ ECO_INFO_NEW) ---------- */
static motor_hal_log_cb_t g_proto_log_cb = NULL;
static int g_proto_recv_cnt[16] = {0};  /* max MOTOR_HAL_MAX_MOTORS */
#define PROTO_RECV_EVERY_N  50   /* 接收帧每 N 帧打印一次 */

void motor_hal_set_log_callback(motor_hal_log_cb_t cb) { g_proto_log_cb = cb; }
void motor_hal_clear_log_callback(void)                { g_proto_log_cb = NULL; }

/* 发送路径: 每次都打印 (频率由调用方控制) */
#define PROTO_SEND(fmt, ...) do { \
    if (g_proto_log_cb) { \
        char _pbuf[320]; \
        snprintf(_pbuf, sizeof(_pbuf), fmt, ##__VA_ARGS__); \
        g_proto_log_cb(_pbuf); \
    } \
} while(0)

/* 接收路径: 降频打印 (每 PROTO_RECV_EVERY_N 帧一次) */
#define PROTO_RECV(node_id, fmt, ...) do { \
    if (g_proto_log_cb && (node_id) >= 1 && (node_id) <= 16 && \
        (++g_proto_recv_cnt[(node_id)-1] % PROTO_RECV_EVERY_N == 0)) { \
        char _rbuf[320]; \
        snprintf(_rbuf, sizeof(_rbuf), fmt, ##__VA_ARGS__); \
        g_proto_log_cb(_rbuf); \
    } \
} while(0)

/* =====================================================
 * 内部: 前向声明拆分模块函数
 * ===================================================== */

/* can_driver.c */
/* (通过 can_driver_internal.h) */

/* sdo_client.c */
/* (通过 sdo_client_internal.h) */

/* nmt_master.c */
void nmt_send(can_driver_t *drv, uint8_t cmd, uint8_t node);
void nmt_broadcast(can_driver_t *drv, uint8_t cmd);

/* heartbeat.c */
void heartbeat_set_period(can_driver_t *drv, uint8_t node, uint32_t ms);
void heartbeat_disable_watchdog(can_driver_t *drv, uint8_t node);
void heartbeat_feed(can_driver_t *drv);

/* pdo_handler.c */
void pdo_ctrl_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                   bool enable, bool release_brake, bool clear_err,
                   int16_t target1, uint16_t target2, int16_t feedforward);
void pdo_ctrl_send_raw(can_driver_t *drv, uint8_t node, uint8_t byte0,
                        int16_t target1, uint16_t target2, int16_t feedforward);
void pdo_mit_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                  bool enable, bool release_brake, bool clear_err,
                  uint16_t position, uint16_t velocity,
                  uint16_t kp, uint16_t kd, int16_t torque);
void pdo_mit_send_raw(can_driver_t *drv, uint8_t node, uint8_t byte0,
                       uint16_t position, uint16_t velocity,
                       uint16_t kp, uint16_t kd, int16_t torque);
void pdo_multi_send(can_driver_t *drv, const multi_axis_cmd_t *cmds, uint8_t count);
void pdo_mit_multi_send(can_driver_t *drv, const multi_mit_cmd_t *cmds, uint8_t count);
void pdo_sync_send(can_driver_t *drv);
void pdo_feedback_parse(const canfd_frame_t *f, motor_feedback_t *fb);

/* feedback_parser.c */
const char* feedback_error_string(uint16_t err);

/* motor_hal_startup.c */
int motor_startup_wait_bootup(can_driver_t *drv, uint8_t node_id, int timeout_ms,
                              const volatile bool *bootup_flag);
int motor_startup_enable(can_driver_t *drv, uint8_t node_id);
int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg,
                       const volatile bool *bootup_flag);

/* utils.c */
uint64_t motor_utils_now_us(void);
void motor_utils_sleep_ms(int ms);

/* =====================================================
 * 单电机状态
 * ===================================================== */

typedef struct {
    uint8_t     node_id;
    motor_state_t state;
    bool        enabled;
    bool        bootup_received;
    bool        pending_startup;   /* auto_enable 触发的待处理启动, 由主线程消费 */
    motor_config_t config;

    /* 反馈缓存 */
    pthread_mutex_t fb_lock;
    motor_feedback_t cached_fb;
    uint64_t last_fb_us;

    /* 回调 */
    motor_feedback_cb_t fb_cb;
    motor_error_cb_t    err_cb;
    motor_state_cb_t    state_cb;
    motor_sensor_cb_t   sensor_cb;
    motor_tpdo_raw_cb_t tpdo_raw_cb;  /* 标准 TPDO 原始帧回调 */
    void               *fb_ctx;
    void               *err_ctx;
    void               *state_ctx;
    void               *sensor_ctx;
    void               *tpdo_raw_ctx;

    /* 传感器缓存 */
    pthread_mutex_t  sensor_lock;
    motor_sensor_t   cached_sensor;
    uint64_t         last_sensor_us;

    /* SDO telemetry cache (0x300 frame: only Iq valid on RV1126B, temp/pos from SDO) */
    int32_t  sdo_temp_01c;      /* 0x2663 temperature, 0.1°C, -1=not yet polled */
    int32_t  sdo_position;      /* 0x6064 position, counts, valid when sdo_temp_01c >= 0 */

    /* MIT 快控缩放 (启动时从 OD 读取, 运行中不变) */
    mit_scales_t mit_scales;

    /* PDO Byte0 — 仅由 PDO API 管理, SDO 不碰 */
    uint8_t  pdo_byte0;         /* Byte0 持久值, 默认 0x00 */
    bool     clr_err_pending;   /* bit5 脉冲标志 */

    /* heartbeat 状态缓存 — 只在变化时打印 */
    uint8_t  last_nmt_state;    /* 上次心跳 NMT 状态 */
} motor_node_t;

/* SDO telemetry polling thread state */
static pthread_t sdo_telemetry_thread;
static bool      sdo_telemetry_running = false;

/* =====================================================
 * HAL 主结构
 * ===================================================== */

struct motor_hal {
    can_driver_t *drv;
    bool          initialized;

    /* 接收线程 */
    pthread_t    recv_thread;
    bool         recv_running;
    bool         recv_rt_enable;
    int          recv_rt_priority;
    int          recv_cpu;           /* 接收线程绑核 (-1=不绑) */

    /* SYNC 定时器线程 */
    pthread_t    sync_thread;
    bool         sync_running;
    bool         sync_rt_enable;
    int          sync_rt_priority;
    int          sync_cpu;          /* SYNC 线程绑核 (-1=不绑) */
    uint32_t     sync_period_us;

    pthread_mutex_t lock;
    motor_node_t    motors[MOTOR_HAL_MAX_MOTORS];
    int             motor_count;
};

/* =====================================================
 * 内部: 查找电机
 * ===================================================== */

static motor_node_t* _find_motor(motor_hal_t *hal, uint8_t node_id)
{
    for (int i = 0; i < hal->motor_count; i++) {
        if (hal->motors[i].node_id == node_id)
            return &hal->motors[i];
    }
    return NULL;
}

/* =====================================================
 * 内部: 消费 pdo_byte0 + clr_err 脉冲自动清除 (锁内调用)
 * ===================================================== */

static uint8_t _consume_pdo_byte0(motor_node_t *m)
{
    uint8_t b0 = m->pdo_byte0;
    if (m->clr_err_pending) {
        b0 &= ~PDO_BYTE0_CLR_ERR;
        m->pdo_byte0 = b0;
        m->clr_err_pending = false;
    }
    return b0;
}

/* =====================================================
 * 内部: 状态迁移
 * ===================================================== */

static void _set_state(motor_hal_t *hal __attribute__((unused)), motor_node_t *m, motor_state_t new_state)
{
    motor_state_t old = m->state;
    m->state = new_state;
    if (m->state_cb) {
        m->state_cb(m->node_id, old, new_state, m->state_ctx);
    }
}

/* =====================================================
 * 公共 API: 生命周期
 * ===================================================== */

motor_hal_t* motor_hal_create(void)
{
    motor_hal_t *hal = calloc(1, sizeof(motor_hal_t));
    if (!hal) return NULL;
    hal->recv_cpu = -1;
    hal->sync_cpu = -1;

    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&hal->lock, &ma);
    hal->motor_count = 0;

    for (int i = 0; i < MOTOR_HAL_MAX_MOTORS; i++) {
        hal->motors[i].node_id = 0;
        hal->motors[i].state = MOTOR_STATE_NOT_READY;
        pthread_mutex_init(&hal->motors[i].fb_lock, &ma);
        pthread_mutex_init(&hal->motors[i].sensor_lock, &ma);
    }
    pthread_mutexattr_destroy(&ma);

    /* 初始化 SDO 响应队列 */
    sdo_queue_init();

    return hal;
}

void motor_hal_destroy(motor_hal_t *hal)
{
    if (!hal) return;

    /* 不发 SDO CW_DISABLE_VOLTAGE:
     *   该命令会让驱动板进入 Stopped 状态,
     *   之后 NMT Start/Reset 都不响应, 只能断电重启。
     *   直接关 CAN 接口, 驱动板保持当前状态,
     *   下次 daemon 重启后 startup 正常工作。 */

    /* 1. 停止接收线程 */
    if (hal->recv_running) {
        hal->recv_running = false;
        pthread_join(hal->recv_thread, NULL);
    }

    /* 2. 停止 SYNC 定时器 */
    if (hal->sync_running) {
        hal->sync_running = false;
        pthread_join(hal->sync_thread, NULL);
    }

    /* 3. 销毁 mutex */
    for (int i = 0; i < MOTOR_HAL_MAX_MOTORS; i++) {
        pthread_mutex_destroy(&hal->motors[i].fb_lock);
        pthread_mutex_destroy(&hal->motors[i].sensor_lock);
    }

    /* 4. 关闭 CAN */
    if (hal->drv) {
        can_driver_close(hal->drv);
        hal->drv = NULL;
    }

    /* 5. 销毁 SDO 队列 */
    sdo_queue_destroy();

    pthread_mutex_destroy(&hal->lock);
    free(hal);
}

int motor_hal_init(motor_hal_t *hal, const char *iface,
                   uint32_t arb_bitrate, uint32_t data_bitrate)
{
    if (!hal || hal->initialized) return -EBUSY;

    int ret = can_driver_open(iface, arb_bitrate, data_bitrate, &hal->drv);
    if (ret < 0) return ret;

    hal->initialized = true;
    return 0;
}

/* =====================================================
 * 公共 API: 电机管理
 * ===================================================== */

int motor_hal_add_motor(motor_hal_t *hal, const motor_config_t *cfg)
{
    if (!hal || !cfg) return -EINVAL;

    pthread_mutex_lock(&hal->lock);

    if (hal->motor_count >= MOTOR_HAL_MAX_MOTORS) {
        pthread_mutex_unlock(&hal->lock);
        return -ENOSPC;
    }

    /* 检查重复 */
    if (_find_motor(hal, cfg->node_id)) {
        pthread_mutex_unlock(&hal->lock);
        return -EEXIST;
    }

    motor_node_t *m = &hal->motors[hal->motor_count];
    m->node_id  = cfg->node_id;
    m->state    = MOTOR_STATE_NOT_READY;
    m->enabled  = false;
    m->bootup_received = false;
    m->pending_startup = false;
    m->pdo_byte0       = 0x00;  /* 默认全关, 算法显式调 pdo_enable 才开启 */
    m->clr_err_pending = false;
    m->sdo_temp_01c    = -1;     /* 尚未 SDO 轮询 */
    m->sdo_position    = 0;
    /* 默认 MIT 缩放 (KWS CANFD V2 出厂值), 后续由 motor_hal_read_mit_scales 覆盖 */
    m->mit_scales.pmax  = 3.14f;
    m->mit_scales.vmax  = 3.14f;
    m->mit_scales.kpmax = 50.0f;
    m->mit_scales.kdmax = 50.0f;
    m->mit_scales.tmax  = 20.0f;
    memcpy(&m->config, cfg, sizeof(motor_config_t));

    hal->motor_count++;

    pthread_mutex_unlock(&hal->lock);
    return 0;
}

void motor_hal_remove_motor(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return;

    pthread_mutex_lock(&hal->lock);

    int idx = -1;
    for (int i = 0; i < hal->motor_count; i++) {
        if (hal->motors[i].node_id == node_id) { idx = i; break; }
    }
    if (idx < 0) { pthread_mutex_unlock(&hal->lock); return; }

    pthread_mutex_destroy(&hal->motors[idx].fb_lock);
    pthread_mutex_destroy(&hal->motors[idx].sensor_lock);

    /* 紧凑数组 */
    if (idx < hal->motor_count - 1) {
        memmove(&hal->motors[idx], &hal->motors[idx + 1],
                sizeof(motor_node_t) * (hal->motor_count - idx - 1));
    }
    hal->motor_count--;

    pthread_mutex_unlock(&hal->lock);
}

/* =====================================================
 * 公共 API: 启动 / 关闭
 * ===================================================== */

int motor_hal_startup(motor_hal_t *hal, uint8_t node_id, int timeout_ms __attribute__((unused)))
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }

    /* 用调用者传入的 timeout_ms 覆盖 cfg 中的默认值 */
    m->config.bootup_timeout_ms = timeout_ms;
    int ret = motor_startup_full(hal->drv, &m->config, &m->bootup_received);
    if (ret == 0) {
        m->bootup_received = true;
        m->enabled = true;
        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_wait_bootup(motor_hal_t *hal, uint8_t node_id, int timeout_ms)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    if (m->bootup_received) { pthread_mutex_unlock(&hal->lock); return 0; }

    int ret = motor_startup_wait_bootup(hal->drv, node_id, timeout_ms, &m->bootup_received);
    if (ret == 0) m->bootup_received = true;
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_enable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }

    int ret = motor_startup_enable(hal->drv, node_id);
    if (ret == 0) {
        m->enabled = true;
        _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_disable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }

    int ret = sdo_write_simple(hal->drv, node_id,
                               OD_CONTROLWORD, 0x00, CW_SHUTDOWN, 2);
    if (ret == 0) {
        m->enabled = false;
        _set_state(hal, m, MOTOR_STATE_READY_TO_SW_ON);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_fault_reset(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    int ret = sdo_write_simple(hal->drv, node_id,
                               OD_CONTROLWORD, 0x00, CW_FAULT_RESET, 2);
    if (ret == 0) {
        motor_node_t *m = _find_motor(hal, node_id);
        if (m) _set_state(hal, m, MOTOR_STATE_SWITCH_ON_DIS);
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

/* =====================================================
 * 公共 API: 实时控制
 * ===================================================== */

int motor_hal_set_position(motor_hal_t *hal, uint8_t node_id, float angle_deg)
{
    if (!hal || !hal->drv) return -ENODEV;

    int16_t counts = motor_deg_to_counts(angle_deg);

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint16_t accel = m ? m->config.profile_accel : 0;
    uint8_t b0 = m ? _consume_pdo_byte0(m) : 0;  /* 锁内读, clr_err脉冲自动清除 */
    if (m) { b0 = (b0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(MOTOR_MODE_CSP); m->pdo_byte0 = b0; }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, counts, accel, 0);
    return 0;
}

int motor_hal_set_velocity(motor_hal_t *hal, uint8_t node_id, float rpm_motor)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint16_t accel = m ? m->config.profile_accel : 0;
    uint8_t b0 = m ? _consume_pdo_byte0(m) : 0;
    if (m) { b0 = (b0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(MOTOR_MODE_CSV); m->pdo_byte0 = b0; }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, (int16_t)rpm_motor, accel, 0);
    return 0;
}

int motor_hal_set_torque(motor_hal_t *hal, uint8_t node_id, int16_t current_ma)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _consume_pdo_byte0(m) : 0;
    if (m) { b0 = (b0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(MOTOR_MODE_CURRENT); m->pdo_byte0 = b0; }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, current_ma, 0, 0);
    return 0;
}

int motor_hal_mit_control(motor_hal_t *hal, uint8_t node_id,
                          float position, float velocity,
                          float kp, float kd, float torque)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    uint8_t b0 = _consume_pdo_byte0(m);
    b0 = (b0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(MOTOR_MODE_MIT) | PDO_BYTE0_ENABLE | PDO_BYTE0_BUS_ON;
    m->pdo_byte0 = b0;
    m->enabled = true;
    mit_scales_t scales = m->mit_scales;
    pthread_mutex_unlock(&hal->lock);

    /* 物理量 → raw 编码 (KWS CANFD V2 协议公式) */
    uint16_t pos_raw, vel_raw, kp_raw, kd_raw, tq_raw;
    motor_hal_mit_encode_raw(&scales, (double)position * M_PI / 180.0,
                             (double)velocity * M_PI / 30.0,
                             kp, kd, torque,
                             &pos_raw, &vel_raw, &kp_raw, &kd_raw, &tq_raw);

    /* KWS V2: 全 0 raw 驱动拒收, 发送前校验 */
    if (pos_raw == 0 && vel_raw == 0 && kp_raw == 0 && kd_raw == 0 && tq_raw == 0)
        return -EINVAL;

    /* MIT 日志降频: 每 50 次打印一次完整 9 行 */
    static int _mit_skip[16] = {0};
    if (++_mit_skip[node_id] % 50 == 0) {

    PROTO_SEND("[MIT_SEND] M%d ========== Layer1: phys → raw ==========", node_id);
    PROTO_SEND("[MIT_SEND] M%d   phys: pos=%.1f° vel=%.0fRPM kp=%.1f kd=%.1f tq=%.2fNm",
               node_id, position, velocity, kp, kd, torque);
    PROTO_SEND("[MIT_SEND] M%d   raw:  pr=%u vr=%u kr=%u dr=%u tr=%d",
               node_id, pos_raw, vel_raw, kp_raw, kd_raw, (int16_t)tq_raw);
    PROTO_SEND("[MIT_SEND] M%d   scales: pmax=%.2f vmax=%.2f kpmax=%.0f kdmax=%.0f tmax=%.0f",
               node_id, (double)scales.pmax, (double)scales.vmax,
               (double)scales.kpmax, (double)scales.kdmax, (double)scales.tmax);

    /* ========== Layer2: raw → byte packing ========== */
    uint8_t d0 = b0;
    uint8_t d1 = (uint8_t)((pos_raw >> 8) & 0xFF);
    uint8_t d2 = (uint8_t)(pos_raw & 0xFF);
    uint8_t d3 = (uint8_t)((vel_raw >> 4) & 0xFF);
    uint8_t d4 = (uint8_t)(((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
    uint8_t d5 = (uint8_t)(kp_raw & 0xFF);
    uint8_t d6 = (uint8_t)((kd_raw >> 4) & 0xFF);
    uint8_t d7 = (uint8_t)(((kd_raw & 0x0F) << 4) | (((uint16_t)tq_raw >> 8) & 0x0F));
    uint8_t d8 = (uint8_t)((uint16_t)tq_raw & 0xFF);

    PROTO_SEND("[MIT_SEND] M%d ========== Layer2: byte packing (DLC=9) ==========", node_id);
    PROTO_SEND("[MIT_SEND] M%d   [0]B0=0x%02X | [1-2]pos_H:L=0x%02X:0x%02X (raw=%u)",
               node_id, d0, d1, d2, pos_raw);
    PROTO_SEND("[MIT_SEND] M%d   [3]vel_H=0x%02X | [4]vel_L:kp_H=0x%02X (vel=%u kp=%u)",
               node_id, d3, d4, vel_raw, kp_raw);
    PROTO_SEND("[MIT_SEND] M%d   [5]kp_L=0x%02X | [6]kd_H=0x%02X | [7]kd_L:tq_H=0x%02X (kd=%u)",
               node_id, d5, d6, d7, kd_raw);
    PROTO_SEND("[MIT_SEND] M%d   [8]tq_L=0x%02X (tq_raw=%d)",
               node_id, d8, (int16_t)tq_raw);

    /* ========== Layer3: CAN frame hex dump ========== */
    PROTO_SEND("[MIT_SEND] M%d ========== Layer3: CAN frame (CAN FD + BRS) ==========", node_id);
    PROTO_SEND("[MIT_SEND] M%d   CAN ID=0x%03X DLC=%d FD=1 BRS=1",
               node_id, (uint32_t)(0x110 + node_id), 9);
    PROTO_SEND("[MIT_SEND] M%d   hex: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
               node_id, d0, d1, d2, d3, d4, d5, d6, d7, d8);

    } /* end if (_mit_skip % 50 == 0) */

    pdo_mit_send_raw(hal->drv, node_id, b0, pos_raw, vel_raw, kp_raw, kd_raw, (int16_t)tq_raw);
    return 0;
}

int motor_hal_read_mit_scales(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    static const uint16_t ods[] = {OD_MIT_POS_SCALE, OD_MIT_VEL_SCALE,
                                   OD_MIT_KP_SCALE, OD_MIT_KD_SCALE, OD_MIT_TQ_SCALE};
    float *dests[] = {&m->mit_scales.pmax, &m->mit_scales.vmax,
                      &m->mit_scales.kpmax, &m->mit_scales.kdmax, &m->mit_scales.tmax};
    float defaults[] = {3.14f, 3.14f, 50.0f, 50.0f, 20.0f};

    int err = 0;
    for (int i = 0; i < 5; i++) {
        uint32_t val = 0;
        int ret = sdo_read_simple(hal->drv, node_id, ods[i], 0x00, &val);
        if (ret != 0 || val == 0) {
            *dests[i] = defaults[i];
            err++;
        } else {
            /* 0x2542/0x2543: val × 0.01; 0x2544~0x2546: val 即目标单位 */
            *dests[i] = (i < 2) ? ((float)(int32_t)val * 0.01f) : (float)(int32_t)val;
        }
    }
    return (err == 0) ? 0 : -1;
}

float motor_hal_get_mit_scale(motor_hal_t *hal, uint8_t node_id, int idx)
{
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return 0;
    switch (idx) {
    case 0: return m->mit_scales.pmax;
    case 1: return m->mit_scales.vmax;
    case 2: return m->mit_scales.kpmax;
    case 3: return m->mit_scales.kdmax;
    case 4: return m->mit_scales.tmax;
    default: return 0;
    }
}

void motor_hal_mit_encode_raw(const mit_scales_t *s,
                              float pos_rad, float vel_rads,
                              float kp, float kd, float tau_ff_nm,
                              uint16_t *pos_raw, uint16_t *vel_raw,
                              uint16_t *kp_raw, uint16_t *kd_raw,
                              uint16_t *tq_raw)
{
    /* 对称量: 先限幅再四舍五入 */
    float p = pos_rad;
    if (p < -s->pmax) p = -s->pmax; else if (p > s->pmax) p = s->pmax;
    *pos_raw = (uint16_t)((p + s->pmax) / (2.0f * s->pmax) * 65535.0f + 0.5f);

    float v = vel_rads;
    if (v < -s->vmax) v = -s->vmax; else if (v > s->vmax) v = s->vmax;
    *vel_raw = (uint16_t)((v + s->vmax) / (2.0f * s->vmax) * 4095.0f + 0.5f);

    /* 非对称量: 限幅到 [0, max], 四舍五入 */
    if (kp < 0) kp = 0; else if (kp > s->kpmax) kp = s->kpmax;
    *kp_raw = (uint16_t)(kp / s->kpmax * 4095.0f + 0.5f);

    if (kd < 0) kd = 0; else if (kd > s->kdmax) kd = s->kdmax;
    *kd_raw = (uint16_t)(kd / s->kdmax * 4095.0f + 0.5f);

    /* 对称量: 力矩 */
    float t = tau_ff_nm;
    if (t < -s->tmax) t = -s->tmax; else if (t > s->tmax) t = s->tmax;
    *tq_raw = (uint16_t)((t + s->tmax) / (2.0f * s->tmax) * 4095.0f + 0.5f);
}

void motor_hal_mit_multi_ctrl_phys(motor_hal_t *hal,
    uint8_t id1, float pos1, float vel1, float kp1, float kd1, float tq1,
    uint8_t id2, float pos2, float vel2, float kp2, float kd2, float tq2)
{
    if (!hal || !hal->drv) return;

    multi_mit_cmd_t cmds[2];
    uint8_t count = 0;
    uint8_t ids[] = {id1, id2};
    float pos[]  = {pos1,  pos2};
    float vel[]  = {vel1,  vel2};
    float kps[]  = {kp1,   kp2};
    float kds[]  = {kd1,   kd2};
    float tqs[]  = {tq1,   tq2};

    for (int i = 0; i < 2; i++) {
        motor_node_t *m = _find_motor(hal, ids[i]);
        if (!m || !m->enabled) continue;

        uint16_t pr, vr, kr, dr, tr;
        motor_hal_mit_encode_raw(&m->mit_scales,
            (double)pos[i] * M_PI / 180.0,
            (double)vel[i] * M_PI / 30.0,
            kps[i], kds[i], tqs[i],
            &pr, &vr, &kr, &dr, &tr);

        if (pr == 0 && vr == 0 && kr == 0 && dr == 0 && tr == 0) continue;

        cmds[count].node_id       = ids[i];
        cmds[count].enable        = true;
        cmds[count].release_brake = false;
        cmds[count].clear_error   = false;
        cmds[count].position      = pr;
        cmds[count].velocity      = vr;
        cmds[count].kp            = kr;
        cmds[count].kd            = dr;
        cmds[count].torque        = tr;
        count++;
    }
    if (count > 0) {
        static int _multi_skip = 0;
        if (++_multi_skip % 50 == 0) {
        PROTO_SEND("[MIT_MULTI_SEND] M%d+M%d count=%d | "
                   "raw1: pr=%u vr=%u kr=%u dr=%u tr=%d | "
                   "raw2: pr=%u vr=%u kr=%u dr=%u tr=%d",
                   id1, id2, count,
                   count >= 1 ? cmds[0].position : 0, count >= 1 ? cmds[0].velocity : 0,
                   count >= 1 ? cmds[0].kp : 0, count >= 1 ? cmds[0].kd : 0,
                   count >= 1 ? cmds[0].torque : 0,
                   count >= 2 ? cmds[1].position : 0, count >= 2 ? cmds[1].velocity : 0,
                   count >= 2 ? cmds[1].kp : 0, count >= 2 ? cmds[1].kd : 0,
                   count >= 2 ? cmds[1].torque : 0);
        }
        motor_hal_mit_multi_ctrl(hal, cmds, count);
    }
}

int motor_hal_ctrl_raw(motor_hal_t *hal, uint8_t node_id,
                       motor_mode_t mode,
                       int16_t target1, uint16_t target2, int16_t feedforward)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    /* 传入 mode 更新 Byte0 mode 字段, 保持其他 bit 不变 */
    uint8_t b0 = m ? _consume_pdo_byte0(m) : 0;
    if (m) { b0 = (b0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(mode); m->pdo_byte0 = b0; }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, target1, target2, feedforward);
    return 0;
}

int motor_hal_stop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _consume_pdo_byte0(m) : 0;
    pthread_mutex_unlock(&hal->lock);

    if (!m || !enabled) return 0;

    pdo_ctrl_send_raw(hal->drv, node_id, b0, 0, 0, 0);
    return 0;
}

int motor_hal_set_brake(motor_hal_t *hal, uint8_t node_id, bool release)
{
    if (!hal || !hal->drv) return -ENODEV;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    bool enabled = m ? m->enabled : false;
    uint8_t b0 = m ? _consume_pdo_byte0(m) : 0;
    if (m) {
        if (release) b0 |=  PDO_BYTE0_BUS_ON;
        else         b0 &= ~PDO_BYTE0_BUS_ON;
        m->pdo_byte0 = b0;
    }
    pthread_mutex_unlock(&hal->lock);

    if (!m) return -ENOENT;
    if (!enabled) return -EAGAIN;

    /* bit6 母线电压 (当前电机不实现机械抱闸, 预留) */
    pdo_ctrl_send_raw(hal->drv, node_id, b0, 0, 0, 0);
    return 0;
}

int motor_hal_quick_stop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id,
                            OD_CONTROLWORD, 0x00, CW_QUICK_STOP, 2);
}

/* =====================================================
 * 公共 API: PDO Byte0 — 实时控制字节
 *
 *   SDO 不管 PDO: startup/enable/disable 不设 pdo_byte0
 *   PDO 不管 SDO: 以下函数只改 pdo_byte0, 不改 m->enabled/state
 *
 *   算法层使用:
 *     startup ,  pdo_enable + pdo_set_mode ,  控制循环(不碰Byte0)
 *     急停    ,  pdo_estop
 *     恢复    ,  pdo_recover
 *     清错    ,  先读 fb->error_code, 再根据错误类型决定是否 clearcf
 *     切模式  ,  pdo_set_mode
 *
 *   bit5 清错流程 (由算法层决策):
 *     1. 读 motor_hal_get_feedback ,  fb.error_code
 *     2. 判断: 温度过高? 等降温后清除; 过流? 失能后清除; 其他? 直接清除
 *     3. 调用 motor_hal_pdo_clear_fault ,  bit5 脉冲, 下一帧自动清0
 * ===================================================== */

int motor_hal_pdo_enable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 |= PDO_BYTE0_ENABLE | PDO_BYTE0_BUS_ON;
    m->enabled = true;
    _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_disable(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 &= ~PDO_BYTE0_ENABLE;
    m->enabled = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_bus_on(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 |= PDO_BYTE0_BUS_ON;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_bus_off(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 &= ~PDO_BYTE0_BUS_ON;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_clear_fault(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 |= PDO_BYTE0_CLR_ERR;
    m->clr_err_pending = true;  /* 下一帧控制函数自动清除 */
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode)
{
    if (!hal) return -EINVAL;
    if (mode > MOTOR_MODE_MIT) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 = (m->pdo_byte0 & ~PDO_BYTE0_MODE_MASK) | pdo_byte0_mode_part(mode);
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_estop(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 &= PDO_BYTE0_MODE_MASK;  /* enable=0, bus=0, 保留mode */
    m->enabled = false;
    m->clr_err_pending = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_recover(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 = (m->pdo_byte0 & PDO_BYTE0_MODE_MASK)
                 | PDO_BYTE0_ENABLE | PDO_BYTE0_BUS_ON;
    m->enabled = true;
    m->clr_err_pending = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_set_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t byte0)
{
    if (!hal) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    m->pdo_byte0 = byte0;
    m->clr_err_pending = false;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_get_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t *byte0)
{
    if (!hal || !byte0) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    *byte0 = m->pdo_byte0;
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

int motor_hal_pdo_consume_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t *byte0)
{
    if (!hal || !byte0) return -EINVAL;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return -ENOENT; }
    *byte0 = _consume_pdo_byte0(m);  /* 自动清除 clr_err 脉冲 */
    pthread_mutex_unlock(&hal->lock);
    return 0;
}

/* =====================================================
 * 公共 API: 模式 / 参数 / PID
 * ===================================================== */

int motor_hal_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode)
{
    if (!hal || !hal->drv) return -ENODEV;

    /* 巨蟹协议: PDO mode flag 和 SDO 0x6060 是两套编码, SDO 走 CiA 402 标准 */
    static const uint8_t sdo_mode_map[] = {
        [MOTOR_MODE_PROFILE_POS] = 0x01,  /* PP:      巨蟹协议 SDO 0x01 (PP)      */
        [MOTOR_MODE_PROFILE_VEL] = 0x03,  /* PV:      巨蟹协议 SDO 0x03 (PV)      */
        [MOTOR_MODE_CSP]         = 0x08,  /* CSP:     巨蟹协议 SDO 0x08 (CSP)     电机暂时不支持*/
        [MOTOR_MODE_CSV]         = 0x09,  /* CSV:     巨蟹协议 SDO 0x09 (CSV)     电机暂时不支持*/
        [MOTOR_MODE_CURRENT]     = 0x0A,  /* 电流环:  巨蟹协议 SDO 0x0A           */
        [MOTOR_MODE_MIT]         = 0x06,  /* MIT:     巨蟹快控 0x06               */
    };

    if (mode >= sizeof(sdo_mode_map)) return -EINVAL;
    return sdo_write_simple(hal->drv, node_id, OD_MODE_OF_OP, 0x00,
                            sdo_mode_map[mode], 1);
}

int motor_hal_set_accel_decel(motor_hal_t *hal, uint8_t node_id,
                              uint16_t accel, uint16_t decel)
{
    if (!hal || !hal->drv) return -ENODEV;
    int r1 = sdo_write_simple(hal->drv, node_id, OD_PROFILE_ACCEL, 0x00, accel, 4);
    int r2 = sdo_write_simple(hal->drv, node_id, OD_PROFILE_DECEL, 0x00, decel, 4);
    return (r1 == 0 && r2 == 0) ? 0 : -1;
}

int motor_hal_set_profile_velocity(motor_hal_t *hal, uint8_t node_id, uint16_t rpm_out)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_PROFILE_VEL, 0x00, rpm_out, 4);
}

int motor_hal_set_pid(motor_hal_t *hal, uint8_t node_id, const motor_pid_t *pid)
{
    if (!hal || !hal->drv || !pid) return -EINVAL;
    int ret = 0;
    ret |= sdo_write_simple(hal->drv, node_id, OD_CURRENT_P,  0x00, pid->current_p,  4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_CURRENT_I,  0x00, pid->current_i,  4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_VELOCITY_P, 0x00, pid->velocity_p, 4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_VELOCITY_I, 0x00, pid->velocity_i, 4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_POSITION_P, 0x00, pid->position_p, 4);
    ret |= sdo_write_simple(hal->drv, node_id, OD_POSITION_I, 0x00, pid->position_i, 4);
    return ret;
}

int motor_hal_save_flash(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_SAVE_FLASH, 0x00, 1, 4);
}

int motor_hal_set_zero(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_ZERO_POSITION, 0x00, 1, 4);
}

int motor_hal_calib_zero(motor_hal_t *hal, uint8_t node_id)
{
    int ret;

    /* 1. 失能: DS402 Shutdown (Controlword 0x6040 = 0x06) */
    ret = motor_hal_sdo_write(hal, node_id, OD_CONTROLWORD, 0x00, 0x06, 2);
    if (ret != 0) return ret;
    usleep(50000);  /* 50ms, 等电机停止输出 */

    /* 2. 下发零位 (0x2531) */
    ret = motor_hal_set_zero(hal, node_id);
    usleep(20000);  /* 20ms */
    return ret;
}

int motor_hal_set_limits(motor_hal_t *hal, uint8_t node_id, float pos_deg, float neg_deg)
{
    if (!hal || !hal->drv) return -ENODEV;

    int32_t pos = (int32_t)(pos_deg * ENCODER_LOAD_RES / 360.0f);
    int32_t neg = (int32_t)(neg_deg * ENCODER_LOAD_RES / 360.0f);

    /* subindex: 0x01=负限位, 0x02=正限位 */
    int r1 = sdo_write_simple(hal->drv, node_id, OD_POS_LIMIT, 0x02,
                              (uint32_t)pos, 4);
    int r2 = sdo_write_simple(hal->drv, node_id, OD_POS_LIMIT, 0x01,
                              (uint32_t)neg, 4);
    return (r1 == 0 && r2 == 0) ? 0 : -1;
}

int motor_hal_disable_watchdog(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
}

int motor_hal_set_pos_ctrl(motor_hal_t *hal, uint8_t node_id, bool start)
{
    if (!hal || !hal->drv) return -ENODEV;
    /* 0x4F=启动绝对位置运动, 0x0F=停止 — 协议要求 2 字节 SDO 写 */
    uint32_t cw = start ? 0x4FU : 0x0FU;
    return sdo_write_simple(hal->drv, node_id, OD_CONTROLWORD, 0x00, cw, 2);
}

int motor_hal_set_pos_target(motor_hal_t *hal, uint8_t node_id, int32_t target_counts)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_TARGET_POS, 0x00,
                            (uint32_t)target_counts, 4);
}

int motor_hal_set_speed_target(motor_hal_t *hal, uint8_t node_id, int32_t target_rpm)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, OD_TARGET_VELOCITY, 0x00,
                            (uint32_t)target_rpm, 4);
}

/* =====================================================
 * 公共 API: 状态查询 (SDO)
 * ===================================================== */

motor_state_t motor_hal_get_state(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return MOTOR_STATE_UNKNOWN;

    uint32_t sw_raw = 0;
    if (sdo_read_simple(hal->drv, node_id, OD_STATUSWORD, 0x00, &sw_raw) != 0)
        return MOTOR_STATE_UNKNOWN;

    uint16_t sw = (uint16_t)(sw_raw & SW_STATE_MASK);

    if (sw & SW_FAULT)  return MOTOR_STATE_FAULT;
    /* Operation Enabled 必须在 Quick Stop 之前判定:
       0x0027(OP) & 0x004F == 0x0007 会误匹配 Quick Stop 过滤器 */
    if (sw == SW_OP_ENABLED) return MOTOR_STATE_OP_ENABLED;
    if ((sw & 0x004F) == SW_QUICK_STOP_STATE) return MOTOR_STATE_QUICK_STOP;

    switch (sw) {
        case SW_NOT_READY:      return MOTOR_STATE_NOT_READY;
        case SW_ON_DISABLED:    return MOTOR_STATE_SWITCH_ON_DIS;
        case SW_READY_TO_SW_ON: return MOTOR_STATE_READY_TO_SW_ON;
        case SW_SWITCHED_ON:    return MOTOR_STATE_SWITCHED_ON;
        case SW_OP_ENABLED:     return MOTOR_STATE_OP_ENABLED;
        default: return MOTOR_STATE_UNKNOWN;
    }
}

uint16_t motor_hal_get_statusword(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_STATUSWORD, 0x00, &val);
    return (uint16_t)val;
}

int32_t motor_hal_get_position(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_POSITION_ACTUAL, 0x00, &val);
    /* 0x6064 是 int16 (±32767, ±180°), SDO 43 响应 4 字节但驱动板不扩展符号位
     * 必须手动取低 16 位做 int16 符号扩展, 否则负数变成大的正数 (差 65536) */
    return (int32_t)(int16_t)(val & 0xFFFF);
}

int32_t motor_hal_get_velocity(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_VELOCITY_ACTUAL, 0x00, &val);
    return (int32_t)val;
}

int32_t motor_hal_get_current(motor_hal_t *hal, uint8_t node_id)
{
    uint32_t val = 0;
    if (hal && hal->drv)
        sdo_read_simple(hal->drv, node_id, OD_CURRENT_ACTUAL, 0x00, &val);
    /* 巨蟹 0x6078 返回 2 字节 int16, 需符号扩展 */
    return (int32_t)(int16_t)(val & 0xFFFF);
}

int motor_hal_read_pid(motor_hal_t *hal, uint8_t node_id, motor_pid_t *pid)
{
    if (!hal || !hal->drv || !pid) return -EINVAL;

    uint32_t v;
    sdo_read_simple(hal->drv, node_id, OD_CURRENT_P,  0x00, &v); pid->current_p  = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_CURRENT_I,  0x00, &v); pid->current_i  = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_VELOCITY_P, 0x00, &v); pid->velocity_p = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_VELOCITY_I, 0x00, &v); pid->velocity_i = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_POSITION_P, 0x00, &v); pid->position_p = (uint16_t)v;
    sdo_read_simple(hal->drv, node_id, OD_POSITION_I, 0x00, &v); pid->position_i = (uint16_t)v;
    return 0;
}

int motor_hal_sdo_read_u32(motor_hal_t *hal, uint8_t node_id,
                           uint16_t index, uint8_t subidx, uint32_t *value)
{
    if (!hal || !hal->drv || !value) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, index, subidx, value);
}

int motor_hal_sdo_write(motor_hal_t *hal, uint8_t node_id,
                        uint16_t index, uint8_t subidx,
                        uint32_t value, uint8_t size)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, index, subidx, value, size);
}

/* =====================================================
 * 公共 API: 反馈缓存
 * ===================================================== */

int motor_hal_get_feedback(motor_hal_t *hal, uint8_t node_id, motor_feedback_t *fb)
{
    if (!hal || !fb) return -EINVAL;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    pthread_mutex_lock(&m->fb_lock);
    memcpy(fb, &m->cached_fb, sizeof(motor_feedback_t));
    pthread_mutex_unlock(&m->fb_lock);

    return 0;
}

/* =====================================================
 * 公共 API: 回调
 * ===================================================== */

void motor_hal_set_feedback_cb(motor_hal_t *hal, uint8_t node_id,
                               motor_feedback_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->fb_cb  = cb;
    m->fb_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_set_error_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_error_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->err_cb  = cb;
    m->err_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_set_state_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_state_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->state_cb  = cb;
    m->state_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_set_sensor_cb(motor_hal_t *hal, uint8_t node_id,
                             motor_sensor_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->sensor_cb  = cb;
    m->sensor_ctx = ctx;
    pthread_mutex_unlock(&hal->lock);
}

/* =====================================================
 * 公共 API: 专用 SDO 控制接口 — 对应巨蟹协议 4.3 章
 * ===================================================== */

int motor_hal_nmt_send(motor_hal_t *hal, uint8_t node_id, uint8_t cmd)
{
    if (!hal || !hal->drv) return -ENODEV;
    canfd_frame_t f;
    canopen_nmt_build(cmd, node_id, &f);
    return can_driver_send(hal->drv, &f) >= 0 ? 0 : -errno;
}

int motor_hal_get_fault_code(motor_hal_t *hal, uint8_t node_id, uint16_t *code)
{
    if (!hal || !hal->drv || !code) return -EINVAL;
    uint32_t val = 0;
    int ret = sdo_read_simple(hal->drv, node_id, 0x603F, 0x00, &val);
    *code = (uint16_t)(val & 0xFFFF);
    return ret;
}

int motor_hal_get_mos_temp(motor_hal_t *hal, uint8_t node_id, int32_t *temp)
{
    if (!hal || !hal->drv || !temp) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, 0x2662, 0x00, (uint32_t *)temp);
}

int motor_hal_get_motor_temp(motor_hal_t *hal, uint8_t node_id, int32_t *temp)
{
    if (!hal || !hal->drv || !temp) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, 0x2663, 0x00, (uint32_t *)temp);
}

int motor_hal_get_max_current(motor_hal_t *hal, uint8_t node_id, uint32_t *ma)
{
    if (!hal || !hal->drv || !ma) return -EINVAL;
    return sdo_read_simple(hal->drv, node_id, 0x2538, 0x00, ma);
}

int motor_hal_set_max_current(motor_hal_t *hal, uint8_t node_id, uint32_t ma)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, 0x2538, 0x00, ma, 4);
}

int motor_hal_set_heartbeat(motor_hal_t *hal, uint8_t node_id, uint32_t ms)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, 0x1017, 0x00, ms, 2);
}

int motor_hal_set_node_id(motor_hal_t *hal, uint8_t node_id, uint8_t new_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (new_id < 1 || new_id > 127) return -EINVAL;
    return sdo_write_simple(hal->drv, node_id, 0x2530, 0x00, new_id, 4);
}

int motor_hal_set_canfd_baud(motor_hal_t *hal, uint8_t node_id, uint8_t baud)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (baud < 1 || baud > 4) return -EINVAL;
    return sdo_write_simple(hal->drv, node_id, 0x2540, 0x00, baud, 4);
}

/* =====================================================
 * 内部: 帧分发 (接收线程 + poll 共用)
 * ===================================================== */

/* =====================================================
 * 内部: 传感器数据解析 (8字节, 小端, bit-packed)
 * ===================================================== */

static void _parse_sensor_frame(const canfd_frame_t *f, motor_sensor_t *s)
{
    memset(s, 0, sizeof(*s));
    if (!f || f->dlc < 8) return;

    /* 拼成 uint64 (little-endian) */
    uint64_t p = 0;
    for (int i = 0; i < 8; i++) {
        p |= ((uint64_t)f->data[i]) << (8U * i);
    }

    s->hall_adc0   = (uint16_t)((p >> 0)  & 0x0FFFU);
    s->hall_adc1   = (uint16_t)((p >> 12) & 0x0FFFU);
    s->hall_adc2   = (uint16_t)((p >> 24) & 0x0FFFU);
    s->force_raw   = (uint16_t)((p >> 36) & 0x3FFFU);
    s->knee_hall    = (uint16_t)((p >> 50) & 0x0FFFU);
    s->hw_sw_pc9   = (uint8_t)((p >> 62) & 0x01U);
    s->data_valid  = (uint8_t)((p >> 63) & 0x01U);
}

static void _dispatch_frame(motor_hal_t *hal, const canfd_frame_t *f)
{
    uint32_t func = canopen_func_code(f->id);

    _dump_can_frame("recv", f);

    /* 0x6A0 运行反馈帧1: Iq(S16)+母线电流(S16)+温度(S16)+错误码(U16) */
    if ((f->id & 0x7F0U) == COB_RUN_FB1_BASE) {
        if (f->dlc >= 8) {
            uint8_t node = f->id & 0x0FU;
            motor_node_t *m = _find_motor(hal, node);
            if (m) {
                pthread_mutex_lock(&m->sensor_lock);
                m->cached_sensor.iq_current  = (int16_t)((uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8));
                m->cached_sensor.bus_current = (int32_t)((int16_t)((uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8)));
                m->cached_sensor.motor_temp_x10 = (int16_t)((uint16_t)f->data[4] | ((uint16_t)f->data[5] << 8));
                m->cached_sensor.error_code    = (uint16_t)((uint16_t)f->data[6] | ((uint16_t)f->data[7] << 8));
                m->cached_sensor.run_fb_timestamp_us = motor_utils_now_us();
                PROTO_RECV(node, "[6A0_RECV] M%d Iq=%.1fA bus=%.2fA temp=%.1f\u00b0C err=0x%04X",
                           node,
                           m->cached_sensor.iq_current / 1000.0,
                           m->cached_sensor.bus_current / 1000.0,
                           m->cached_sensor.motor_temp_x10 / 10.0,
                           m->cached_sensor.error_code);
                pthread_mutex_unlock(&m->sensor_lock);
            }
        }
        return;
    }

    /* 0x690 运行反馈帧0: 位置 S32 LE + 速度 S32 LE (V1.1 §4.2) */
    if ((f->id & 0x7F0U) == 0x690) {
        if (f->dlc >= 8) {
            uint8_t node = f->id & 0x0FU;
            motor_node_t *m = _find_motor(hal, node);
            if (m) {
                /* 小端 S32: 实际位置 (cnt, -32768~32767 → -180°~180°) */
                int32_t raw_pos = (int32_t)((uint32_t)f->data[0]
                                  | ((uint32_t)f->data[1] << 8)
                                  | ((uint32_t)f->data[2] << 16)
                                  | ((uint32_t)f->data[3] << 24));
                /* 小端 S32: 实际速度 (RPM) */
                int32_t raw_vel = (int32_t)((uint32_t)f->data[4]
                                  | ((uint32_t)f->data[5] << 8)
                                  | ((uint32_t)f->data[6] << 16)
                                  | ((uint32_t)f->data[7] << 24));
                pthread_mutex_lock(&m->sensor_lock);
                m->cached_sensor.motor_pos_raw = raw_pos;
                m->cached_sensor.motor_vel_raw = raw_vel;
                m->cached_sensor.run_fb_timestamp_us = motor_utils_now_us();
                PROTO_RECV(node, "[690_RECV] M%d pos=%d cnt (%.1f\u00b0) vel=%d RPM",
                           node, raw_pos, raw_pos * 360.0 / 65536.0, raw_vel);
                pthread_mutex_unlock(&m->sensor_lock);
            }
        }
        return;
    }

    /* 0x6C0 32 Byte CAN FD 聚合透传帧 (V1.2 mode=3) */
    if ((f->id & 0x7F0U) == COB_SENSOR_AGGR_BASE && f->dlc >= 32) {
        uint8_t node = f->id & 0x0FU;
        motor_node_t *m = _find_motor(hal, node);
        if (m) {
            pthread_mutex_lock(&m->sensor_lock);

            /* 将 32Byte 聚合帧拆为 4段 8Byte, 复用现有解析逻辑 */
            /* 段1: Byte 0-7  → 0x680 Hall+力矩 */
            canfd_frame_t f680 = *f;
            f680.dlc = 8;
            _parse_sensor_frame(&f680, &m->cached_sensor);
            m->cached_sensor.timestamp_us = motor_utils_now_us();

            /* 段2: Byte 8-15 → 0x690 位置S32+速度S32 */
            m->cached_sensor.motor_pos_raw = (int32_t)((uint32_t)f->data[8]
                                              | ((uint32_t)f->data[9] << 8)
                                              | ((uint32_t)f->data[10] << 16)
                                              | ((uint32_t)f->data[11] << 24));
            m->cached_sensor.motor_vel_raw = (int32_t)((uint32_t)f->data[12]
                                              | ((uint32_t)f->data[13] << 8)
                                              | ((uint32_t)f->data[14] << 16)
                                              | ((uint32_t)f->data[15] << 24));

            /* 段3: Byte 16-23 → 0x6A0 (V1.1 §4.3):
             * [16-17]=iq_current S16, [18-19]=bus_current S16,
             * [20-21]=motor_temp S16, [22-23]=error_code U16 */
            m->cached_sensor.iq_current  = (int16_t)((uint16_t)f->data[16] | ((uint16_t)f->data[17] << 8));
            m->cached_sensor.bus_current = (int32_t)((int16_t)((uint16_t)f->data[18] | ((uint16_t)f->data[19] << 8)));
            m->cached_sensor.motor_temp_x10 = (int16_t)((uint16_t)f->data[20] | ((uint16_t)f->data[21] << 8));
            m->cached_sensor.error_code    = (uint16_t)((uint16_t)f->data[22] | ((uint16_t)f->data[23] << 8));
            m->cached_sensor.run_fb_timestamp_us = motor_utils_now_us();

            /* 段4: Byte 24-31 → 0x6B0 SPI力矩 (复用 CMD_SPI 解析) */
            {
                spi_force_frame_t st;
                canfd_frame_t f6b0 = *f;
                f6b0.dlc = 8;
                memcpy(f6b0.data, &f->data[24], 8);
                canopen_parse_spi_force(&f6b0, &st);
                m->cached_sensor.spi_force_raw_s24 = st.force_raw_s24;
                m->cached_sensor.spi_valid         = st.valid;
                m->cached_sensor.spi_error         = st.error;
                m->cached_sensor.spi_timestamp_us  = motor_utils_now_us();
            }

            pthread_mutex_unlock(&m->sensor_lock);

            PROTO_RECV(node, "[6C0_RECV] M%d hall=(%u,%u,%u) force=%u knee=%u key=%u valid=%u | "
                       "pos=%d cnt vel=%d RPM | Iq=%.1fA bus=%.2fA temp=%.1f\u00b0C err=0x%04X | "
                       "spi_raw=%d spi_v=%u spi_e=%u",
                       node,
                       m->cached_sensor.hall_adc0, m->cached_sensor.hall_adc1, m->cached_sensor.hall_adc2,
                       m->cached_sensor.force_raw, m->cached_sensor.knee_hall, m->cached_sensor.hw_sw_pc9,
                       m->cached_sensor.data_valid,
                       m->cached_sensor.motor_pos_raw, m->cached_sensor.motor_vel_raw,
                       m->cached_sensor.iq_current / 1000.0, m->cached_sensor.bus_current / 1000.0,
                       m->cached_sensor.motor_temp_x10 / 10.0, m->cached_sensor.error_code,
                       m->cached_sensor.spi_force_raw_s24, m->cached_sensor.spi_valid, m->cached_sensor.spi_error);

            /* 触发传感器回调 */
            if (m->sensor_cb) {
                m->sensor_cb(node, &m->cached_sensor, m->sensor_ctx);
            }
        }
        return;
    }

    /* 0x6B0 SPI 力矩帧: 与 0x680 系列在 CANopen func code (&0x780) 下同码,
     * 必须在 switch 前用 0x7F0 掩码区分 (node_id < 16 成立, 当前双电机满足),
     * 否则会落入 0x680 分支被丢弃. */
    if ((f->id & 0x7F0U) == COB_SPI_TORQUE_BASE) {
        uint8_t node = canopen_extract_node(f->id, COB_SPI_TORQUE_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (m) {
            spi_force_frame_t st;
            canopen_parse_spi_force(f, &st);
            pthread_mutex_lock(&m->sensor_lock);
            m->cached_sensor.spi_force_raw_s24 = st.force_raw_s24;
            m->cached_sensor.spi_valid         = st.valid;
            m->cached_sensor.spi_error         = st.error;
            m->cached_sensor.spi_timestamp_us  = motor_utils_now_us();
            pthread_mutex_unlock(&m->sensor_lock);
        }
        return;
    }

    switch (func) {
    case 0x580: {  /* SDO 响应 ,  入队, 等待 sdo_client 消费 */
        sdo_push_response(f);
        break;
    }

    case 0x700: {  /* Bootup / Heartbeat */
        if (canopen_is_bootup(f->id, f->data[0])) {
            uint8_t node = canopen_extract_node(f->id, COB_BOOTUP_BASE);
            motor_node_t *m = _find_motor(hal, node);
            if (m) {
                m->bootup_received = true;
                fprintf(stderr, "  ,  Bootup node=%d\n", node);
                /* 自动启动: 只设标志, 由主线程 motor_hal_process_pending_startups() 消费 */
                /* 不能在 recv 线程里调 motor_startup_full — SDO 阻塞会导致自己收不到响应 */
                if (m->state == MOTOR_STATE_NOT_READY) {
                    m->pending_startup = true;
                }
            } else {
                fprintf(stderr, "  ,  WARN: Bootup node=%d not registered\n", node);
            }
        } else {
            /* 心跳帧: 只在 NMT 状态变化时打印 */
            uint8_t node = canopen_extract_node(f->id, COB_BOOTUP_BASE);
            motor_node_t *m = _find_motor(hal, node);
            uint8_t cur_st = f->data[0];
            if (m && cur_st != m->last_nmt_state) {
                m->last_nmt_state = cur_st;
                const char *st = motor_utils_nmt_state_str(cur_st);
                fprintf(stderr, "  ,  Heartbeat node=%d state=%s (0x%02X)\n",
                        node, st ? st : "?", cur_st);
            }
        }
        break;
    }

    case 0x300: {  /* 反馈帧 (巨蟹私有) */
        uint8_t node = canopen_extract_node(f->id, COB_FEEDBACK_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        motor_feedback_t fb;
        pdo_feedback_parse(f, &fb);

        /* 更新缓存 */
        pthread_mutex_lock(&m->fb_lock);
        memcpy(&m->cached_fb, &fb, sizeof(fb));
        m->last_fb_us = fb.timestamp_us;
        pthread_mutex_unlock(&m->fb_lock);

        /* 触发反馈回调 */
        if (m->fb_cb) {
            m->fb_cb(node, &fb, m->fb_ctx);
        }

        /* 检测错误 */
        if (fb.status_byte & 0x20) {
            if (m->err_cb) {
                m->err_cb(node, fb.error_code, m->err_ctx);
            }
        }
        break;
    }

    case 0x180: {  /* 标准 TPDO1 (同步周期上报: 0x180+node) */
        uint8_t node = (uint8_t)(f->id & 0x7F);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        /* 优先调用户自定义回调 (自定义映射时) */
        if (m->tpdo_raw_cb) {
            m->tpdo_raw_cb(node, f, m->tpdo_raw_ctx);
            break;
        }

        /* 回退: 默认硬编码解析 Statusword+Position+Velocity+Current */
        if (f->dlc < 8) break;
        uint16_t sw = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        int32_t  pos = (int32_t)((uint32_t)f->data[2]
                      | ((uint32_t)f->data[3] << 8)
                      | ((uint32_t)f->data[4] << 16)
                      | ((uint32_t)f->data[5] << 24));
        /* 如果 TPDO 包含了 Velocity + Current (12 bytes), 也解析 */
        int32_t  vel = 0;
        int16_t  cur = 0;
        if (f->dlc >= 12) {
            vel = (int32_t)((uint32_t)f->data[6]
                  | ((uint32_t)f->data[7] << 8)
                  | ((uint32_t)f->data[8] << 16)
                  | ((uint32_t)f->data[9] << 24));
            cur = (int16_t)((uint16_t)f->data[10] | ((uint16_t)f->data[11] << 8));
        }

        /* 更新缓存 (补充 TPDO 数据到缓存) */
        pthread_mutex_lock(&m->fb_lock);
        m->cached_fb.position   = (int16_t)pos;  /* 截断到 16bit, 兼容现有类型 */
        m->cached_fb.velocity   = (int16_t)vel;
        m->cached_fb.current_iq = cur;
        m->cached_fb.timestamp_us = motor_utils_now_us();
        /* 从 statusword 推导状态 */
        m->cached_fb.status_byte = (sw & 0x000F) |  /* 低4位=状态 */
                                    ((sw & 0x1000) ? 0 : 0x80);  /* bit12=0, enabled */
        pthread_mutex_unlock(&m->fb_lock);

        /* 触发反馈回调 */
        if (m->fb_cb) {
            m->fb_cb(node, &m->cached_fb, m->fb_ctx);
        }
        break;
    }

    case 0x080: {  /* EMCY 紧急报文 */
        uint8_t node = canopen_extract_node(f->id, COB_EMCY_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        uint16_t err = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        if (m->err_cb) {
            m->err_cb(node, err, m->err_ctx);
        }
        break;
    }

    case 0x680: {  /* 传感器透传 (0x680 + node_id) */
        uint8_t node = canopen_extract_node(f->id, COB_SENSOR_BASE);
        motor_node_t *m = _find_motor(hal, node);
        if (!m) break;

        motor_sensor_t s;
        _parse_sensor_frame(f, &s);
        s.timestamp_us = motor_utils_now_us();

        pthread_mutex_lock(&m->sensor_lock);
        /* 只写入 _parse_sensor_frame 实际解析的字段, 不动 SPI 力矩字段 */
        m->cached_sensor.hall_adc0   = s.hall_adc0;
        m->cached_sensor.hall_adc1   = s.hall_adc1;
        m->cached_sensor.hall_adc2   = s.hall_adc2;
        m->cached_sensor.force_raw   = s.force_raw;
        m->cached_sensor.knee_hall    = s.knee_hall;
        m->cached_sensor.hw_sw_pc9   = s.hw_sw_pc9;
        m->cached_sensor.data_valid  = s.data_valid;
        m->cached_sensor.timestamp_us = s.timestamp_us;
        m->last_sensor_us = s.timestamp_us;
        pthread_mutex_unlock(&m->sensor_lock);

        if (m->sensor_cb) {
            m->sensor_cb(node, &s, m->sensor_ctx);
        }
        break;
    }

    default:
        break;
    }
}

/* =====================================================
 * 接收线程 (唯一 recv 入口)
 * ===================================================== */

static void* _recv_thread_fn(void *arg)
{
    motor_hal_t *hal = (motor_hal_t*)arg;

    if (hal->recv_rt_enable) {
        struct sched_param sp;
        sp.sched_priority = hal->recv_rt_priority;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }

    /* 绑核: 与 RT 线程同核, 降低跨核迁移导致的反馈打点延迟 */
    if (hal->recv_cpu >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(hal->recv_cpu, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    }

    while (hal->recv_running) {
        canfd_frame_t f;
        int ret = can_driver_recv(hal->drv, &f, 100);
        if (ret <= 0) continue;
        _dispatch_frame(hal, &f);
    }

    return NULL;
}

/* =====================================================
 * 公共 API: 接收线程控制
 * ===================================================== */

int motor_hal_recv_start(motor_hal_t *hal)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (hal->recv_running) return -EBUSY;

    hal->recv_running = true;

    int ret = pthread_create(&hal->recv_thread, NULL, _recv_thread_fn, hal);
    if (ret != 0) {
        hal->recv_running = false;
        return -ret;
    }

    return 0;
}

void motor_hal_recv_set_rt(motor_hal_t *hal, bool enable, int priority)
{
    if (!hal) return;
    hal->recv_rt_enable   = enable;
    hal->recv_rt_priority = priority;
}

void motor_hal_recv_set_affinity(motor_hal_t *hal, int cpu)
{
    if (!hal) return;
    hal->recv_cpu = cpu;
}

int motor_hal_recv_stop(motor_hal_t *hal)
{
    if (!hal || !hal->recv_running) return -EINVAL;

    hal->recv_running = false;
    pthread_join(hal->recv_thread, NULL);

    return 0;
}

bool motor_hal_recv_is_running(motor_hal_t *hal)
{
    return hal && hal->recv_running;
}

/* =====================================================
 * 公共 API: 轮询 (向前兼容, 接收线程启动后无需调用)
 * ===================================================== */

void motor_hal_poll(motor_hal_t *hal, int timeout_ms)
{
    if (!hal || !hal->drv || hal->recv_running) return;

    canfd_frame_t f;
    int ret = can_driver_recv(hal->drv, &f, timeout_ms);
    if (ret <= 0) return;
    _dispatch_frame(hal, &f);
}

/* =====================================================
 * 公共 API: 处理待启动电机 (主线程调用, 不能从 recv 线程调)
 *
 * 当 recv 线程收到 bootup 帧且 auto_enable=true 时,
 * 只设 pending_startup 标志, 由主线程定期调用此函数执行
 * motor_startup_full (SDO 操作).
 * 这样 SDO 响应由 recv 线程正常接收, 避免死锁.
 * ===================================================== */

int motor_hal_process_pending_startups(motor_hal_t *hal)
{
    if (!hal || !hal->drv) return 0;

    int started = 0;

    for (int i = 0; i < hal->motor_count; i++) {
        motor_node_t *m = &hal->motors[i];

        /* 快速检查: 无需持锁 */
        if (!m->pending_startup) continue;

        pthread_mutex_lock(&hal->lock);
        /* 双重检查 — 可能被其他 startup 命令抢走 */
        if (!m->pending_startup || m->state != MOTOR_STATE_NOT_READY) {
            pthread_mutex_unlock(&hal->lock);
            continue;
        }
        m->pending_startup = false;
        pthread_mutex_unlock(&hal->lock);

        /* 放锁后调 SDO — recv 线程可以正常收帧 */
        fprintf(stderr, "  ,  processing auto-startup node=%d...\n", m->node_id);
        int ret = motor_startup_full(hal->drv, &m->config, &m->bootup_received);
        if (ret == 0) {
            pthread_mutex_lock(&hal->lock);
            if (m->config.auto_enable) {
                m->enabled = true;
                _set_state(hal, m, MOTOR_STATE_OP_ENABLED);
            } else {
                m->enabled = false;
                _set_state(hal, m, MOTOR_STATE_SWITCH_ON_DIS);
            }
            pthread_mutex_unlock(&hal->lock);
            fprintf(stderr, "  ,  node=%d %s\n", m->node_id,
                    m->config.auto_enable ? "OPERATION_ENABLED (auto)" : "SWITCH_ON_DISABLED (ready)");
        } else {
            fprintf(stderr, "  ,  node=%d auto-startup failed (ret=%d)\n", m->node_id, ret);
        }
        started++;
    }

    return started;
}

/* =====================================================
 * 公共 API: 全局控制
 * ===================================================== */

void motor_hal_nmt_broadcast(motor_hal_t *hal, uint8_t cmd)
{
    if (!hal || !hal->drv) return;
    nmt_broadcast(hal->drv, cmd);
}

void motor_hal_sync(motor_hal_t *hal)
{
    if (!hal || !hal->drv) return;
    pdo_sync_send(hal->drv);
}

/* =====================================================
 * SYNC 定时器线程
 * ===================================================== */

static void* _sync_thread_fn(void *arg)
{
    motor_hal_t *hal = (motor_hal_t*)arg;
    uint32_t period_us = hal->sync_period_us;

    if (hal->sync_rt_enable) {
        struct sched_param sp;
        sp.sched_priority = hal->sync_rt_priority;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }

    if (hal->sync_cpu >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(hal->sync_cpu, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    }

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (hal->sync_running) {
        pdo_sync_send(hal->drv);

        /* 绝对时间基准, 无累积漂移 */
        next.tv_nsec += (long)(period_us * 1000UL);
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}

int motor_hal_sync_start(motor_hal_t *hal, uint32_t period_us)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (hal->sync_running) return -EBUSY;
    if (period_us < 500 || period_us > 1000000) return -EINVAL;

    hal->sync_period_us = period_us;
    hal->sync_running = true;

    int ret = pthread_create(&hal->sync_thread, NULL, _sync_thread_fn, hal);
    if (ret != 0) {
        hal->sync_running = false;
        return -ret;
    }

    fprintf(stderr, "[SYNC] started: period=%u us (%.1f Hz)\n",
            period_us, 1000000.0f / (float)period_us);
    return 0;
}

int motor_hal_sync_stop(motor_hal_t *hal)
{
    if (!hal || !hal->sync_running) return -EINVAL;

    hal->sync_running = false;
    pthread_join(hal->sync_thread, NULL);

    fprintf(stderr, "[SYNC] stopped\n");
    return 0;
}

void motor_hal_sync_set_rt(motor_hal_t *hal, bool enable, int priority)
{
    if (!hal) return;
    hal->sync_rt_enable   = enable;
    hal->sync_rt_priority = priority;
}

void motor_hal_sync_set_affinity(motor_hal_t *hal, int cpu)
{
    if (!hal) return;
    hal->sync_cpu = cpu;
}

bool motor_hal_sync_is_running(motor_hal_t *hal)
{
    return hal && hal->sync_running;
}

/* =====================================================
 * TPDO/RPDO 配置 — 通用映射 (按巨蟹文档时序)
 *
 * 巨蟹文档 PDO 映射流程 (以 TPDO1 为例):
 *   1. 关闭不需要的 PDO 通道 (1801/1802/1803 sub01 bit31=1, best-effort)
 *   2. 设置传输类型 (1800 sub02)
 *   3. 清空映射 (1A00 sub00=0)
 *   4. 写入映射条目 (1A00 sub01/sub02/...)
 *   5. 保存映射数量 (1A00 sub00=N)
 *   6. 启用 PDO (1800 sub01=COB-ID)
 *
 * RPDO 同理, 使用 140x/160x 索引。
 * ===================================================== */

static int _pdo_map_core(motor_hal_t *hal, uint8_t node_id,
                         const pdo_map_entry_cfg_t *entries, uint8_t count,
                         uint16_t comm_idx, uint16_t map_idx,
                         uint32_t cob_id, uint8_t trans_type)
{
    int ret;

    /* 1. 设置传输类型 (trans_type, 1字节) */
    ret = sdo_write_simple(hal->drv, node_id, comm_idx, 0x02,
                           trans_type, 1);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d set trans_type=%d failed\n",
                node_id, trans_type);
        return ret;
    }

    /* 2. 清空映射 (map sub00=0, 1字节) */
    ret = sdo_write_simple(hal->drv, node_id, map_idx, 0x00, 0, 1);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d clear map failed\n", node_id);
        return ret;
    }

    /* 3. 写入映射条目 */
    for (uint8_t i = 0; i < count; i++) {
        /* 映射条目编码: Index[31:16] SubIdx[15:8] BitLen[7:0] */
        uint32_t entry = ((uint32_t)entries[i].index << 16)
                       | ((uint32_t)entries[i].subidx << 8)
                       | (uint32_t)entries[i].bitlen;
        ret = sdo_write_simple(hal->drv, node_id, map_idx,
                               (uint8_t)(i + 1), entry, 4);
        if (ret != 0) {
            fprintf(stderr, "[PDO] node=%d map[%d] 0x%04X.%02X@%db failed\n",
                    node_id, i, entries[i].index,
                    entries[i].subidx, entries[i].bitlen);
            return ret;
        }
    }

    /* 4. 保存映射数量 (1字节) */
    ret = sdo_write_simple(hal->drv, node_id, map_idx, 0x00, count, 1);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d set map count=%d failed\n",
                node_id, count);
        return ret;
    }

    /* 5. 启用 PDO + 设置 COB-ID */
    ret = sdo_write_simple(hal->drv, node_id, comm_idx, 0x01, cob_id, 4);
    if (ret != 0) {
        fprintf(stderr, "[PDO] node=%d set COB=0x%03X failed\n",
                node_id, cob_id);
        return ret;
    }

    return 0;
}

/* ---------- 关闭其他 PDO 通道 (best-effort) ---------- */

static void _pdo_disable_others(motor_hal_t *hal, uint8_t node_id,
                                pdo_type_t type, uint8_t keep_idx)
{
    /* 关闭 TPDO2/3/4 或 RPDO2/3/4 (跳过 keep_idx) */
    const uint16_t comm_base = (type == PDO_TYPE_RPDO)
                               ? OD_RPDO1_COMM : OD_TPDO1_COMM;

    for (uint8_t i = 1; i <= 3; i++) {  /* PDO2/PDO3/PDO4 */
        if (i == keep_idx) continue;
        /* COB-ID bit31=1 ,  停用, 其他位保留原始 COB-ID (任意非零值) */
        uint16_t idx = comm_base + (uint16_t)i;
        sdo_write_simple(hal->drv, node_id, idx, 0x01, 0x80000000UL | (uint32_t)(0x80 + (i + 1)), 4);
    }
}

/* ---------- 公共 API ---------- */

int motor_hal_pdo_map(motor_hal_t *hal, uint8_t node_id,
                      const pdo_map_entry_cfg_t *entries, uint8_t count,
                      uint8_t pdo_idx, pdo_type_t type,
                      uint32_t cob_id, uint8_t trans_type)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (!entries || count == 0 || count > 8) return -EINVAL;
    if (pdo_idx > 1) return -EINVAL;

    /* 选择通信参数和映射表索引 */
    uint16_t comm_idx, map_idx;
    if (type == PDO_TYPE_RPDO) {
        comm_idx = (pdo_idx == 0) ? OD_RPDO1_COMM : OD_RPDO2_COMM;
        map_idx  = (pdo_idx == 0) ? OD_RPDO1_MAP  : OD_RPDO2_MAP;
    } else {
        comm_idx = (pdo_idx == 0) ? OD_TPDO1_COMM : OD_TPDO2_COMM;
        map_idx  = (pdo_idx == 0) ? OD_TPDO1_MAP  : OD_TPDO2_MAP;
    }

    const char *dir = (type == PDO_TYPE_RPDO) ? "RPDO" : "TPDO";

    /* 0. 关闭其他 PDO 通道 (best-effort, 失败不阻塞) */
    _pdo_disable_others(hal, node_id, type, pdo_idx);

    /* 1-5. 核心映射时序 */
    int ret = _pdo_map_core(hal, node_id, entries, count,
                            comm_idx, map_idx, cob_id, trans_type);
    if (ret == 0) {
        fprintf(stderr, "[%s] node=%d COB=0x%03X, %d entries, ttype=%d\n",
                dir, node_id, cob_id, count, trans_type);
    }
    return ret;
}

int motor_hal_tpdo_config(motor_hal_t *hal, uint8_t node_id, uint8_t sync_count)
{
    if (!hal || !hal->drv) return -ENODEV;
    if (sync_count == 0 || sync_count > 240) return -EINVAL;

    pdo_map_entry_cfg_t entries[] = {
        {OD_STATUSWORD,      0x00, 16},  /* Statusword */
        {OD_POSITION_ACTUAL, 0x00, 32},  /* Position */
        {OD_VELOCITY_ACTUAL, 0x00, 32},  /* Velocity */
        {OD_CURRENT_ACTUAL,  0x00, 16},  /* Current */
    };

    return motor_hal_pdo_map(hal, node_id, entries, 4, 0,
                             PDO_TYPE_TPDO,
                             PDO_TPDO1_COB(node_id), sync_count);
}

/* =====================================================
 * 标准 RPDO 发送 — 用户自定义映射后发送控制帧
 * ===================================================== */

int motor_hal_rpdo_send(motor_hal_t *hal, uint8_t node_id,
                        const uint8_t *data, uint8_t dlc)
{
    if (!hal || !hal->drv || !data) return -ENODEV;
    if (dlc == 0 || dlc > 8) return -EINVAL;

    canfd_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id     = (uint32_t)(COB_RPDO1_BASE + node_id);  /* 0x200 + node */
    f.dlc    = dlc;
    f.is_fd  = false;   /* 标准 CAN 帧 */
    f.use_brs = false;
    memcpy(f.data, data, dlc);

    return can_driver_send(hal->drv, &f) >= 0 ? 0 : -errno;
}

/* =====================================================
 * 标准 TPDO 原始帧回调
 * ===================================================== */

void motor_hal_set_tpdo_cb(motor_hal_t *hal, uint8_t node_id,
                           motor_tpdo_raw_cb_t cb, void *ctx)
{
    if (!hal) return;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) { pthread_mutex_unlock(&hal->lock); return; }
    m->tpdo_raw_cb   = cb;
    m->tpdo_raw_ctx  = ctx;
    pthread_mutex_unlock(&hal->lock);
}

void motor_hal_multi_ctrl(motor_hal_t *hal, const multi_axis_cmd_t *cmds, uint8_t count)
{
    if (!hal || !hal->drv) return;
    pdo_multi_send(hal->drv, cmds, count);
}

int motor_hal_single_ctrl(motor_hal_t *hal, const multi_axis_cmd_t *cmd)
{
    if (!hal || !hal->drv || !cmd) return -ENODEV;
    pdo_ctrl_send(hal->drv, cmd->node_id, cmd->mode,
                  cmd->enable, cmd->release_brake, cmd->clear_error,
                  cmd->target1, cmd->target2, cmd->feedforward);
    return 0;
}

void motor_hal_mit_multi_ctrl(motor_hal_t *hal, const multi_mit_cmd_t *cmds, uint8_t count)
{
    if (!hal || !hal->drv) return;
    pdo_mit_multi_send(hal->drv, cmds, count);
}

/* =====================================================
 * 公共 API: 传感器透传控制
 * ===================================================== */

/* 完整透传配置: period_div + bus_format + mode + force_module
 * 配置字 (OD 0x5503:04):
 *   period_div[15:0] | bus_format[17:16] | mode[19:18] | force_module[21:20] */
int motor_hal_sensor_config_ex(motor_hal_t *hal, uint8_t node_id,
                               uint16_t period_div, uint8_t bus_format,
                               uint8_t mode, uint8_t force_module)
{
    if (!hal || !hal->drv) return -ENODEV;

    uint32_t cfg = (uint32_t)period_div
         | (((uint32_t)bus_format   & SENSOR_CFG_FIELD_MASK) << SENSOR_CFG_BUS_FORMAT_SHIFT)
         | (((uint32_t)mode         & SENSOR_CFG_FIELD_MASK) << SENSOR_CFG_MODE_SHIFT)
         | (((uint32_t)force_module & SENSOR_CFG_FIELD_MASK) << SENSOR_CFG_FORCE_MODULE_SHIFT);

    return sdo_write_simple(hal->drv, node_id, OD_SENSOR_CONFIG,
                            OD_SENSOR_CONFIG_SUB, cfg, 4);
}

/* 兼容旧签名: 默认 SPI 模式 (force_module=1) + mode=2 (0x680 + 0x6B0 全发),
 * 周期由调用方传入 (0.5ms 基准分频, 默认 1=2000Hz). */
int motor_hal_sensor_config(motor_hal_t *hal, uint8_t node_id,
                            uint16_t period_div, uint8_t bus_format)
{
    return motor_hal_sensor_config_ex(hal, node_id, period_div, bus_format,
                                      SENSOR_MODE_ALL, FORCE_MODULE_SPI);
}

int motor_hal_sensor_stop(motor_hal_t *hal, uint8_t node_id)
{
    return motor_hal_sensor_config_ex(hal, node_id, 0, 0, 0, FORCE_MODULE_CAN);
}

/* =====================================================
 * LED 灯控制 (OD 0x5503:06, SDO 读写)
 * ===================================================== */

int motor_hal_led_set(motor_hal_t *hal, uint8_t node_id, const led_config_t *cfg)
{
    if (!hal || !hal->drv || !cfg) return -ENODEV;

    uint32_t val = (uint32_t)(cfg->enable_mask | (cfg->mode & 0x0F))
                 | ((uint32_t)cfg->r << 8)
                 | ((uint32_t)cfg->g << 16)
                 | ((uint32_t)cfg->b << 24);

    return sdo_write_simple(hal->drv, node_id, OD_SENSOR_CONFIG,
                            OD_LED_CTRL_SUB, val, 4);
}

int motor_hal_led_get(motor_hal_t *hal, uint8_t node_id, led_config_t *cfg)
{
    if (!hal || !hal->drv || !cfg) return -ENODEV;

    uint32_t val = 0;
    int ret = sdo_read_simple(hal->drv, node_id, OD_SENSOR_CONFIG,
                              OD_LED_CTRL_SUB, &val);
    if (ret != 0) return ret;

    cfg->enable_mask = (uint8_t)(val & 0xF0);
    cfg->mode        = (uint8_t)(val & 0x0F);
    cfg->r           = (uint8_t)((val >> 8)  & 0xFF);
    cfg->g           = (uint8_t)((val >> 16) & 0xFF);
    cfg->b           = (uint8_t)((val >> 24) & 0xFF);

    return 0;
}

/* =====================================================
 * V1.1 新增对象字典 SDO 接口
 * ===================================================== */

int motor_hal_get_torque_sensor(motor_hal_t *hal, uint8_t node_id, int16_t *torque_001nm)
{
    if (!hal || !hal->drv || !torque_001nm) return -ENODEV;
    uint32_t val = 0;
    int ret = sdo_read_simple(hal->drv, node_id, 0x6077, 0x00, &val);
    *torque_001nm = (int16_t)(val & 0xFFFF);
    return ret;
}

int motor_hal_get_bus_current(motor_hal_t *hal, uint8_t node_id, int32_t *bus_ma)
{
    if (!hal || !hal->drv || !bus_ma) return -ENODEV;
    uint32_t val = 0;
    int ret = sdo_read_simple(hal->drv, node_id, 0x2661, 0x00, &val);
    *bus_ma = (int32_t)val;
    return ret;
}

int motor_hal_store_params(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    return sdo_write_simple(hal->drv, node_id, 0x1010, 0x01, 1, 4);
}

int motor_hal_mit_migrate_scales(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    /* 写 0x2546=20 (Tmax=20Nm, V2 出厂值), 然后 0x2539=1 保存到 Flash */
    int ret = sdo_write_simple(hal->drv, node_id, OD_MIT_TQ_SCALE, 0x00, 20, 4);
    if (ret != 0) {
        PROTO_SEND("[MIT_MIGRATE] M%d write 0x2546=20 FAILED ret=%d", node_id, ret);
        return ret;
    }
    usleep(50000);
    ret = sdo_write_simple(hal->drv, node_id, OD_SAVE_FLASH, 0x00, 1, 4);
    PROTO_SEND("[MIT_MIGRATE] M%d 0x2546=20 saved, Flash ret=%d", node_id, ret);
    return ret;
}

int motor_hal_torque_zero_calib(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;
    PROTO_SEND("[CALIB_ZERO] M%d write 0x2531=2 (zero calibration)", node_id);
    return sdo_write_simple(hal->drv, node_id, 0x2531, 0x00, 2, 4);
}

int motor_hal_torque_calib(motor_hal_t *hal, uint8_t node_id, int32_t torque_mNm)
{
    if (!hal || !hal->drv) return -ENODEV;
    /* opcode=2 | (int24 mNm << 8), 小端; 先转无符号避免负数左移UB */
    int32_t packed = (int32_t)(((uint32_t)torque_mNm << 8) | 0x02);
    PROTO_SEND("[CALIB_TORQUE] M%d torque=%d mNm (%.2f Nm) packed=0x%08X",
               node_id, torque_mNm, torque_mNm / 1000.0, (uint32_t)packed);
    return sdo_write_simple(hal->drv, node_id, 0x2531, 0x00, (uint32_t)packed, 4);
}

int motor_hal_get_sensor(motor_hal_t *hal, uint8_t node_id, motor_sensor_t *s)
{
    if (!hal || !s) return -EINVAL;

    motor_node_t *m = _find_motor(hal, node_id);
    if (!m) return -ENOENT;

    pthread_mutex_lock(&m->sensor_lock);
    memcpy(s, &m->cached_sensor, sizeof(motor_sensor_t));
    pthread_mutex_unlock(&m->sensor_lock);

    return 0;
}

/* =====================================================
 * SDO telemetry: temperature (0x2663) + position (0x6064)
 * 0x300 feedback frame only has valid Iq on RV1126B.
 * Temp/pos polled via dedicated non-RT thread at ~5ms per motor.
 * ===================================================== */

static int motor_hal_poll_sdo_telemetry(motor_hal_t *hal, uint8_t node_id);

static void* _sdo_telemetry_thread_fn(void *arg)
{
    motor_hal_t *hal = (motor_hal_t*)arg;
    
    while (sdo_telemetry_running) {
        for (int i = 0; i < hal->motor_count && sdo_telemetry_running; i++) {
            motor_hal_poll_sdo_telemetry(hal, hal->motors[i].node_id);
        }
        usleep(500);
    }
    return NULL;
}

int motor_hal_sdo_telemetry_start(motor_hal_t *hal)
{
    if (!hal || sdo_telemetry_running) return -EBUSY;
    sdo_telemetry_running = true;
    if (pthread_create(&sdo_telemetry_thread, NULL, _sdo_telemetry_thread_fn, hal) != 0) {
        sdo_telemetry_running = false;
        return -1;
    }
    return 0;
}

int motor_hal_sdo_telemetry_stop(motor_hal_t *hal)
{
    (void)hal;
    if (!sdo_telemetry_running) return 0;
    sdo_telemetry_running = false;
    pthread_join(sdo_telemetry_thread, NULL);
    return 0;
}

int motor_hal_poll_sdo_telemetry(motor_hal_t *hal, uint8_t node_id)
{
    if (!hal || !hal->drv) return -ENODEV;

    uint32_t val = 0;
    int32_t pos = 0;

    /* 0x6064 actual position, counts (温度由 0x6A0 透传帧提供) */
    if (sdo_read_simple(hal->drv, node_id, 0x6064, 0x00, &val) == 0)
    pos = (int32_t)val;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (m) { m->sdo_position = pos; }
    pthread_mutex_unlock(&hal->lock);

    return 0;
}

int motor_hal_get_sdo_temperature(motor_hal_t *hal, uint8_t node_id, int32_t *temp)
{
    if (!hal || !temp) return -EINVAL;

    int ret = -EAGAIN;
    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (m && m->sdo_temp_01c >= 0) {
        *temp = m->sdo_temp_01c;
        ret = 0;
    }
    pthread_mutex_unlock(&hal->lock);
    return ret;
}

int motor_hal_get_sdo_position(motor_hal_t *hal, uint8_t node_id, int32_t *pos)
{
    if (!hal || !pos) return -EINVAL;

    pthread_mutex_lock(&hal->lock);
    motor_node_t *m = _find_motor(hal, node_id);
    if (m) *pos = m->sdo_position;
    pthread_mutex_unlock(&hal->lock);

    return m ? 0 : -ENOENT;
}
