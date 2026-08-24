/*
 * stark_client.h -- 算法控制接口 (Header-Only)
 * 版本: V2.3 | 日期: 2026-07-13 | 维护: zhiqiang.yang
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 依赖: stark_shm.h (共享内存布局)
 * 编译: gcc -O2 your_algo.c -lpthread -lrt -lm
 *
 * 数据方向:
 *   motor_node  -- fb_buffer -->  算法
 *   算法        -- mailbox  -->  motor_node
 *
 * 使用流程:
 *   1. stark_open    - 连接 SHM
 *   2. stark_enable  - 使能电机
 *   3. 控制循环      - 读 stark_fb/imu, 写 stark_multi/torque/speed/position
 *   4. stark_disable - 失能电机
 */
#pragma once

#include "stark_shm.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <linux/futex.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制模式常量, 对应 stark_set_mode */
#define STARK_MODE_PP       1   /* 轮廓位置, 驱动板侧梯形加减速 */
#define STARK_MODE_PV       2   /* 轮廓速度, 驱动板侧梯形加减速 */
#define STARK_MODE_CSP      3   /* 循环同步位置, SYNC 触发 */
#define STARK_MODE_CSV      4   /* 循环同步速度, SYNC 触发 */
#define STARK_MODE_CURRENT  5   /* Q轴电流直控 */
#define STARK_MODE_MIT      6   /* MIT 阻抗控制, 走 0x110 独立帧 */
#define STARK_MODE_TORQUE   7   /* 力矩环, 走 0x100/0x200 */

/* 时间戳辅助 — 始终打点: timestamp_us 是 mailbox 数据字段,
 * RT 侧依赖它计算 mbox_age/ctrl_e2e, 不能因统计开关而失效.
 * clock_gettime(CLOCK_MONOTONIC) 是 vDSO 调用 (~20ns), 开销可忽略. */
static inline uint64_t _stark_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* 客户端句柄 */
typedef struct {
    int           fd;
    stark_shm_t*  shm;
} stark_client_t;

/* 逐帧耗时跟踪句柄 (提前定义, 供反馈读取函数自动打点) */
typedef struct {
    int                 fd;
    stark_trace_shm_t*  shm;
} stark_trace_t;

/* 进程内全局 trace 句柄 (stark_trace_open 设置, close 清除).
 * 每个 .c 文件持有独立副本, 互不影响. */
static stark_trace_t* g_stark_trace = NULL;

static inline void stark_trace_fb_update(stark_client_t* c, stark_trace_t* t);

/* 读取反馈即打点: 只要有反馈数据被读取, 就统计上行耗时.
 * 按帧去重 (ts_shm_write), 同一帧多次读取只统计一次. */
static inline void stark_trace_fb_tick(stark_client_t* c)
{
    if (g_stark_trace && g_stark_trace->shm) stark_trace_fb_update(c, g_stark_trace);
}

static inline void stark_close(stark_client_t* c);

/* -- 生命周期 ---------------------------------------------------- */

static inline int stark_open(stark_client_t* c)
{
    if (!c) return -1;
    stark_close(c);
    c->fd = shm_open(STARK_SHM_NAME, O_RDWR, 0666);
    if (c->fd < 0) return -1;
    c->shm = (stark_shm_t*)mmap(NULL, STARK_SHM_SIZE,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 c->fd, 0);
    if (c->shm == MAP_FAILED) {
        close(c->fd);
        c->fd = -1;
        return -1;
    }
    /* 版本协商: magic/version 不符则拒绝 (防新旧进程混跑读错偏移) */
    if (c->shm->magic != STARK_SHM_MAGIC || c->shm->version != STARK_SHM_VERSION) {
        munmap(c->shm, STARK_SHM_SIZE);
        close(c->fd);
        c->shm = NULL;
        c->fd  = -1;
        return -2;
    }
    return 0;
}

static inline void stark_close(stark_client_t* c)
{
    if (!c || !c->shm) return;
    munmap(c->shm, STARK_SHM_SIZE);
    close(c->fd);
    c->shm = NULL;
    c->fd  = -1;
}

/* -- 状态查询 ---------------------------------------------------- */

/* 电机就绪: calib_state==2 表示可控制.*/
static inline int stark_ready(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return (c->shm->calib_state == 2);
}

static inline int stark_online(stark_client_t* c, int id)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return 0;
    return (c->shm->motor_online & (1 << (id - 1))) != 0;
}

static inline int stark_state(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return __atomic_load_n(&c->shm->node_state, __ATOMIC_ACQUIRE);
}

static inline int stark_calib(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return c->shm->calib_state;
}

/* 校准进行中: calib_state==1 */
static inline int stark_is_calibrating(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return (c->shm->calib_state == 1);
}

static inline int stark_severity(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return c->shm->motor_severity;
}

static inline int stark_fault_reason(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return c->shm->fault_reason;
}

/* -- 反馈读取 (零拷贝, 读 SHM 双 Buffer active 端) --------------- */

static inline motor_data_t stark_fb(stark_client_t* c, int id)
{
    motor_data_t fb = {0};
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return fb;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    stark_trace_fb_tick(c);   /* 读取即打点 (上行耗时统计) */
    const feedback_frame_t* f = &c->shm->fb_buffer[idx];
    return f->motor[id - 1];
}

static inline imu_data_t stark_imu(stark_client_t* c)
{
    imu_data_t imu = {0};
    if (!c || !c->shm) return imu;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    stark_trace_fb_tick(c);
    return c->shm->fb_buffer[idx].imu;
}

static inline stark_sensor_data_t stark_sensor(stark_client_t* c, int id)
{
    stark_sensor_data_t s = {0};
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return s;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    stark_trace_fb_tick(c);
    return c->shm->fb_buffer[idx].sensor[id - 1];
}

static inline barometer_data_t stark_baro(stark_client_t* c)
{
    barometer_data_t b = {0};
    if (!c || !c->shm) return b;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    stark_trace_fb_tick(c);
    return c->shm->fb_buffer[idx].baro;
}

static inline foot_pressure_data_t stark_foot_pressure(stark_client_t* c)
{
    foot_pressure_data_t fp = {0};
    if (!c || !c->shm) return fp;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    stark_trace_fb_tick(c);
    return c->shm->fb_buffer[idx].foot_pressure;
}

/* -- 控制命令: 实时路径 (环形缓冲 mailbox, 不丢帧) ------------- */

/* SPSC 环形缓冲写入, 返回槽位索引. 缓冲满时短暂自旋等待. */
static inline int _stark_mbox_begin(stark_client_t* c)
{
    if (!c || !c->shm) return -1;
    uint64_t w, r;
    do {
        w = __atomic_load_n(&c->shm->mailbox.seq_write, __ATOMIC_RELAXED);
        r = __atomic_load_n(&c->shm->mailbox.seq_read,  __ATOMIC_ACQUIRE);
        if (w - r >= STARK_MBOX_DEPTH) usleep(50);  /* 缓冲满 */
    } while (w - r >= STARK_MBOX_DEPTH);
    return (int)(w % STARK_MBOX_DEPTH);
}

/* 提交写入: 递增 seq_write, 通知 RT */
static inline void _stark_mbox_commit(stark_client_t* c)
{
    __atomic_add_fetch(&c->shm->mailbox.seq_write, 1, __ATOMIC_RELEASE);
}

/* 电流控制, 单位 mA */
static inline void stark_torque(stark_client_t* c, int id, int32_t ma)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_TORQUE;
    c->shm->mailbox.frames[slot].cmd[idx].value    = ma;
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* 力矩环控制 (V2 新增), 单位 0.05N.m, 通过 0x100+mode=6 控制 */
static inline void stark_torque_ctrl(stark_client_t* c, int id, int32_t torque_005nm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_TORQUE_CTRL;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = torque_005nm;
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* 速度控制 (CSV), 单位 RPM */
static inline void stark_speed(stark_client_t* c, int id, float rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_SPEED;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* 轮廓速度 (PV), rpm=目标速度 accel=加速度 RPM/s */
static inline void stark_pv(stark_client_t* c, int id, float rpm, float accel)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_PV;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].value2   = (int32_t)(accel * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* 循环同步速度 (CSV), 单位 RPM */
static inline void stark_csv(stark_client_t* c, int id, float rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_CSV;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* 绝对位置控制 (CSP), 单位 deg, 范围 [-180, 180] */
static inline void stark_position(stark_client_t* c, int id, float deg)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd      = STARK_CMD_POS;
    c->shm->mailbox.frames[slot].cmd[idx].value    = (int32_t)(deg * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* 相对位置控制, 自动读当前位置加偏移, 钳位到 [-180, 180] */
static inline void stark_rel_position(stark_client_t* c, int id, float delta_deg)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    motor_data_t fb = stark_fb(c, id);
    float cur_deg  = (float)fb.position * (360.0f / 65536.0f);
    float target   = cur_deg + delta_deg;

    if (target > 180.0f)  target -= 360.0f;
    if (target < -180.0f) target += 360.0f;

    stark_position(c, id, target);
}

/* 轮廓位置 (PP), deg=目标角度 accel=加速度RPM/s vel=轮廓速度RPM */
static inline void stark_pp(stark_client_t* c, int id,
                             float deg, float accel_rpm, float vel_rpm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id    = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd         = STARK_CMD_PP;
    c->shm->mailbox.frames[slot].cmd[idx].value       = (int32_t)(deg * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].value2      = (int32_t)(accel_rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].feedforward = (int32_t)(vel_rpm * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* MIT 阻抗控制, pos_deg=平衡点 vel_rpm=阻尼速度 kp=刚度 kd=阻尼 torque=前馈力矩 Nm */
static inline void stark_mit(stark_client_t* c, int id,
                              float pos_deg, float vel_rpm,
                              float kp, float kd, float torque)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    int idx = id - 1;

    c->shm->mailbox.frames[slot].cmd[idx].motor_id   = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[idx].cmd        = STARK_CMD_MIT;
    c->shm->mailbox.frames[slot].cmd[idx].mit_pos    = (uint16_t)((pos_deg + 180.0f) * 65535.0f / 360.0f);
    c->shm->mailbox.frames[slot].cmd[idx].mit_vel    = (int16_t)(vel_rpm);
    c->shm->mailbox.frames[slot].cmd[idx].mit_kp     = (uint16_t)(kp * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].mit_kd     = (uint16_t)(kd * 100.0f);
    c->shm->mailbox.frames[slot].cmd[idx].mit_torque = (int16_t)(torque);
    c->shm->mailbox.frames[slot].cmd[idx].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* MIT 多轴广播 (0x210, 64Byte CAN FD, 双电机一帧) */
static inline void stark_mit_multi(stark_client_t* c,
                                     float pos1, float vel1, float kp1, float kd1, float tq1,
                                     float pos2, float vel2, float kp2, float kd2, float tq2)
{
    if (!c || !c->shm) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));

    /* M1 */
    c->shm->mailbox.frames[slot].cmd[0].motor_id   = 1;
    c->shm->mailbox.frames[slot].cmd[0].cmd        = STARK_CMD_MIT_MULTI;
    c->shm->mailbox.frames[slot].cmd[0].mit_pos    = (uint16_t)((pos1 + 180.0f) * 65535.0f / 360.0f);
    c->shm->mailbox.frames[slot].cmd[0].mit_vel    = (int16_t)(vel1);
    c->shm->mailbox.frames[slot].cmd[0].mit_kp     = (uint16_t)(kp1 * 100.0f);
    c->shm->mailbox.frames[slot].cmd[0].mit_kd     = (uint16_t)(kd1 * 100.0f);
    c->shm->mailbox.frames[slot].cmd[0].mit_torque = (int16_t)(tq1);
    c->shm->mailbox.frames[slot].cmd[0].timestamp_us = _stark_now_us();

    /* M2 */
    c->shm->mailbox.frames[slot].cmd[1].motor_id   = 2;
    c->shm->mailbox.frames[slot].cmd[1].cmd        = STARK_CMD_MIT_MULTI;
    c->shm->mailbox.frames[slot].cmd[1].mit_pos    = (uint16_t)((pos2 + 180.0f) * 65535.0f / 360.0f);
    c->shm->mailbox.frames[slot].cmd[1].mit_vel    = (int16_t)(vel2);
    c->shm->mailbox.frames[slot].cmd[1].mit_kp     = (uint16_t)(kp2 * 100.0f);
    c->shm->mailbox.frames[slot].cmd[1].mit_kd     = (uint16_t)(kd2 * 100.0f);
    c->shm->mailbox.frames[slot].cmd[1].mit_torque = (int16_t)(tq2);
    c->shm->mailbox.frames[slot].cmd[1].timestamp_us = _stark_now_us();

    _stark_mbox_commit(c);
}

/* -- SDO 控制命令 (通过 mailbox, RT 转发到主循环) ----- */

#define STARK_SDO_DEFAULT_ACCEL  500
#define STARK_SDO_DEFAULT_VEL    10

/* SDO 电流控制, 单位 mA */
static inline void stark_sdo_cur(stark_client_t* c, int id, int32_t ma)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;
    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SDO_CUR;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = ma;
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* SDO 绝对位置 (PP), deg=角度° accel=加速度RPM/s vel=轮廓速度RPM */
static inline void stark_sdo_pos(stark_client_t* c, int id, float deg,
                                  float accel, float vel)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;
    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SDO_POS;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = (int32_t)(deg * 100.0f);
    c->shm->mailbox.frames[slot].cmd[id - 1].value2   = (int32_t)(accel * 100.0f);
    c->shm->mailbox.frames[slot].cmd[id - 1].feedforward = (int32_t)(vel * 100.0f);
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* SDO 轮廓位置 (PP, SDO 通过 mailbox → main_loop → StarkMotorCtrl) */
static inline void stark_sdo_vel(stark_client_t* c, int id, int32_t rpm,
                                  int32_t accel)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;
    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SDO_VEL;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = (int32_t)(rpm * 100);
    c->shm->mailbox.frames[slot].cmd[id - 1].value2   = (int32_t)(accel * 100);
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* SDO 力矩标定, value=torque_mNm */
static inline void stark_sdo_torque_calib(stark_client_t* c, int id, int32_t torque_mNm)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;
    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SDO_TORQUE_CALIB;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = torque_mNm;
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* SDO MIT 缩放迁移: 写 0x2546=20(Tmax) + 0x2539=1 保存 Flash */
static inline void stark_sdo_mit_migrate(stark_client_t* c, int id)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;
    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SDO_MIT_MIGRATE;
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* SDO 轮廓速度 (PV, SDO 通过 mailbox → main_loop → StarkMotorCtrl) */

/*
 * 多轴广播, 一帧 64B CANFD 同时控制双电机.
 *
 * 参数: t1/t2=力矩(mA) v1/v2=速度前馈(RPM) p1/p2=位置(deg)
 * 实际生效的字段取决于当前控制模式:
 *   CURRENT 模式: 只用 t1/t2, v 和 p 填 0
 *   CSP 模式:     p1/p2 输入绝对角度
 *   CSV/PV 模式:  v1/v2 输入速度
 */
static inline void stark_multi(stark_client_t* c, int mode,
                                int32_t t1, int32_t v1, int32_t p1,
                                int32_t t2, int32_t v2, int32_t p2)
{
    if (!c || !c->shm) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;

    c->shm->mailbox.frames[slot].cmd[0].motor_id    = 1;
    c->shm->mailbox.frames[slot].cmd[0].cmd         = STARK_CMD_MULTI;
    c->shm->mailbox.frames[slot].cmd[0].multi_mode  = (uint8_t)mode;
    c->shm->mailbox.frames[slot].cmd[0].value       = t1;
    c->shm->mailbox.frames[slot].cmd[0].value2      = v1;
    c->shm->mailbox.frames[slot].cmd[0].feedforward = p1;

    c->shm->mailbox.frames[slot].cmd[1].motor_id    = 2;
    c->shm->mailbox.frames[slot].cmd[1].cmd         = STARK_CMD_MULTI;
    c->shm->mailbox.frames[slot].cmd[1].multi_mode  = (uint8_t)mode;
    c->shm->mailbox.frames[slot].cmd[1].value       = t2;
    c->shm->mailbox.frames[slot].cmd[1].value2      = v2;
    c->shm->mailbox.frames[slot].cmd[1].feedforward = p2;

    {
        uint64_t ts = _stark_now_us();
        c->shm->mailbox.frames[slot].cmd[0].timestamp_us = ts;
        c->shm->mailbox.frames[slot].cmd[1].timestamp_us = ts;
    }

    _stark_mbox_commit(c);
}

/* -- 管理命令 (per-motor mgmt slot, 不和算法 mailbox 竞争) --- */

/*
 * enable/disable/estop/recover/clear_fault 走 mgmt 通道.
 * 每电机独立 slot (mgmt_cmd[id-1] / mgmt_seq[id-1] / mgmt_ack[id-1]),
 * 不受其他电机或 mailbox 控制循环的 seq 覆盖影响.
 */
static inline void _stark_mgmt_cmd(stark_client_t* c, int id, int cmd)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int idx = id - 1;
    c->shm->mgmt_cmd[idx] = (uint8_t)cmd;
    /* 写屏障: cmd 必须在 seq 递增前对其他核心可见 */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_add_fetch(&c->shm->mgmt_seq[idx], 1, __ATOMIC_RELEASE);
}

/* 使能电机 */
static inline void stark_enable(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_ENABLE); }

/*
 * 失能电机: 先 PDO 失能, 再通过多轴广播发 disable+release_brake+电流0,
 * 确保电机停止后刹车松开.
 */
static inline void stark_disable(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_DISABLE); }

/*
 * 急停: PDO 失能 + bus=OFF (刹车抱死), 再发 disable+brake_hold+电流0.
 */
static inline void stark_estop(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_ESTOP); }

/* 从急停恢复 */
static inline void stark_recover(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_RECOVER); }

/*
 * 清除故障: PDO 清故障位 + 自动使能 + 切电流模式 target=0.
 * 清障后电机会恢复使能状态, 可直接进入控制循环.
 */
static inline void stark_clear_fault(stark_client_t* c, int id)
    { _stark_mgmt_cmd(c, id, STARK_CMD_CLEAR_FAULT); }

/* 按键 3连击等效. 调用后轮询 stark_ready/stark_is_calibrating 获取状态. */
static inline void stark_request_calib(stark_client_t* c)
{
    if (!c || !c->shm) return;
    __atomic_store_n(&c->shm->calib_requested, 1, __ATOMIC_RELEASE);
}

/* 切换控制模式, mode 取值见 STARK_MODE_* 常量.
 * set_mode 在控制循环启动前调用, 无 stark_multi 竞争, 走 mailbox 即可. */
static inline void stark_set_mode(stark_client_t* c, int id, int mode)
{
    if (!c || !c->shm || id < 1 || id > STARK_MAX_MOTORS) return;

    int slot = _stark_mbox_begin(c);
    if (slot < 0) return;
    memset(&c->shm->mailbox.frames[slot], 0, sizeof(mailbox_frame_t));
    c->shm->mailbox.frames[slot].cmd[id - 1].motor_id = (uint8_t)id;
    c->shm->mailbox.frames[slot].cmd[id - 1].cmd      = STARK_CMD_SET_MODE;
    c->shm->mailbox.frames[slot].cmd[id - 1].value    = mode;
    c->shm->mailbox.frames[slot].cmd[id - 1].timestamp_us = _stark_now_us();
    _stark_mbox_commit(c);
}

/* -- 双向心跳 -------------------------------------------------- */

/* 算法声明存活, 建议每 200ms 调一次 */
static inline void stark_heartbeat(stark_client_t* c)
{
    if (!c || !c->shm) return;
    __atomic_add_fetch(&c->shm->algo_heartbeat, 1, __ATOMIC_RELEASE);
}

/* stark_node 反向存活检测, 同频调用. 返回 0 需重连 */
static inline int stark_rt_alive(stark_client_t* c, uint32_t *last_cycle)
{
    if (!c || !c->shm) return 0;
    if (!stark_ready(c)) return 0;
    uint32_t cur = __atomic_load_n(&c->shm->rt_cycle, __ATOMIC_ACQUIRE);
    if (cur == *last_cycle) return 0;
    *last_cycle = cur;
    return 1;
}

/* -- LED 灯控制 (非 RT, 主循环处理) --------------------------------- */

/*
 * LED 灯效控制. mask: LED1~4 掩码, mode: 灯效模式, r/g/b: 0-255.
 *
       # 全灭            mask  mode  R  G  B
       ./demo_algo led 2 0x00  0     0  0  0

       # 四灯全亮常亮 RGB
       ./demo_algo led 2 0xF0 0 255 0 0    # 红
       ./demo_algo led 2 0xF0 0 0 255 0    # 绿
       ./demo_algo led 2 0xF0 0 0 0 255    # 蓝
       ./demo_algo led 2 0xF0 0 255 255 0  # 黄
       ./demo_algo led 2 0xF0 0 255 0 255  # 紫

       # 单灯 test（常亮红）
       ./demo_algo led 2 0x10 0 255 0 0    # LED1
       ./demo_algo led 2 0x20 0 255 0 0    # LED2
       ./demo_algo led 2 0x40 0 255 0 0    # LED3
       ./demo_algo led 2 0x80 0 255 0 0    # LED4

       # 四灯全亮，各模式对比  ./demo_algo led 2 0xF0 0 0 255 0    # 常亮 绿
       ./demo_algo led 2 0xF0 1 0 255 0    # 闪烁 绿
       ./demo_algo led 2 0xF0 2 0 255 0    # 呼吸 绿
       ./demo_algo led 2 0xF0 3 0 255 0    # 流水 绿
*/

static inline void stark_led_ctrl(stark_client_t* c, int motor_id,
                                   uint8_t mask, uint8_t mode,
                                   uint8_t r, uint8_t g, uint8_t b)
{
    if (!c || !c->shm || motor_id < 1 || motor_id > STARK_MAX_MOTORS) return;
    int i = motor_id - 1;
    c->shm->led_cfg[i].enable_mask = mask;
    c->shm->led_cfg[i].mode        = mode;
    c->shm->led_cfg[i].r           = r;
    c->shm->led_cfg[i].g           = g;
    c->shm->led_cfg[i].b           = b;
    __atomic_add_fetch(&c->shm->led_seq[i], 1, __ATOMIC_RELEASE);
}

/* 双电机 LED 控制 */
static inline void stark_led_ctrl_all(stark_client_t* c,
                                       uint8_t mask, uint8_t mode,
                                       uint8_t r, uint8_t g, uint8_t b)
{
    stark_led_ctrl(c, 1, mask, mode, r, g, b);
    stark_led_ctrl(c, 2, mask, mode, r, g, b);
}

/* -- 按键 B 状态 (GPIO 线程写, 算法只读) ------------------------ */

static inline uint8_t stark_btn_state(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return __atomic_load_n(&c->shm->btn_report_state, __ATOMIC_ACQUIRE);
}

static inline uint32_t stark_btn_seq(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return __atomic_load_n(&c->shm->btn_report_seq, __ATOMIC_ACQUIRE);
}

/* -- 周期上报 (5ms 自动推送, 校准完成后自动开启) ------------------ */

/* 返回周期上报数据指针, 未开启时返回 NULL */
static inline const PeriodicUploadData* stark_report_data(stark_client_t* c)
{
    if (!c || !c->shm || !c->shm->periodic_enabled) return NULL;
    return &c->shm->periodic_data;
}

/* 上报版本号, 单调递增, 对比上次可检测数据更新 */
static inline uint32_t stark_report_version(stark_client_t* c)
{
    if (!c || !c->shm) return 0;
    return __atomic_load_n(&c->shm->periodic_version, __ATOMIC_ACQUIRE);
}

/*
 * 尝试读取新数据. 封装版本号比对, 控制循环内一行调用.
 * 用法: stark_report_try_read(&c, &ver, &d), 返回 1 表示新数据, d 指向 SHM 零拷贝
 */
static inline int stark_report_try_read(stark_client_t* c, uint32_t *last_ver,
                                         const PeriodicUploadData** out)
{
    if (!c || !c->shm || !c->shm->periodic_enabled) return 0;
    uint32_t cur = __atomic_load_n(&c->shm->periodic_version, __ATOMIC_ACQUIRE);
    if (cur == *last_ver) return 0;
    *last_ver = cur;
    *out = &c->shm->periodic_data;
    return 1;
}

/*
 * 阻塞等待新的周期上报数据. 无新数据时通过 futex 让出 CPU,
 * 有新数据或超时后返回. 适合算法侧单独开一个数据接收线程被动等待,
 * 主控制循环不必轮询.
 *
 * 参数:
 *   last_ver   算法侧持有的版本号, 首次传入 0, 函数内部更新
 *   out        输出参数, 指向 SHM 内零拷贝数据, 仅返回 1 时有效
 *   timeout_ms 最长等待毫秒, 传 <0 表示无限等待
 * 返回:
 *   1  有新数据, *out 有效, *last_ver 已更新
 *   0  超时, 上报未开启, 或被信号打断, 无新数据
 *
 * 与 stark_report_try_read 共用 periodic_version, 两种取数方式可自由
 * 选择且互不影响. 唤醒由 stark 节点在每次上报后 futex_wake 触发.
 * 内部使用共享 futex (不带 FUTEX_PRIVATE_FLAG), 支持跨进程共享内存.
 */
static inline int stark_report_wait(stark_client_t* c, uint32_t *last_ver,
                                    const PeriodicUploadData** out,
                                    int timeout_ms)
{
    if (!c || !c->shm || !last_ver || !out) return 0;
    if (!c->shm->periodic_enabled) return 0;

    for (;;) {
        uint32_t cur = __atomic_load_n(&c->shm->periodic_version, __ATOMIC_ACQUIRE);
        if (cur != *last_ver) {
            *last_ver = cur;
            *out = &c->shm->periodic_data;
            return 1;
        }

        struct timespec  ts;
        struct timespec *pts = NULL;
        if (timeout_ms >= 0) {
            ts.tv_sec  = timeout_ms / 1000;
            ts.tv_nsec = (long)(timeout_ms % 1000) * 1000000L;
            pts = &ts;
        }

        /* 版本仍为 cur 时睡眠, 等待写端递增并 futex_wake.
         * FUTEX_WAIT 的 timeout 为相对时间. */
        long r = syscall(SYS_futex, &c->shm->periodic_version,
                         FUTEX_WAIT, cur, pts, NULL, 0);
        if (r == 0)                          continue;  /* 被唤醒, 重新比对版本 */
        if (errno == EAGAIN)                 continue;  /* 版本已变, 回头读到新数据 */
        if (errno == EINTR && timeout_ms < 0) continue; /* 无限等待被信号打断, 重进 */
        return 0;                                       /* 超时 / 被打断, 本次无新数据 */
    }
}

/* -- 逐帧耗时跟踪 (独立 trace SHM, 上行打点) -------------------- */

static inline int stark_trace_open(stark_trace_t* t)
{
    if (!t) return -1;
    t->fd  = -1;
    t->shm = NULL;
    int fd = shm_open(STARK_TRACE_SHM_NAME, O_RDWR, 0666);
    if (fd < 0) return -1;
    void* p = mmap(NULL, STARK_TRACE_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) { close(fd); return -1; }
    t->fd  = fd;
    t->shm = (stark_trace_shm_t*)p;
    g_stark_trace = t;   /* 注册全局句柄, 反馈读取函数自动打点 */
    return 0;
}

static inline void stark_trace_close(stark_trace_t* t)
{
    if (!t || !t->shm) return;
    munmap(t->shm, STARK_TRACE_SHM_SIZE);
    close(t->fd);
    t->shm = NULL;
    t->fd  = -1;
    g_stark_trace = NULL;
}

/*
 * 上行反馈打点 (demo_algo 读透传数据后调用):
 *   读 active fb_buffer 的 ts_can_rx / ts_shm_read 时间戳,
 *   计算 up_seg2 = now - ts_shm_read, up_total = now - ts_can_rx,
 *   聚合统计并逐帧写 fb ring (上行总曲线).
 *
 * 真实性保障:
 *   - 校验 ts_can_rx/ts_shm_read 合法性, 时间戳单调性
 *   - 按 ts_shm_write 去重, 每帧只统计一次
 *   - 过滤异常大值 (>100ms, 跨周期错位/电机离线)
 */
static inline void stark_trace_fb_update(stark_client_t* c, stark_trace_t* t)
{
    if (!c || !c->shm || !t || !t->shm) return;
    if (!__atomic_load_n(&t->shm->enabled, __ATOMIC_ACQUIRE)) return;

    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    const feedback_frame_t* f = &c->shm->fb_buffer[idx];

    uint64_t now      = _stark_now_us();
    uint64_t can_rx   = f->ts_can_rx;
    uint64_t shm_read = f->ts_shm_read;

    /* 时间戳合法性校验, 防虚假耗时 */
    if (can_rx == 0 || shm_read == 0) return;
    if (shm_read < can_rx || now < shm_read) return;

    /* 按帧去重: 同一 ts_shm_write (组装时刻) 只统计一次 */
    static uint64_t last_ts_shm_write = 0;
    if (f->ts_shm_write == last_ts_shm_write) return;
    last_ts_shm_write = f->ts_shm_write;

    uint64_t seg2  = now - shm_read;   /* rt读 → demo_algo读到 */
    uint64_t total = now - can_rx;     /* can0 → demo_algo读到 */
    if (seg2 > 100000 || total > 100000) return;   /* 过滤异常值 */

    /* 聚合统计 (单写者: demo_algo) */
    trace_stat_update(&t->shm->up_seg2, seg2);
    trace_stat_update(&t->shm->up_total, total);

    /* 逐帧写 fb ring (上行总曲线) */
    uint32_t h = __atomic_load_n(&t->shm->fb_head, __ATOMIC_RELAXED);
    trace_sample_t* s = &t->shm->fb_samples[h % STARK_TRACE_FB_RING];
    s->cycle    = (uint32_t)(__atomic_load_n(&c->shm->rt_cycle, __ATOMIC_ACQUIRE) & 0xFFFFFFFF);
    s->kind     = TRACE_KIND_FB;
    s->e2e_us   = (uint16_t)(total > 65535 ? 65535 : total);
    s->motor_id = 0;
    s->reserved = 0;
    s->ts_us    = (uint32_t)(can_rx & 0xFFFFFFFF);
    __atomic_store_n(&t->shm->fb_head, h + 1, __ATOMIC_RELEASE);
}

#ifdef __cplusplus
}
#endif
