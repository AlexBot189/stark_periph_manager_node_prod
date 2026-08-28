/*
 * stark_rt_worker.cpp — RT 工作线程实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 关键约束:
 *   Run() 循环内禁止阻塞调用 (SDO/sleep/malloc)
 *   仅用 motor_hal non-blocking API (get_feedback / multi_ctrl / multi_pdo)
 *   安全动作 (torque=0 / disable) 走 PDO 路径
 */
#include "motor/motor_rt_worker.h"
#include "utils/rt_log.h"
#include "motor/motor_ctrl.h"
#include "sensor/imu/imu_source.h"
#include "sensor/foot_pressure/FootPressureSensor.h"
#include "sensor/barometer/barometer_source.h"
#include <log_helper/LogHelper.h>

#include <cstring>
#include <ctime>
#include <cassert>
#include <climits>
#include <cstdlib>
#include <cstdio>
#include <sched.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <linux/futex.h>
#include <unistd.h>

namespace stark_periph_manager_node {

/* CLOCK_MONOTONIC 当前时刻 (us), vDSO 直取 ~20ns 开销 */
static inline uint64_t _rt_now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* 构造 / 析构 */

StarkRtWorker::StarkRtWorker(motor_hal_t* hal, stark_shm_t* shm,
                         StarkMotorCtrl* ctrl, IImuSource* imu_sensor,
                         FootPressureSensor* foot_sensor,
                         IBarometerSource* baro_sensor,
                         int motor_count)
    : m_hal(hal)
    , m_shm(shm)
    , m_ctrl(ctrl)
    , m_imu_sensor(imu_sensor)
    , m_foot_sensor(foot_sensor)
    , m_baro_sensor(baro_sensor)
    , m_report_divider(m_rt.report_divider)
    , m_report_enabled(false)
    , m_report_period_ms(5)
    , m_periodic_last_cycle(0)
    , m_cycle_count(0)
    , m_overrun_count(0)
    , m_jitter_min_us(UINT32_MAX)
    , m_jitter_max_us(0)
    , m_jitter_acc_us(0)
    , m_jitter_cnt(0)
    , m_motor_count(motor_count)
{
    memset(m_last_position, 0, sizeof(m_last_position));
    memset(m_pos_stall_us, 0, sizeof(m_pos_stall_us));
    memset(m_sensor_notified, 0, sizeof(m_sensor_notified));
    /* 初始: 无效模式 (0xFF), 确保首帧触发模式切换双发, 防首帧丢失 */
    for (int i = 0; i < STARK_MAX_MOTORS; i++) m_last_pdo_mode[i] = (motor_mode_t)0xFF;
    memset(m_pending_retry, 0, sizeof(m_pending_retry));
    m_imu_notified = false;
}

StarkRtWorker::~StarkRtWorker()
{
    if (m_running.load(std::memory_order_acquire)) {
        Stop();
    }
}

/* Start() / Stop() */

void StarkRtWorker::Start()
{
    if (m_running.load(std::memory_order_acquire)) return;
    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&StarkRtWorker::Run, this);
}

void StarkRtWorker::Stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void StarkRtWorker::SetRtConfig(const RtConfig& cfg)
{
    m_rt = cfg;
    m_report_divider      = cfg.report_divider;  /* 立即生效 */
}

void StarkRtWorker::SetReportEnabled(bool enabled, uint32_t period_ms)
{
    m_report_enabled = enabled;
    if (period_ms >= 1 && period_ms <= 1000) {
        m_report_period_ms = period_ms;
    }
    /* 首次触发: 从当前 RT 周期开始计时 */
    m_periodic_last_cycle = m_cycle_count;
    if (m_shm) {
        m_shm->periodic_enabled   = enabled ? 1 : 0;
        m_shm->periodic_period_ms = m_report_period_ms;
    }
}

void StarkRtWorker::SetDataSource(const std::string& src)
{
    if (src == "unified_6c0") {
        m_data_source = DS_UNIFIED_6C0;
    } else {
        m_data_source = DS_MIXED;
    }
    ECO_INFO_NEW("[RT] data_source={}", m_data_source == DS_UNIFIED_6C0 ? "unified_6c0" : "mixed");
}

/* Run() — RT 线程主循环 1KHz */

void StarkRtWorker::Run()
{
    SetThreadRt();

    if (m_shm) {
        m_shm->rt_mode = m_rt.enable_rt ? 1 : 0;
        m_shm->period_us = (uint16_t)m_rt.period_us;
        m_shm->rt_priority = (uint8_t)m_rt.priority;
        m_shm->rt_cpu = (uint8_t)m_rt.cpu_affinity[0];
        m_shm->perf_trace_enabled = m_rt.perf_trace ? 1 : 0;
    }

    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    const long period_ns = (long)m_rt.period_us * 1000L;

    while (m_running.load(std::memory_order_acquire)) {

        m_cur_jitter = 0;   /* 本周期抖动, 睡眠后测量 */

        /* 消费 perf_reset_request: web/算法下发"清空"→清零本轮 max, 重新累计 */
        if (m_shm && __atomic_load_n(&m_shm->perf_reset_request, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&m_shm->perf_reset_request, 0, __ATOMIC_RELEASE);
            m_jitter_max_us    = 0;
        }

        ProcessMgmt();
        ProcessMailbox();
        PublishFeedback();

        m_cycle_count++;

        if (m_shm) m_shm->rt_cycle = (uint32_t)(m_cycle_count & 0xFFFFFFFF);

        /* 精确周期休眠 */
        next_wake.tv_nsec += period_ns;
        while (next_wake.tv_nsec >= 1000000000L) {
            next_wake.tv_nsec -= 1000000000L;
            next_wake.tv_sec++;
        }

        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                  &next_wake, nullptr);

        /* 周期抖动统计 (仅 rt.perf_trace=true 时) :
         * 实际唤醒时刻 - 理想目标时刻, 这是"错过截止期"的真实度量.
         * 注意: clock_nanosleep 返回非0 = 被信号打断(EINTR), 并非超限;
         * 原实现用 ret!=0 统计超限是错误的(永远≈0), 已修正为 actual>target. */
        m_cur_jitter = 0;
        if (m_rt.perf_trace) {
            struct timespec now_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            uint64_t actual_us = (uint64_t)now_ts.tv_sec * 1000000ULL + (uint64_t)now_ts.tv_nsec / 1000ULL;
            uint64_t target_us = (uint64_t)next_wake.tv_sec * 1000000ULL + (uint64_t)next_wake.tv_nsec / 1000ULL;
            if (actual_us > target_us) {
                uint32_t j = (uint32_t)(actual_us - target_us);
                m_cur_jitter = j;
                if (j < m_jitter_min_us) m_jitter_min_us = j;
                if (j > m_jitter_max_us) m_jitter_max_us = j;
                m_jitter_acc_us += j;
                m_jitter_cnt++;
                if (j > 100) m_overrun_count++;   /* 抖动 >100us 才算周期超限 */
            }
        }

        /* 被信号打断(EINTR)等异常: 重新对齐绝对唤醒基准, 防止漂移 */
        if (ret != 0) {
            clock_gettime(CLOCK_MONOTONIC, &next_wake);
        }

        /* 每1000周期 (~1s): 输出周期抖动统计 */
        if (m_rt.perf_trace && (m_cycle_count % 1000) == 0 && m_jitter_cnt > 0) {
            uint32_t avg = (uint32_t)(m_jitter_acc_us / m_jitter_cnt);
            RT_LOG(RT_LOG_INFO, CYCLE_JITTER, m_jitter_min_us, avg, m_jitter_max_us, m_jitter_cnt);
            RT_LOG(RT_LOG_INFO, CYCLE_OVERRUN,
                   (uint32_t)m_overrun_count, (uint32_t)(m_overrun_count >> 32), 0, 0);
            if (m_shm) {
                m_shm->cycle_jitter_max_us = m_jitter_max_us;
            }
            m_jitter_min_us = UINT32_MAX;
            /* m_jitter_max_us 不清零 — 本轮累计, 由 perf_reset_request 清零 */
            m_jitter_acc_us = 0;
            m_jitter_cnt = 0;
        }

        /* 逐帧监控: 写本周期抖动样本到 jitter ring (16字节, 开销可忽略) */
        PublishTrace();
    }
}

/*
 * ProcessMgmt() — 读 SHM mgmt 通道, 处理管理命令 (独立于 mailbox)
 *
 * 每电机独立 slot: mgmt_cmd[id-1] / mgmt_seq[id-1] / mgmt_ack[id-1].
 * 写入方递增 mgmt_seq[i], RT 处理后将 seq 拷贝到 mgmt_ack[i].
 * 多电机命令互不覆盖, 不依赖 mailbox seq.
 */

void StarkRtWorker::ProcessMgmt()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm || !m_hal) return;

    for (int i = 0; i < m_motor_count; i++) {
        uint8_t seq = __atomic_load_n(&m_shm->mgmt_seq[i], __ATOMIC_ACQUIRE);
        uint8_t ack = __atomic_load_n(&m_shm->mgmt_ack[i], __ATOMIC_RELAXED);

        if (seq == ack) continue;

        uint8_t cmd = m_shm->mgmt_cmd[i];
        uint8_t id  = (uint8_t)(i + 1);

        if (cmd == 0) {
            __atomic_store_n(&m_shm->mgmt_ack[i], seq, __ATOMIC_RELEASE);
            continue;
        }

        switch (cmd) {
        case STARK_CMD_ENABLE:
            motor_hal_pdo_enable(m_hal, id);
            break;
        case STARK_CMD_DISABLE:
            motor_hal_pdo_disable(m_hal, id);
            {
                multi_axis_cmd_t mcmd = {};
                mcmd.node_id       = id;
                mcmd.enable        = false;
                mcmd.release_brake = true;
                mcmd.mode          = MOTOR_MODE_CURRENT;
                mcmd.target1       = 0;
                motor_hal_multi_ctrl(m_hal, &mcmd, 1);
            }
            break;
        case STARK_CMD_ESTOP:
            motor_hal_pdo_estop(m_hal, id);
            {
                multi_axis_cmd_t mcmd = {};
                mcmd.node_id       = id;
                mcmd.enable        = false;
                mcmd.release_brake = false;
                mcmd.mode          = MOTOR_MODE_CURRENT;
                mcmd.target1       = 0;
                motor_hal_multi_ctrl(m_hal, &mcmd, 1);
            }
            break;
        case STARK_CMD_RECOVER:
            motor_hal_pdo_recover(m_hal, id);
            break;
        case STARK_CMD_CLEAR_FAULT:
            motor_hal_pdo_clear_fault(m_hal, id);
            motor_hal_pdo_enable(m_hal, id);
            motor_hal_ctrl_raw(m_hal, id, MOTOR_MODE_CURRENT, 0, 0, 0);
            break;
        default:
            break;
        }

        __atomic_store_n(&m_shm->mgmt_ack[i], seq, __ATOMIC_RELEASE);
    }
}

/*
 * ProcessMailbox() — 读 SHM mailbox ,  PDO multi_ctrl 广播
 *
 * 支持双电机同时控制 (一次 multi_ctrl 发完 ID 1,2).
 * SPSC 环形缓冲: 算法写 slot[seq_write % DEPTH], RT 消费 seq_read 到 seq_write.
 */

/*
 * 辅助: 单轴 PDO 发送 (0x100+node), 模式切换时异步补发 (不阻塞 RT 线程)
 */
inline void _pdo_send_with_switch(motor_hal_t* hal, multi_axis_cmd_t* cmd,
                                   motor_mode_t* last_mode, motor_mode_t new_mode,
                                   uint8_t si, bool* pending_retry)
{
    bool mode_changed = (*last_mode != new_mode);
    *last_mode = new_mode;
    motor_hal_single_ctrl(hal, cmd);
    if (mode_changed) {
        pending_retry[si] = true;
    } else if (pending_retry[si]) {
        pending_retry[si] = false;
        motor_hal_single_ctrl(hal, cmd);
    }
}
void StarkRtWorker::ProcessMailbox()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm || !m_hal) return;

    uint64_t r = __atomic_load_n(&m_shm->mailbox.seq_read,  __ATOMIC_ACQUIRE);
    uint64_t w = __atomic_load_n(&m_shm->mailbox.seq_write, __ATOMIC_ACQUIRE);

    if (r >= w) return;  /* 无新数据 */

    /* 消费所有待处理帧 */
    uint64_t count = w - r;
    if (count > STARK_MBOX_DEPTH) count = STARK_MBOX_DEPTH;

    for (uint64_t n = 0; n < count; n++) {
        uint64_t idx = (r + n) % STARK_MBOX_DEPTH;
        motor_command_t* cmds = m_shm->mailbox.frames[idx].cmd;  /* cmd[STARK_MAX_MOTORS] */
        /* 下行段1终点: RT 读到 mailbox 帧的时刻 (仅 tracing 开启时打点) */
        if (m_trace_shm && __atomic_load_n(&m_trace_shm->enabled, __ATOMIC_ACQUIRE))
            m_mailbox_read_us = _rt_now_us();
        /* Byte0 管理命令 (改 pdo_byte0) */
        for (int i = 0; i < m_motor_count; i++) {
            motor_command_t c = cmds[i];
            uint8_t mid = c.motor_id;
            if (mid < 1) continue;

            switch (c.cmd) {
            case STARK_CMD_ENABLE:
                motor_hal_pdo_enable(m_hal, mid); break;
            case STARK_CMD_DISABLE:
                motor_hal_pdo_disable(m_hal, mid);
                { multi_axis_cmd_t mcmd = {}; mcmd.node_id = mid;
                  mcmd.enable = false; mcmd.release_brake = true;
                  mcmd.mode = MOTOR_MODE_CURRENT; mcmd.target1 = 0;
                  motor_hal_multi_ctrl(m_hal, &mcmd, 1); }
                break;
            case STARK_CMD_ESTOP:
                motor_hal_pdo_estop(m_hal, mid);
                { multi_axis_cmd_t mcmd = {}; mcmd.node_id = mid;
                  mcmd.enable = false; mcmd.release_brake = false;
                  mcmd.mode = MOTOR_MODE_CURRENT; mcmd.target1 = 0;
                  motor_hal_multi_ctrl(m_hal, &mcmd, 1); }
                break;
            case STARK_CMD_RECOVER:
                motor_hal_pdo_recover(m_hal, mid); break;
            case STARK_CMD_SET_MODE:
                motor_hal_pdo_set_mode(m_hal, mid, (motor_mode_t)c.value); break;
            case STARK_CMD_CLEAR_FAULT:
                motor_hal_pdo_clear_fault(m_hal, mid);
                motor_hal_pdo_enable(m_hal, mid);
                motor_hal_ctrl_raw(m_hal, mid, MOTOR_MODE_CURRENT, 0, 0, 0);
                break;
            default: break;
            }
        }

        /* 是否存在 MULTI 广播命令 */
        bool any_multi = false;
        for (int i = 0; i < m_motor_count; i++) {
            if (cmds[i].cmd == STARK_CMD_MULTI) { any_multi = true; break; }
        }

        /* MIT 多轴: 双电机用 0x210 一帧同时控制 (仅 2 电机时) */
        if (m_motor_count == 2 &&
            cmds[0].cmd == STARK_CMD_MIT_MULTI && cmds[1].cmd == STARK_CMD_MIT_MULTI) {
            motor_hal_mit_multi_ctrl_phys(m_hal,
                cmds[0].motor_id,
                (float)cmds[0].mit_pos * (360.0f / 65535.0f) - 180.0f,
                (float)((int16_t)(cmds[0].mit_vel << 4)) / 16.0f,
                (float)cmds[0].mit_kp / 100.0f,
                (float)cmds[0].mit_kd / 100.0f,
                (float)((int16_t)(cmds[0].mit_torque << 4)) / 16.0f,
                cmds[1].motor_id,
                (float)cmds[1].mit_pos * (360.0f / 65535.0f) - 180.0f,
                (float)((int16_t)(cmds[1].mit_vel << 4)) / 16.0f,
                (float)cmds[1].mit_kp / 100.0f,
                (float)cmds[1].mit_kd / 100.0f,
                (float)((int16_t)(cmds[1].mit_torque << 4)) / 16.0f);
            uint64_t origin = (cmds[0].timestamp_us > 0 &&
                               (cmds[1].timestamp_us == 0 || cmds[0].timestamp_us <= cmds[1].timestamp_us))
                                  ? cmds[0].timestamp_us : cmds[1].timestamp_us;
            PublishCtrlSample(TRACE_KIND_CTRL_MULTI, 0, origin);
        }
        /* 控制命令 (通过 pdo_byte0 发 PDO) */
        else if (any_multi) {
            multi_axis_cmd_t mcmds[STARK_MAX_MOTORS] = {};
            int mcount = 0;

            for (int i = 0; i < m_motor_count; i++) {
                motor_command_t c = cmds[i];
                if (c.cmd != STARK_CMD_MULTI) continue;
                if (c.motor_id < 1) continue;

                motor_mode_t m = (motor_mode_t)(c.multi_mode & 0x0F);
                if ((uint8_t)m == 0 || m == MOTOR_MODE_MIT) {
                    RT_LOG(RT_LOG_WARN, MULTI_INVALID, (uint32_t)c.motor_id, (uint32_t)m, 0, 0);
                    continue;
                }

                mcmds[mcount].node_id       = c.motor_id;
                mcmds[mcount].mode          = m;
                mcmds[mcount].enable        = true;
                mcmds[mcount].release_brake = true;
                mcmds[mcount].target1       = (int16_t)c.value;
                mcmds[mcount].target2       = (uint16_t)c.value2;
                mcmds[mcount].feedforward   = (int16_t)c.feedforward;
                mcount++;
            }
            if (mcount > 0) {
                motor_hal_multi_ctrl(m_hal, mcmds, (uint8_t)mcount);
                uint64_t origin = 0;
                for (int i = 0; i < m_motor_count; i++) {
                    uint64_t t = cmds[i].timestamp_us;
                    if (t > 0 && (origin == 0 || t < origin)) origin = t;
                }
                PublishCtrlSample(TRACE_KIND_CTRL_MULTI, 0, origin);
            }
        } else {
            for (int i = 0; i < m_motor_count; i++) {
                motor_command_t c = cmds[i];
                uint8_t mid = c.motor_id;
                if (mid < 1 || mid > (uint8_t)m_motor_count) continue;

                switch (c.cmd) {
                case STARK_CMD_TORQUE_CTRL:
                    {
                        int16_t torque_target = (int16_t)c.value;
                        uint8_t si = mid - 1;
                        multi_axis_cmd_t mcmd = {};
                        mcmd.node_id       = mid;
                        mcmd.mode          = MOTOR_MODE_TORQUE;
                        mcmd.enable        = true;
                        mcmd.release_brake = true;
                        mcmd.target1       = torque_target;
                        _pdo_send_with_switch(m_hal, &mcmd, &m_last_pdo_mode[si], MOTOR_MODE_TORQUE, si, m_pending_retry);
                        PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    }
                    break;
                case STARK_CMD_TORQUE:
                    {
                        int32_t ma = c.value;
                        uint8_t si = mid - 1;
                        multi_axis_cmd_t mcmd = {};
                        mcmd.node_id       = mid;
                        mcmd.mode          = MOTOR_MODE_CURRENT;
                        mcmd.enable        = true;
                        mcmd.release_brake = true;
                        mcmd.target1       = (int16_t)ma;
                        _pdo_send_with_switch(m_hal, &mcmd, &m_last_pdo_mode[si], MOTOR_MODE_CURRENT, si, m_pending_retry);
                        PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    }
                    break;
                case STARK_CMD_SPEED:
                case STARK_CMD_CSV:
                    {
                        int32_t rpm = c.value / 100;
                        uint8_t si = mid - 1;
                        multi_axis_cmd_t mcmd = {};
                        mcmd.node_id       = mid;
                        mcmd.mode          = MOTOR_MODE_CSV;
                        mcmd.enable        = true;
                        mcmd.release_brake = true;
                        mcmd.target1       = (int16_t)rpm;
                        _pdo_send_with_switch(m_hal, &mcmd, &m_last_pdo_mode[si], MOTOR_MODE_CSV, si, m_pending_retry);
                        PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    }
                    break;
                case STARK_CMD_PV:
                    {
                        int32_t rpm = c.value / 100;
                        uint16_t accel = (c.value2 > 0) ? (uint16_t)(c.value2 / 100) : (uint16_t)500;
                        uint8_t si = mid - 1;
                        multi_axis_cmd_t mcmd = {};
                        mcmd.node_id       = mid;
                        mcmd.mode          = MOTOR_MODE_PROFILE_VEL;
                        mcmd.enable        = true;
                        mcmd.release_brake = true;
                        mcmd.target1       = (int16_t)rpm;
                        mcmd.target2       = accel;
                        _pdo_send_with_switch(m_hal, &mcmd, &m_last_pdo_mode[si], MOTOR_MODE_PROFILE_VEL, si, m_pending_retry);
                        PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    }
                    break;
                case STARK_CMD_POS:
                    {
                        float deg = (float)c.value / 100.0f;
                        int32_t cnt = motor_deg_to_counts(deg);
                        uint8_t si = mid - 1;
                        multi_axis_cmd_t mcmd = {};
                        mcmd.node_id       = mid;
                        mcmd.mode          = MOTOR_MODE_CSP;
                        mcmd.enable        = true;
                        mcmd.release_brake = true;
                        mcmd.target1       = (int16_t)cnt;
                        _pdo_send_with_switch(m_hal, &mcmd, &m_last_pdo_mode[si], MOTOR_MODE_CSP, si, m_pending_retry);
                        PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    }
                    break;
                case STARK_CMD_PP:
                    {
                        float deg = (float)c.value / 100.0f;
                        int32_t cnt = motor_deg_to_counts(deg);
                        uint16_t accel = (c.value2 > 0) ? (uint16_t)(c.value2 / 100) : (uint16_t)500;
                        int16_t vel = (int16_t)(c.feedforward / 100);
                        uint8_t si = mid - 1;
                        multi_axis_cmd_t mcmd = {};
                        mcmd.node_id       = mid;
                        mcmd.mode          = MOTOR_MODE_PROFILE_POS;
                        mcmd.enable        = true;
                        mcmd.release_brake = true;
                        mcmd.target1       = (int16_t)cnt;
                        mcmd.target2       = accel;
                        mcmd.feedforward   = vel;
                        _pdo_send_with_switch(m_hal, &mcmd, &m_last_pdo_mode[si], MOTOR_MODE_PROFILE_POS, si, m_pending_retry);
                        PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    }
                    break;
                case STARK_CMD_MIT:
                case STARK_CMD_MIT_MULTI:
                    /* decode SHM raw → float → motor_hal V2 编码 */
                    motor_hal_mit_control(m_hal, mid,
                        (float)c.mit_pos * (360.0f / 65535.0f) - 180.0f,
                        (float)((int16_t)(c.mit_vel << 4)) / 16.0f,
                        (float)c.mit_kp / 100.0f,
                        (float)c.mit_kd / 100.0f,
                        (float)((int16_t)(c.mit_torque << 4)) / 16.0f);
                    PublishCtrlSample(TRACE_KIND_CTRL_SINGLE, mid, c.timestamp_us);
                    break;
                /* SDO 命令: 转发到 sdo_cmds, 主循环处理 */
                case STARK_CMD_SDO_CUR:
                case STARK_CMD_SDO_POS:
                case STARK_CMD_SDO_VEL:
                case STARK_CMD_SDO_TORQUE_CALIB:
                case STARK_CMD_SDO_MIT_MIGRATE:
                    {
                        int si = (int)(mid - 1);
                        if (si >= 0 && si < STARK_MAX_MOTORS) {
                            m_shm->sdo_cmds[si].motor_id    = mid;
                            m_shm->sdo_cmds[si].cmd         = c.cmd;
                            m_shm->sdo_cmds[si].value       = c.value;
                            m_shm->sdo_cmds[si].value2      = c.value2;
                            m_shm->sdo_cmds[si].feedforward = c.feedforward;
                            __atomic_thread_fence(__ATOMIC_RELEASE);
                            __atomic_add_fetch(&m_shm->sdo_seq[si], 1, __ATOMIC_RELEASE);
                        }
                    }
                    break;
                default: break;
                }
            }
        }
    }

    /* 确认消费 */
    __atomic_store_n(&m_shm->mailbox.seq_read, r + count, __ATOMIC_RELEASE);
}

/*
 * PublishFeedback() — 读 fb_cache + 组装 feedback_frame_t ,  SHM
 *
 * 频率: 每 m_report_divider (5) 个周期 = 200Hz
 */

void StarkRtWorker::PublishFeedback()
{
    /* 读 IMU 一次, feedback_frame_t 和 PeriodicUploadData 共用 */
    imu_data_t imu_local;
    bool imu_valid = false;
    if (m_imu_sensor && m_imu_sensor->IsReady()) {
        m_imu_sensor->Read(&imu_local);
        imu_valid = true;
    }

    /* 读气压计一次, feedback_frame_t 和 PeriodicUploadData 共用 */
    barometer_data_t baro_local;
    bool baro_valid = false;
    if (m_baro_sensor && m_baro_sensor->IsReady()) {
        m_baro_sensor->Read(&baro_local);
        baro_valid = true;
    }

    /* 周期上报: 基于 RT 周期计数器, 不嵌套在 feedback_frame_t 分频内 */
    if (m_report_enabled && m_shm && m_hal) {
        uint64_t elapsed = m_cycle_count - m_periodic_last_cycle;
        if (elapsed >= m_report_period_ms) {
            m_periodic_last_cycle = m_cycle_count;

            PeriodicUploadData d;
            memset(&d, 0, sizeof(d));

            if (imu_valid) {
                d.gyro_dps_x  = imu_local.gyro_x;
                d.gyro_dps_y  = imu_local.gyro_y;
                d.gyro_dps_z  = imu_local.gyro_z;
                d.quat_w      = imu_local.quat_w;
                d.quat_x      = imu_local.quat_x;
                d.quat_y      = imu_local.quat_y;
                d.quat_z      = imu_local.quat_z;
                d.gyro_roll   = imu_local.roll;
                d.gyro_pitch  = imu_local.pitch;
                d.gyro_yaw    = imu_local.yaw;
                d.acc_x       = imu_local.acc_x;
                d.acc_y       = imu_local.acc_y;
                d.acc_z       = imu_local.acc_z;
            }
            if (baro_valid) {
                d.air_pressure = baro_local.pressure_hpa;
            }

            /* 双电机 */
            uint32_t motor_ts_min = 0xFFFFFFFF;
            uint32_t sensor_ts_min = 0xFFFFFFFF;

            for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
                motor_feedback_t mfb;
                motor_sensor_t s;
                bool is_right = (id == 1);

                if (m_data_source == DS_UNIFIED_6C0) {
                    /* 统一6C0: status+torque_fb 走0x300, 其余全部走0x6C0 */
                    int16_t mstate = 0;
                    int16_t torque_fb = 0;
                    uint32_t motor_ts = 0;
                    if (motor_hal_get_feedback(m_hal, id, &mfb) == 0) {
                        mstate    = (int16_t)mfb.status_byte;
                        torque_fb = mfb.torque_nm;
                        motor_ts  = (uint32_t)(mfb.timestamp_us & 0xFFFFFFFF);
                    }
                    if (motor_ts < motor_ts_min) motor_ts_min = motor_ts;

                    if (motor_hal_get_sensor(m_hal, id, &s) == 0) {
                        uint32_t sts = (uint32_t)(s.timestamp_us & 0xFFFFFFFF);
                        if (sts < sensor_ts_min) sensor_ts_min = sts;

                        int32_t vel_x10   = s.motor_vel_raw;
                        int16_t iq_x100   = (int16_t)(s.iq_current);
                        int16_t fcode     = (int16_t)s.error_code;
                        int32_t tmp_x100  = (int32_t)s.motor_temp_x10 * 10;
                        int16_t ang_x10   = (int16_t)(s.motor_pos_raw * 3600 / 65536);
                        int16_t bus_x100  = (int16_t)(s.bus_current / 10);

                        if (is_right) {
                            d.RealtimeVelocity = vel_x10;
                            d.motor_abs_angle  = ang_x10;
                            d.cal_Iq_current   = iq_x100;
                            d.motor_temp       = tmp_x100;
                            d.cal_bus_current  = bus_x100;
                            d.fault_code       = fcode;
                            d.motor_state      = mstate;
                            d.torque_feedback  = torque_fb;
                            d.df181_torque     = s.force_raw;
                            d.hall_a_data      = s.hall_adc0;
                            d.hall_b_data      = s.hall_adc1;
                            d.hall_c_data      = s.hall_adc2;
                            d.knee_hall        = (int16_t)s.knee_hall;
                            d.key_landing      = s.hw_sw_pc9;
                            d.torque_valid     = s.data_valid;
                            d.spi_torque       = (float)s.spi_force_raw_s24 / 100.0f;
                            d.spi_valid        = s.spi_valid;
                            d.spi_error        = s.spi_error;
                        } else {
                            d.RealtimeVelocity_left = vel_x10;
                            d.motor_abs_angle_left  = ang_x10;
                            d.cal_Iq_current_left   = iq_x100;
                            d.motor_temp_left       = tmp_x100;
                            d.cal_bus_current_left  = bus_x100;
                            d.fault_code_left       = fcode;
                            d.motor_state_left      = mstate;
                            d.torque_feedback_left  = torque_fb;
                            d.df181_torque_left     = s.force_raw;
                            d.hall_a_data_left      = s.hall_adc0;
                            d.hall_b_data_left      = s.hall_adc1;
                            d.hall_c_data_left      = s.hall_adc2;
                            d.knee_hall_left        = (int16_t)s.knee_hall;
                            d.key_landing_left      = s.hw_sw_pc9;
                            d.torque_valid_left     = s.data_valid;
                            d.spi_torque_left       = (float)s.spi_force_raw_s24 / 100.0f;
                            d.spi_valid_left        = s.spi_valid;
                            d.spi_error_left        = s.spi_error;
                        }
                    }
                } else {
                    /* mixed: 原有混合来源逻辑, 保持不变 */

                    if (motor_hal_get_feedback(m_hal, id, &mfb) == 0) {
                        uint32_t ts = (uint32_t)(mfb.timestamp_us & 0xFFFFFFFF);
                        if (ts < motor_ts_min) motor_ts_min = ts;
               //         int32_t vel_x10  = (int32_t)mfb.velocity * 10;
               //         int16_t iq_x100  = (int16_t)(mfb.current_iq / 10);
                    int32_t vel_x10  = (int32_t)mfb.velocity;
                        int16_t iq_x100  = (int16_t)(mfb.current_iq);
           
    	   	    int16_t fcode    = (int16_t)mfb.error_code;
                        int16_t mstate   = (int16_t)mfb.status_byte;

                        /* 温度: 优先 0x6A0 透传帧, 回退 0x300 */
                        int32_t tmp_x100;
                        (void)motor_hal_get_sensor(m_hal, id, &s);
                        if (s.motor_temp_x10 != 0)
                            tmp_x100 = (int32_t)s.motor_temp_x10 * 10;
                        else
                            tmp_x100 = (int32_t)mfb.temperature * 10;

                        int32_t sdo_val = 0;
                        int16_t ang_x10;
                        if (motor_hal_get_sdo_position(m_hal, id, &sdo_val) == 0)
                            ang_x10 = (int16_t)(sdo_val * 3600 / 65536);
                        else
                            ang_x10 = (int16_t)((int32_t)mfb.position * 3600 / 65536);

                        if (is_right) {
                            d.RealtimeVelocity = vel_x10;
                            d.motor_abs_angle  = ang_x10;
                            d.cal_Iq_current   = iq_x100;
                            d.motor_temp       = tmp_x100;
                            d.cal_bus_current   = (int16_t)(s.bus_current / 10);
                            d.fault_code       = fcode;
                            d.motor_state      = mstate;
                        } else {
                            d.RealtimeVelocity_left = vel_x10;
                            d.motor_abs_angle_left  = ang_x10;
                            d.cal_Iq_current_left   = iq_x100;
                            d.motor_temp_left       = tmp_x100;
                            d.cal_bus_current_left   = (int16_t)(s.bus_current / 10);
                            d.fault_code_left       = fcode;
                            d.motor_state_left      = mstate;
                        }
                    }

                    if (motor_hal_get_sensor(m_hal, id, &s) == 0) {
                        uint32_t sts = (uint32_t)(s.timestamp_us & 0xFFFFFFFF);
                        if (sts < sensor_ts_min) sensor_ts_min = sts;
                        if (is_right) {
                            d.hall_a_data  = s.hall_adc0;
                            d.hall_b_data  = s.hall_adc1;
                            d.hall_c_data  = s.hall_adc2;
                            d.knee_hall   = (int16_t)s.knee_hall;
                            d.key_landing  = s.hw_sw_pc9;
                            d.torque_valid = s.data_valid;
                            d.df181_torque = s.force_raw;
                            d.torque_feedback = mfb.torque_nm;
                        } else {
                            d.hall_a_data_left  = s.hall_adc0;
                            d.hall_b_data_left  = s.hall_adc1;
                            d.hall_c_data_left  = s.hall_adc2;
                            d.knee_hall_left   = (int16_t)s.knee_hall;
                            d.key_landing_left  = s.hw_sw_pc9;
                            d.torque_valid_left = s.data_valid;
                            d.df181_torque_left = s.force_raw;
                            d.torque_feedback_left = mfb.torque_nm;
                        }
                        /* 0x6B0 力矩并入 PeriodicUploadData (单一上报路径) */
                        if (is_right) {
                            d.spi_torque        = (float)s.spi_force_raw_s24 / 100.0f;
                            d.spi_valid         = s.spi_valid;
                            d.spi_error         = s.spi_error;
                        } else {
                            d.spi_torque_left        = (float)s.spi_force_raw_s24 / 100.0f;
                            d.spi_valid_left         = s.spi_valid;
                            d.spi_error_left         = s.spi_error;
                        }
                    }
                }  /* end if data_source */
            }
            struct timespec now_ts;
            clock_gettime(CLOCK_REALTIME, &now_ts);
            d.timestamp_ms  = (uint32_t)(now_ts.tv_sec * 1000ULL +
                                          now_ts.tv_nsec / 1000000ULL);
            d.frame_cycle   = (uint32_t)m_cycle_count;
            d.motor_ts_us   = (motor_ts_min != 0xFFFFFFFF) ? motor_ts_min : 0;
            d.imu_ts_us     = (uint32_t)(imu_local.timestamp_us & 0xFFFFFFFF);
            d.sensor_ts_us  = (sensor_ts_min != 0xFFFFFFFF) ? sensor_ts_min : 0;

            /* 足底压力 */
            if (m_foot_sensor && m_foot_sensor->IsReady()) {
                m_foot_sensor->Read(&d.foot_pressure);
                d.foot_pressure.update_cycle = (uint32_t)m_cycle_count;
            }

            memcpy(&m_shm->periodic_data, &d, sizeof(d));
            __atomic_add_fetch(&m_shm->periodic_version, 1, __ATOMIC_RELEASE);
            /* 唤醒阻塞在 stark_report_wait 的算法侧接收线程. 共享 futex,
             * 无等待者时内核直接返回, 5ms 一次开销可忽略, 不影响 RT 时序. */
            syscall(SYS_futex, &m_shm->periodic_version, FUTEX_WAKE,
                    INT_MAX, NULL, NULL, 0);
        }
    }

    if (--m_report_divider > 0) return;
    m_report_divider = m_rt.report_divider;

    if (!m_hal || !m_shm) return;
    if (!m_active.load(std::memory_order_acquire)) return;

    uint32_t active    = __atomic_load_n(&m_shm->active_idx, __ATOMIC_ACQUIRE);
    uint32_t write_idx = active ^ 1;

    feedback_frame_t* fb = &m_shm->fb_buffer[write_idx];
    memset(fb, 0, sizeof(feedback_frame_t));

    struct timespec ts;

    uint64_t read_rt_us = 0;
    if (m_rt.perf_trace) {
        struct timespec ts_rt;
        clock_gettime(CLOCK_MONOTONIC, &ts_rt);
        read_rt_us = (uint64_t)ts_rt.tv_sec * 1000000ULL + (uint64_t)ts_rt.tv_nsec / 1000ULL;
    }
    uint64_t min_fb_ts = UINT64_MAX;

    /* 填充电机反馈 (从 HAL 反馈缓存) */
    for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
        motor_feedback_t mfb;
        if (motor_hal_get_feedback(m_hal, id, &mfb) == 0) {
            if (m_rt.perf_trace && mfb.timestamp_us > 0 && mfb.timestamp_us < read_rt_us && (read_rt_us - mfb.timestamp_us) < 10000 && mfb.timestamp_us < min_fb_ts)
                min_fb_ts = mfb.timestamp_us;
            uint8_t idx = id - 1;
            fb->motor[idx].position    = mfb.position;
            fb->motor[idx].velocity    = mfb.velocity;
            fb->motor[idx].current_iq  = mfb.current_iq;
            fb->motor[idx].temperature = mfb.temperature;
            fb->motor[idx].status_byte = mfb.status_byte;
            fb->motor[idx].mode        = mfb.mode;
            fb->motor[idx].error_code  = mfb.error_code;
        }
    }

    /* 填充传感器透传 */
    for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
        motor_sensor_t s;
        int ret = motor_hal_get_sensor(m_hal, id, &s);
        if (ret == 0) {
            uint8_t idx = id - 1;
            fb->sensor[idx].hall_adc0   = s.hall_adc0;
            fb->sensor[idx].hall_adc1   = s.hall_adc1;
            fb->sensor[idx].hall_adc2   = s.hall_adc2;
            fb->sensor[idx].force_raw   = s.force_raw;
            fb->sensor[idx].knee_hall    = s.knee_hall;
            fb->sensor[idx].key_landing = s.hw_sw_pc9;
            fb->sensor[idx].data_valid  = s.data_valid;
        } else if (!m_sensor_notified[id - 1]) {
            RT_LOG(RT_LOG_WARN, SENSOR_NOCFG, (uint32_t)id, (uint32_t)ret, 0, 0);
            m_sensor_notified[id - 1] = true;
        }
    }

    /* 填充 IMU 融合数据 (非阻塞, 硬件未就绪时全零) */
    if (imu_valid) {
        fb->imu = imu_local;
    } else {
        memset(&fb->imu, 0, sizeof(fb->imu));
    }

    /* 气压计: 读传感器缓存, 每周期写入 SHM */
    if (baro_valid) {
        fb->baro = baro_local;
    } else {
        memset(&fb->baro, 0, sizeof(fb->baro));
    }

    /* 足底压力: 读传感器缓存, 每周期写入 SHM */
    if (m_foot_sensor && m_foot_sensor->IsReady()) {
        foot_pressure_data_t fp;
        m_foot_sensor->Read(&fp);
        fp.update_cycle = (uint32_t)m_cycle_count;
        fb->foot_pressure = fp;
        fb->ts_foot_rx = fp.timestamp_us;
    }

    /* T4: SHM 切换 */
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fb->ts_shm_write      = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    fb->ts_frame_assembly = fb->ts_shm_write;
    fb->timestamp_us      = fb->ts_shm_write;

    /* 记录 CAN 收帧时刻 (供 demo_algo 计算反馈 e2e = algo读时刻 - CAN收帧) */
    if (min_fb_ts != UINT64_MAX) {
        fb->ts_can_rx = min_fb_ts;
    }

    /* 上行分段统计: 段1 = rt读 - can0收帧 (含协议解析+排队).
     * read_rt_us 仅在 perf_trace 时打点, 与 trace 开关一致. */
    fb->ts_shm_read = read_rt_us;
    if (m_trace_shm && __atomic_load_n(&m_trace_shm->enabled, __ATOMIC_ACQUIRE) &&
        min_fb_ts != UINT64_MAX && read_rt_us >= min_fb_ts) {
        uint64_t up_seg1 = read_rt_us - min_fb_ts;
        if (up_seg1 < 100000) {
            trace_stat_update(&m_trace_shm->up_seg1, up_seg1);
        }
    }

    /* 切换活跃 Buffer */
    __atomic_store_n(&m_shm->active_idx, write_idx, __ATOMIC_RELEASE);

    /* 周期超限次数 (前端状态显示) */
    if (m_rt.perf_trace) {
        m_shm->cycle_overrun_count = (uint32_t)(m_overrun_count & 0xFFFFFFFFULL);
    }
}

/*
 * PublishCtrlSample() — 逐帧写一条控制样本到 ctrl ring (单写多读无锁)
 *
 * kind: TRACE_KIND_CTRL_SINGLE(单电机命令) / TRACE_KIND_CTRL_MULTI(多轴广播)
 * origin_us: 算法下发时刻 (motor_command_t.timestamp_us), 0=无有效起点
 * e2e_us = 发送完成时刻(now) - 算法下发时刻
 *
 * 写入顺序: 先填 ctrl_samples[head % RING], 再 release 递增 ctrl_head.
 * 此时 m_cycle_count 尚未递增, 样本 cycle = 帧发生的当前周期 (对齐修复).
 */
void StarkRtWorker::PublishCtrlSample(uint16_t kind, uint16_t motor_id, uint64_t origin_us)
{
    if (!m_trace_shm) return;
    if (!__atomic_load_n(&m_trace_shm->enabled, __ATOMIC_ACQUIRE)) return;
    if (origin_us == 0) return;

    uint64_t now = _rt_now_us();   /* can0 发送完成时刻 (下行段2终点) */
    if (now < origin_us) return;
    uint64_t e2e = now - origin_us;   /* 下行总 */

    /* 下行分段统计: 段1=algo→rt读mailbox, 段2=rt读→can0发送.
     * 校验时间戳单调性, 过滤跨周期错位导致的虚假耗时. */
    if (m_mailbox_read_us >= origin_us && now >= m_mailbox_read_us) {
        uint64_t seg1 = m_mailbox_read_us - origin_us;
        uint64_t seg2 = now - m_mailbox_read_us;
        if (seg1 < 100000 && seg2 < 100000 && e2e < 100000) {
            trace_stat_update(&m_trace_shm->dn_seg1, seg1);
            trace_stat_update(&m_trace_shm->dn_seg2, seg2);
            trace_stat_update(&m_trace_shm->dn_total, e2e);
        }
    }

    uint32_t head = __atomic_load_n(&m_trace_shm->ctrl_head, __ATOMIC_RELAXED);
    trace_sample_t* s = &m_trace_shm->ctrl_samples[head % STARK_TRACE_CTRL_RING];
    s->cycle    = (uint32_t)(m_cycle_count & 0xFFFFFFFF);
    s->kind     = kind;
    s->e2e_us   = (uint16_t)(e2e > 65535 ? 65535 : e2e);
    s->motor_id = motor_id;
    s->reserved = 0;
    s->ts_us    = (uint32_t)(origin_us & 0xFFFFFFFF);
    __atomic_store_n(&m_trace_shm->ctrl_head, head + 1, __ATOMIC_RELEASE);
}

/*
 * PublishTrace() — 每周期写一条抖动样本到 jitter ring (单写多读无锁)
 *
 * jitter 是"本周期睡眠后的实际唤醒 - 理想目标", 属性上属于下一周期,
 * 因此用已递增后的 m_cycle_count 作为周期标签.
 */
void StarkRtWorker::PublishTrace()
{
    if (!m_trace_shm) return;
    if (!__atomic_load_n(&m_trace_shm->enabled, __ATOMIC_ACQUIRE)) return;

    uint32_t head = __atomic_load_n(&m_trace_shm->jitter_head, __ATOMIC_RELAXED);
    trace_sample_t* s = &m_trace_shm->jitter_samples[head % STARK_TRACE_JITTER_RING];
    s->cycle    = (uint32_t)(m_cycle_count & 0xFFFFFFFF);
    s->kind     = 0;
    s->e2e_us   = (uint16_t)(m_cur_jitter > 65535 ? 65535 : m_cur_jitter);
    s->motor_id = 0;
    s->reserved = 0;
    s->ts_us    = (uint32_t)(_rt_now_us() & 0xFFFFFFFF);
    __atomic_store_n(&m_trace_shm->jitter_head, head + 1, __ATOMIC_RELEASE);
}

/*
 * 安全监控 (SafetyCheck) 与算法心跳握手/超时脱使能逻辑已移除.
 * 电机使能状态完全由控制命令 (mailbox 控制帧 / mgmt 通道) 驱动,
 * 不再因算法侧静默 (无心跳) 而自动脱使能.
 * 如需重新引入过温/编码器停滞/CAN断线等硬件级保护, 应基于反馈帧
 * 独立实现, 不要复用算法心跳机制.
 */

/* SetThreadRt() — RT 线程属性 */

void StarkRtWorker::SetThreadRt()
{
    if (!m_rt.enable_rt) {
        prctl(PR_SET_NAME, "stark_nrt", 0, 0, 0);
        printf("[StarkRtWorker] Thread: SCHED_OTHER (non-RT mode, no affinity)\n");
        return;
    }

    prctl(PR_SET_NAME, "stark_rt", 0, 0, 0);

    struct sched_param param;
    param.sched_priority = m_rt.priority;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        RT_LOG(RT_LOG_ERROR, SCHED_FIFO_FAIL, 0, 0, 0, 0);
    }

    /* 锁定当前+未来全部内存页, 防止 RT 线程缺页中断 */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RT_LOG(RT_LOG_ERROR, MLOCKALL_FAIL, 0, 0, 0, 0);
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(m_rt.cpu_affinity[0], &cpuset);
    if (m_rt.cpu_affinity[1] >= 0) {
        CPU_SET(m_rt.cpu_affinity[1], &cpuset);
    }
    ret = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (ret != 0) {
        RT_LOG(RT_LOG_ERROR, CPU_AFFINITY_FAIL, 0, 0, 0, 0);
    }

    printf("[StarkRtWorker] Thread: SCHED_FIFO prio=%d period=%dus cpu=%d,%d\n",
           m_rt.priority, m_rt.period_us,
           m_rt.cpu_affinity[0], m_rt.cpu_affinity[1]);
}

/* GetPendingState() — 主循环读取 RT 线程请求的状态切换
 * atomic exchange: 读当前值并清零为 STATE_BOOTING
 */

stark_state_t StarkRtWorker::GetPendingState()
{
    return (stark_state_t)__atomic_exchange_n(
        &m_pending_state, (uint32_t)STATE_BOOTING, __ATOMIC_ACQUIRE);
}

}  /* namespace stark_periph_manager_node */
