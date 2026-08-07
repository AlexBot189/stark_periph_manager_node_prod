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
#include "utils/latency_trace.h"
#include "motor/motor_ctrl.h"
#include "imu/imu_sensor.h"
#include "foot_pressure/FootPressureSensor.h"
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

/* 构造 / 析构 */

StarkRtWorker::StarkRtWorker(motor_hal_t* hal, stark_shm_t* shm,
                         StarkMotorCtrl* ctrl, ImuHALSensor* imu_sensor,
                         FootPressureSensor* foot_sensor,
                         int motor_count)
    : m_hal(hal)
    , m_shm(shm)
    , m_ctrl(ctrl)
    , m_imu_sensor(imu_sensor)
    , m_foot_sensor(foot_sensor)
    , m_motor_count(motor_count)
    , m_can_last_frame_us(0)
    , m_latency_idx(0)
    , m_report_divider(m_rt.report_divider)
    , m_report_enabled(false)
    , m_report_period_ms(5)
    , m_periodic_last_cycle(0)
    , m_cycle_count(0)
    , m_overrun_count(0)
{
    memset(m_last_position, 0, sizeof(m_last_position));
    memset(m_pos_stall_us, 0, sizeof(m_pos_stall_us));
    memset(m_sensor_notified, 0, sizeof(m_sensor_notified));
    memset(m_latency_history, 0, sizeof(m_latency_history));
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
    }

    struct timespec next_wake;
    clock_gettime(CLOCK_MONOTONIC, &next_wake);

    const long period_ns = (long)m_rt.period_us * 1000L;

    while (m_running.load(std::memory_order_acquire)) {

        ProcessMgmt();
        ProcessMailbox();
        PublishFeedback();

        /* ⑤ 提交延迟样本 */
        m_tracer.commit_sample();

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
        if (ret != 0) {
            m_overrun_count++;
            clock_gettime(CLOCK_MONOTONIC, &next_wake);
        }
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
 * 辅助: PDO multi_ctrl 发送, 模式切换时异步补发 (不阻塞 RT 线程)
 */
inline void _pdo_send_with_switch(motor_hal_t* hal, multi_axis_cmd_t* cmd,
                                   motor_mode_t* last_mode, motor_mode_t new_mode,
                                   uint8_t si, bool* pending_retry)
{
    bool mode_changed = (*last_mode != new_mode);
    *last_mode = new_mode;
    motor_hal_multi_ctrl(hal, cmd, 1);
    if (mode_changed) {
        pending_retry[si] = true;
    } else if (pending_retry[si]) {
        pending_retry[si] = false;
        motor_hal_multi_ctrl(hal, cmd, 1);
    }
}
static int _stark_tx_dbg(void)
{
    static int e = -1;
    if (e < 0) {
        const char* v = getenv("STARK_TX_DEBUG");
        e = (v && v[0] && v[0] != '0') ? 1 : 0;
    }
    return e;
}

static const char* _stark_cmd_name(uint8_t cmd)
{
    switch (cmd) {
    case STARK_CMD_TORQUE:      return "TORQUE";
    case STARK_CMD_SPEED:       return "SPEED";
    case STARK_CMD_POS:         return "POS";
    case STARK_CMD_MIT:         return "MIT";
    case STARK_CMD_MIT_MULTI: return "MIT_MULTI";
    case STARK_CMD_PP:          return "PP";
    case STARK_CMD_CSV:         return "CSV";
    case STARK_CMD_MULTI:       return "MULTI";
    case STARK_CMD_PV:          return "PV";
    case STARK_CMD_ENABLE:      return "ENABLE";
    case STARK_CMD_DISABLE:     return "DISABLE";
    case STARK_CMD_ESTOP:       return "ESTOP";
    case STARK_CMD_RECOVER:     return "RECOVER";
    case STARK_CMD_SET_MODE:    return "SET_MODE";
    case STARK_CMD_CLEAR_FAULT: return "CLEAR_FAULT";
    case STARK_CMD_SDO_CUR:          return "SDO_CUR";
    case STARK_CMD_SDO_POS:          return "SDO_POS";
    case STARK_CMD_SDO_VEL:          return "SDO_VEL";
    case STARK_CMD_SDO_TORQUE_CALIB: return "SDO_TORQUE_CALIB";
    case STARK_CMD_SDO_MIT_MIGRATE:  return "SDO_MIT_MIGRATE";
    default:                          return "?";
    }
}

static void _stark_dbg_tx(const motor_command_t& c)
{
    if (c.motor_id < 1) return;
    if (c.cmd == STARK_CMD_MIT) {
        fprintf(stderr, "[TX] id=%u %s pos=%u vel=%d kp=%u kd=%u tau=%d\n",
                c.motor_id, _stark_cmd_name(c.cmd),
                c.mit_pos, c.mit_vel, c.mit_kp, c.mit_kd, c.mit_torque);
    } else {
        fprintf(stderr, "[TX] id=%u %s value=%d value2=%d ff=%d\n",
                c.motor_id, _stark_cmd_name(c.cmd),
                c.value, c.value2, c.feedforward);
    }
}

void StarkRtWorker::ProcessMailbox()
{
    if (!m_active.load(std::memory_order_acquire)) return;
    if (!m_shm || !m_hal) return;

    uint64_t r = __atomic_load_n(&m_shm->mailbox.seq_read,  __ATOMIC_ACQUIRE);
    uint64_t w = __atomic_load_n(&m_shm->mailbox.seq_write, __ATOMIC_ACQUIRE);

    if (r >= w) return;  /* 无新数据 */

    m_tracer.mark_mailbox_read();

#if STARK_LATENCY_TRACE
    struct timespec ts_mbox;
    clock_gettime(CLOCK_MONOTONIC, &ts_mbox);
    uint64_t mbox_read_us = (uint64_t)ts_mbox.tv_sec * 1000000ULL + (uint64_t)ts_mbox.tv_nsec / 1000ULL;
#endif

    /* 消费所有待处理帧 */
    uint64_t count = w - r;
    if (count > STARK_MBOX_DEPTH) count = STARK_MBOX_DEPTH;

    for (uint64_t n = 0; n < count; n++) {
        uint64_t idx = (r + n) % STARK_MBOX_DEPTH;
        motor_command_t cmd0 = m_shm->mailbox.frames[idx].cmd[0];
        motor_command_t cmd1 = m_shm->mailbox.frames[idx].cmd[1];
        if (_stark_tx_dbg()) { _stark_dbg_tx(cmd0); _stark_dbg_tx(cmd1); }

        /* Byte0 管理命令 (改 pdo_byte0) */
        for (int i = 0; i < m_motor_count; i++) {
            motor_command_t c = (i == 0) ? cmd0 : cmd1;
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

        /* MIT 多轴: 双电机用 0x210 一帧同时控制 */
        if (cmd0.cmd == STARK_CMD_MIT_MULTI && cmd1.cmd == STARK_CMD_MIT_MULTI) {
            motor_hal_mit_multi_ctrl_phys(m_hal,
                cmd0.motor_id,
                (float)cmd0.mit_pos * (360.0f / 65535.0f) - 180.0f,
                (float)((int16_t)(cmd0.mit_vel << 4)) / 16.0f,
                (float)cmd0.mit_kp / 100.0f,
                (float)cmd0.mit_kd / 100.0f,
                (float)((int16_t)(cmd0.mit_torque << 4)) / 16.0f,
                cmd1.motor_id,
                (float)cmd1.mit_pos * (360.0f / 65535.0f) - 180.0f,
                (float)((int16_t)(cmd1.mit_vel << 4)) / 16.0f,
                (float)cmd1.mit_kp / 100.0f,
                (float)cmd1.mit_kd / 100.0f,
                (float)((int16_t)(cmd1.mit_torque << 4)) / 16.0f);
        }
        /* 控制命令 (通过 pdo_byte0 发 PDO) */
        else if (cmd0.cmd == STARK_CMD_MULTI || cmd1.cmd == STARK_CMD_MULTI) {
            multi_axis_cmd_t mcmds[STARK_MAX_MOTORS] = {};
            int mcount = 0;

            for (int i = 0; i < 2; i++) {
                motor_command_t c = (i == 0) ? cmd0 : cmd1;
                if (c.cmd != STARK_CMD_MULTI) continue;
                if (c.motor_id < 1) continue;

                motor_mode_t m = (motor_mode_t)(c.multi_mode & 0x0F);
                if ((uint8_t)m == 0 || m == MOTOR_MODE_MIT) {
                    ECO_WARN_NEW("[MULTI] M%d multi_mode=%d invalid, skip", c.motor_id, (int)m);
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
            if (mcount > 0) motor_hal_multi_ctrl(m_hal, mcmds, (uint8_t)mcount);
        } else {
            for (int i = 0; i < m_motor_count; i++) {
                motor_command_t c = (i == 0) ? cmd0 : cmd1;
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
                        (float)((int16_t)(c.mit_torque << 4)) / 16.0f); break;
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

#if STARK_LATENCY_TRACE
    /* mbox_age: 算法写 mailbox → RT 读到 */
    if (count > 0) {
        uint64_t last_idx = (r + count - 1) % STARK_MBOX_DEPTH;
        uint64_t min_ts = UINT64_MAX;
        for (int i = 0; i < STARK_MAX_MOTORS; i++) {
            uint64_t ts = m_shm->mailbox.frames[last_idx].cmd[i].timestamp_us;
            if (ts > 0 && ts < min_ts) min_ts = ts;
        }
        uint32_t age = 0;
        if (min_ts != UINT64_MAX && mbox_read_us > min_ts)
            age = (uint32_t)(mbox_read_us - min_ts);

        static uint32_t s_mbox_max = 0;
        if (age > s_mbox_max) s_mbox_max = age;
        m_shm->mbox_age_max_us = (uint16_t)(s_mbox_max > 65535 ? 65535 : s_mbox_max);

        static uint32_t s_mbox_sum = 0, s_mbox_min_win = UINT32_MAX;
        static uint16_t s_mbox_cnt = 0;
        s_mbox_sum += age; s_mbox_cnt++;
        if (age < s_mbox_min_win) s_mbox_min_win = age;
        if (s_mbox_cnt >= 1000) {
            s_mbox_sum = 0; s_mbox_cnt = 0;
            s_mbox_min_win = UINT32_MAX;
        }
        m_shm->mbox_age_avg_us = (uint16_t)(s_mbox_cnt > 0 ? s_mbox_sum / s_mbox_cnt : 0);
        m_shm->mbox_age_min_us = (uint16_t)(s_mbox_min_win != UINT32_MAX ? s_mbox_min_win : 0);
    }
#endif

    m_tracer.mark_pdo_sent();
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

    /* T1: 开始读 fb_cache */
    m_tracer.mark_fb_read_start();

#if STARK_LATENCY_TRACE
    struct timeval tv_rt;
    gettimeofday(&tv_rt, NULL);
    uint64_t read_rt_us = tv_rt.tv_sec * 1000000ULL + tv_rt.tv_usec;
#endif
    uint64_t min_fb_ts = UINT64_MAX;

    /* 填充电机反馈 (从 HAL 反馈缓存) */
    for (uint8_t id = 1; id <= (uint8_t)m_motor_count; ++id) {
        motor_feedback_t mfb;
        if (motor_hal_get_feedback(m_hal, id, &mfb) == 0) {
#if STARK_LATENCY_TRACE
            if (mfb.timestamp_us > 0 && mfb.timestamp_us < read_rt_us && (read_rt_us - mfb.timestamp_us) < 10000 && mfb.timestamp_us < min_fb_ts)
                min_fb_ts = mfb.timestamp_us;
#endif
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

    /* T2: fb_cache 读取完成 */
    m_tracer.mark_fb_read_done();

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
            RT_LOG("sensor not configured for motor %d (ret=%d)", id, ret);
            m_sensor_notified[id - 1] = true;
        }
    }

    /* 填充 IMU 融合数据 (非阻塞, 硬件未就绪时全零) */
    if (imu_valid) {
        fb->imu = imu_local;
    } else {
        memset(&fb->imu, 0, sizeof(fb->imu));
    }

    /* 气压计: 硬件未接入, 保持全零 */
    memset(&fb->baro, 0, sizeof(fb->baro));

    /* 足底压力: 读传感器缓存, 每周期写入 SHM */
    if (m_foot_sensor && m_foot_sensor->IsReady()) {
        foot_pressure_data_t fp;
        m_foot_sensor->Read(&fp);
        fp.update_cycle = (uint32_t)m_cycle_count;
        fb->foot_pressure = fp;
        fb->ts_foot_rx = fp.timestamp_us;
    }

    /* IMU+传感器+气压计 组装完成 */
    m_tracer.mark_mock_sensor_done();

    /* T4: SHM 切换 */
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fb->ts_shm_write      = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    fb->ts_frame_assembly = fb->ts_shm_write;
    fb->timestamp_us      = fb->ts_shm_write;

    /* 切换活跃 Buffer */
    __atomic_store_n(&m_shm->active_idx, write_idx, __ATOMIC_RELEASE);

    /* T4: SHM 双 Buffer 切换完成 */
    m_tracer.mark_shm_write_done();

    /* 填充 SHM 耗时统计 (供 perf_test 读取) */
    {
        latency_stats_t st = {};
        m_tracer.fill_shm_stats(st);
        m_shm->fb_read_avg_us    = (uint16_t)st.fb_read_avg;
        m_shm->fb_read_max_us    = (uint16_t)st.fb_read_max;
        m_shm->fb_total_avg_us   = (uint16_t)st.fb_total_avg;
        m_shm->fb_total_max_us   = (uint16_t)st.fb_total_max;
        m_shm->fb_total_min_us   = (uint16_t)st.fb_total_min;
        m_shm->ctrl_total_avg_us = (uint16_t)st.ctrl_total_avg;
        m_shm->ctrl_total_min_us = (uint16_t)st.ctrl_total_min;
        {
            static uint32_t s_ctrl_max = 0;
            if (st.ctrl_total_max > s_ctrl_max) s_ctrl_max = st.ctrl_total_max;
            m_shm->ctrl_total_max_us = (uint16_t)(s_ctrl_max > 65535 ? 65535 : s_ctrl_max);
        }

#if STARK_LATENCY_TRACE
        /* 反馈数据年龄 (CAN 收帧 → RT worker 读到) */
        {
            uint32_t fb_age = 0;
            if (min_fb_ts != UINT64_MAX && read_rt_us > min_fb_ts)
                fb_age = (uint32_t)(read_rt_us - min_fb_ts);

            static uint32_t s_age_max = 0;
            if (fb_age > s_age_max) s_age_max = fb_age;
            m_shm->fb_age_max_us = (uint16_t)(s_age_max > 65535 ? 65535 : s_age_max);

            static uint32_t s_age_sum = 0, s_age_min_win = UINT32_MAX;
            static uint16_t s_age_cnt = 0;
            s_age_sum += fb_age;
            s_age_cnt++;
            if (fb_age < s_age_min_win) s_age_min_win = fb_age;
            if (s_age_cnt >= 1000) {
                s_age_sum = 0;
                s_age_cnt = 0;
                s_age_min_win = UINT32_MAX;
            }
            m_shm->fb_age_avg_us = (uint16_t)(s_age_cnt > 0 ? s_age_sum / s_age_cnt : 0);
            m_shm->fb_age_min_us = (uint16_t)(s_age_min_win != UINT32_MAX ? s_age_min_win : 0);
        }
#endif
        m_shm->trace_cycle_count = st.cycle_count;
        m_shm->shm_write_avg_us  = (uint16_t)st.shm_write_avg;
    }

    m_shm->cycle_overrun_count = (uint16_t)(m_overrun_count & 0xFFFF);
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
        RT_LOG("SCHED_FIFO failed (need root/CAP_SYS_NICE)");
    }

    /* 锁定当前+未来全部内存页, 防止 RT 线程缺页中断 */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RT_LOG("mlockall failed (page faults possible)");
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(m_rt.cpu_affinity[0], &cpuset);
    if (m_rt.cpu_affinity[1] >= 0) {
        CPU_SET(m_rt.cpu_affinity[1], &cpuset);
    }
    ret = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (ret != 0) {
        RT_LOG("CPU affinity failed");
    }

    printf("[StarkRtWorker] Thread: SCHED_FIFO prio=%d period=%dus cpu=%d,%d\n",
           m_rt.priority, m_rt.period_us,
           m_rt.cpu_affinity[0], m_rt.cpu_affinity[1]);
}

/* GetAvgLatencyUs() — 诊断: 最近 64 次闭环延迟平均值 */

uint32_t StarkRtWorker::GetAvgLatencyUs() const
{
    uint64_t sum = 0;
    uint32_t n   = (m_latency_idx < 64) ? m_latency_idx : 64;
    if (n == 0) return 0;

    for (uint32_t i = 0; i < n; i++) {
        sum += m_latency_history[i];
    }
    return (uint32_t)(sum / n);
}

/*
 * GetPendingState() — 主循环读取 RT 线程请求的状态切换
 * atomic exchange: 读当前值并清零为 STATE_BOOTING
 */

stark_state_t StarkRtWorker::GetPendingState()
{
    return (stark_state_t)__atomic_exchange_n(
        &m_pending_state, (uint32_t)STATE_BOOTING, __ATOMIC_ACQUIRE);
}

}  /* namespace stark_periph_manager_node */
