/*
 * main_loop.cpp — 主循环逻辑实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 从 src/main.cpp 提取: 主循环, 轮询函数, 信号处理, 日志drain.
 * 全局变量通过 extern 引用 main.cpp 中的定义.
 */
#include <signal.h>
#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <memory>

extern "C" {
#include "motor_hal.h"
}

#include <log_helper/LogHelper.h>
#include "motor/motor_init.h"
#include "motor/motor_rt_worker.h"
#include "motor/motor_state.h"
#include "motor/motor_context.h"
#include "utils/rt_log.h"
#include "stark_shm.h"
#include "button/button_handler.hpp"

#ifdef ENABLE_ROS
#include "ros/ros.h"
#include <stark_msgs/PowerCtrl.h>
#endif

using namespace stark_periph_manager_node;

/* 长按关机: 发布 PowerCtrl CTRL_SHUTDOWN, 由 power 节点处理 0x9001 */
#ifdef ENABLE_ROS
static void request_shutdown()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<stark_msgs::PowerCtrl>(
        "/stark/power_ctrl", 1);
    stark_msgs::PowerCtrl ctrl;
    ctrl.cmd = stark_msgs::PowerCtrl::CTRL_SHUTDOWN;
    pub.publish(ctrl);
    ECO_INFO_NEW("[BTN] long press -> publish CTRL_SHUTDOWN");
}
#else
static void request_shutdown()
{
    ECO_WARN_NEW("[BTN] long press ignored (ROS disabled)");
}
#endif

/* 引用 main.cpp 中的全局变量 */
extern volatile int g_running;
extern volatile int g_log_running;
extern stark_periph_manager_node::CanDispatcher* g_dispatcher;
extern stark_periph_manager_node::StarkRtWorker* g_rt_worker;

/* 本文件内部的静态变量 */
static bool g_sensor_configured[STARK_MAX_MOTORS];
static bool g_sensor_logged[STARK_MAX_MOTORS];   /* 仅打印一次日志 */
static bool g_mit_scales_done[STARK_MAX_MOTORS];
static bool g_sdo_telemetry_started = false;
static bool g_report_started = false;    /* 周期上报是否已启动 */
static stark_state_t g_prev_state = STATE_BOOTING;  /* 上一状态, 用于灯效切换 */
static uint8_t g_prev_online_mask = 0;
static uint64_t g_boot_first_online_us = 0;  /* 首个电机上线时刻, 用于单电机超时降级 */
static std::unique_ptr<ButtonHandler> g_btn_handler;

/*
 * log_drain_thread — 从 ring buffer drain 到 ECO_INFO_NEW
 * RT 线程写 ring buffer (lock-free, <1μs), 此非 RT 线程负责 drain.
 */

void* log_drain_thread(void*)
{
    rt_log_sink_init();
    while (g_log_running) {
        rt_log_drain();
        usleep(50000);  /* 50ms drain */
    }
    rt_log_drain();
    rt_log_sink_close();
    return nullptr;
}

/*
 * update_shm_online — 更新 SHM motor_online 掩码
 */

static void update_shm_online(motor_hal_t* hal, stark_shm_t* shm, uint8_t motor_count)
{
    if (!hal || !shm) return;

    uint8_t mask = 0;
    for (uint8_t id = 1; id <= motor_count; ++id) {
        motor_state_t state = motor_hal_get_state(hal, id);
        if (state >= MOTOR_STATE_SWITCH_ON_DIS && state != MOTOR_STATE_UNKNOWN) {
            mask |= (1 << (id - 1));
        }
    }
    shm->motor_online = mask;
}

/*
 * all_motors_online — 检查是否所有已注册电机都已上线
 */

static bool all_motors_online(stark_shm_t* shm, int motor_count)
{
    if (!shm) return false;
    uint8_t expected = (uint8_t)((1 << motor_count) - 1);
    return (shm->motor_online & expected) == expected;
}

/*
 * any_motor_online — 检查是否至少1个电机在线
 */

static bool any_motor_online(stark_shm_t* shm, int motor_count)
{
    if (!shm) return false;
    for (int i = 0; i < motor_count; i++) {
        if (shm->motor_online & (1 << i)) return true;
    }
    return false;
}

/*
 * any_motor_op_enabled — 是否至少1个在线电机已使能 (算法控制中)
 *   通过反馈帧 status_byte bit7 判断电机真实使能状态 (PDO 使能也反映在此)
 */

static bool any_motor_op_enabled(motor_hal_t* hal, stark_shm_t* shm, int motor_count)
{
    if (!hal || !shm) return false;
    for (int id = 1; id <= motor_count; id++) {
        if (!(shm->motor_online & (1 << (id - 1)))) continue;
        motor_feedback_t fb;
        if (motor_hal_get_feedback(hal, (uint8_t)id, &fb) == 0) {
            if (fb.status_byte & 0x80) return true;  /* bit7 = 使能 */
        }
    }
    return false;
}

/*
 * set_state_led — 按节点状态设置灯效
 *   READY=橙呼吸, FAULT=红闪烁, 其余灭灯
 */

static void set_state_led(motor_hal_t* hal, int led_motor_id, stark_state_t state)
{
    if (!hal || led_motor_id < 1) return;

    led_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.enable_mask = LED_ALL;

    switch (state) {
    case STATE_READY:
        cfg.mode = LED_BREATH;
        cfg.r = 255; cfg.g = 128; cfg.b = 0;   /* 橙 */
        break;
    case STATE_FAULT:
        cfg.mode = LED_BLINK;
        cfg.r = 255; cfg.g = 0; cfg.b = 0;     /* 红 */
        break;
    default:
        cfg.enable_mask = 0;                    /* 灭灯 */
        break;
    }

    motor_hal_led_set(hal, (uint8_t)led_motor_id, &cfg);
}

/*
 * poll_fault_check — 故障检测
 *   电机 error_code≠0, 或力矩传感器 err==9 → 有故障 (IMU 暂不判断)
 */

static bool poll_fault_check(motor_hal_t* hal, stark_shm_t* shm, int motor_count)
{
    if (!hal || !shm) return false;

    for (int id = 1; id <= motor_count; id++) {
        if (!(shm->motor_online & (1 << (id - 1)))) continue;

        motor_sensor_t s;
        if (motor_hal_get_sensor(hal, (uint8_t)id, &s) != 0) continue;

        if (s.error_code != 0) return true;      /* 电机故障 */
        if (s.spi_error == 9)  return true;      /* 力矩传感器异常 */
    }
    return false;
}

/*
 * enter_fault_actions — 进入故障态: 停控制 + 失能
 */

static void enter_fault_actions(stark_shm_t* shm, int motor_count)
{
    /* 停 RT 控制, 不接收控制指令 */
    if (g_rt_worker) g_rt_worker->SetActive(false);

    /* 失能在线电机 (DS402 Shutdown) */
    if (g_dispatcher) {
        auto* ctrl = g_dispatcher->GetCtrl();
        if (ctrl) {
            for (int id = 1; id <= motor_count; id++) {
                if (shm->motor_online & (1 << (id - 1))) {
                    int ret = ctrl->SdoWrite(id, 0x6040, 0, 0x0006, 2);
                    if (ret != 0) {
                        ECO_WARN_NEW("[main] SDO shutdown motor {} failed: {}", id, ret);
                    }
                }
            }
        }
    }
}

/*
 * poll_common — 每轮都执行的公共逻辑
 *   motor auto-startup 推进 + SHM online 掩码更新 + sensor 透传自动配置
 */

static void poll_common(motor_hal_t* hal, stark_shm_t* shm, uint8_t motor_count)
{
    if (!hal) return;

    motor_hal_process_pending_startups(hal);
    update_shm_online(hal, shm, motor_count);

    /* 检测电机掉线: online位从1变0, 清零透传标记, 下次上线后自动重配 */
    uint8_t dropped = g_prev_online_mask & ~shm->motor_online;
    for (uint8_t i = 0; i < motor_count; i++) {
        if (dropped & (1 << i)) {
            g_sensor_configured[i] = false;
            g_mit_scales_done[i] = false;
        }
    }
    g_prev_online_mask = shm->motor_online;

    for (uint8_t id = 1; id <= motor_count; id++) {
        if (g_sensor_configured[id - 1]) continue;

        motor_state_t st = motor_hal_get_state(hal, id);
        if (st >= MOTOR_STATE_SWITCH_ON_DIS && st != MOTOR_STATE_UNKNOWN) {
            uint16_t period_div = g_ctx->sensor_period_div;  /* 直接用 period_div (0.5ms 基准) */
            int ret = motor_hal_sensor_config_ex(hal, id, period_div,
                                                 g_ctx->sensor_bus_format,
                                                 g_ctx->sensor_mode,
                                                 g_ctx->sensor_force_module);
            if (ret == 0) {
                g_sensor_configured[id - 1] = true;
                if (!g_sensor_logged[id - 1]) {
                    g_sensor_logged[id - 1] = true;
                    ECO_INFO_NEW("[main] sensor passthrough: motor {} period_div={} bus={} mode={} force={}",
                                 id, period_div,
                                 g_ctx->sensor_bus_format == 3 ? "CANFD BRS" : "Classic CAN",
                                 g_ctx->sensor_mode, g_ctx->sensor_force_module);
                }
            }
        }

        /* 读取 MIT 快控缩放 (仅一次, 电机在线后) */
        if (!g_mit_scales_done[id - 1]) {
            motor_state_t st = motor_hal_get_state(hal, id);
            if (st >= MOTOR_STATE_SWITCH_ON_DIS && st != MOTOR_STATE_UNKNOWN) {
                int ret = motor_hal_read_mit_scales(hal, id);
                g_mit_scales_done[id - 1] = true;
                ECO_INFO_NEW("[main] MIT scales M{}: pmax={:.2f} vmax={:.2f} kpmax={:.0f} kdmax={:.0f} tmax={:.0f} (ret={})",
                             id,
                             motor_hal_get_mit_scale(hal, id, 0),
                             motor_hal_get_mit_scale(hal, id, 1),
                             motor_hal_get_mit_scale(hal, id, 2),
                             motor_hal_get_mit_scale(hal, id, 3),
                             motor_hal_get_mit_scale(hal, id, 4),
                             ret);
            }
        }
    }
}

/*
 * poll_booting — BOOTING 状态逻辑
 *   电机全在线, 进入 READY, 懒启动 RT 线程和 SYNC
 */

static void poll_booting(stark_shm_t* shm, int motor_count,
                         bool enable_rt, bool enable_sync, bool& sync_started)
{
    if (!any_motor_online(shm, motor_count)) {
        g_boot_first_online_us = 0;
        return;
    }

    /* 记录首个电机上线时刻 */
    if (g_boot_first_online_us == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        g_boot_first_online_us = (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
    }

    /* 至少1个电机在线: 启动 RT 线程和 SYNC */
    if (g_rt_worker && !g_rt_worker->IsRunning()) {
        g_rt_worker->Start();
        ECO_INFO_NEW("[main] RT worker started (1KHz, {})",
                     enable_rt ? "SCHED_FIFO 90" : "SCHED_OTHER");
    }

    /* 数据上报: 传感器初始化完就开始 */
    if (!g_report_started && g_rt_worker && g_ctx->report_auto_enable) {
        g_rt_worker->SetReportEnabled(true, g_ctx->report_period_ms);
        g_report_started = true;
        ECO_INFO_NEW("[main] periodic report enabled, period={}ms",
                     g_ctx->report_period_ms);
    }

    if (!sync_started) {
        motor_hal_t* hal = g_ctx->hal;
        if (hal) {
            bool sync_ok = true;
            if (enable_sync) {
                const RtConfig& rc = g_dispatcher->GetRtConfig();
                motor_hal_sync_set_rt(hal, true, rc.sync_priority);
                motor_hal_sync_set_affinity(hal, rc.sync_cpu);
                int ret = motor_hal_sync_start(hal, 1000);  /* 1ms = 1KHz */
                if (ret != 0) {
                    sync_ok = false;
                    ECO_WARN_NEW("[main] SYNC start failed (ret={}), will retry", ret);
                }
            }
            if (sync_ok) {
                sync_started = true;
                if (!g_sdo_telemetry_started) {
                    motor_hal_sdo_telemetry_start(hal);
                    g_sdo_telemetry_started = true;
                    ECO_INFO_NEW("[main] SDO telemetry thread started (temp+pos @5ms)");

                    /* LED 灯灭初始化 */
                    int lm = g_ctx->led_motor_id;
                    if (lm >= 1 && lm <= motor_count) {
                        led_config_t led_off = {0, 0, 0, 0, 0};
                        motor_hal_led_set(hal, (uint8_t)lm, &led_off);
                        ECO_INFO_NEW("[main] LED init off, motor {}", lm);
                    }

                    /* 按键初始化 */
                    if (!g_btn_handler) {
                        ButtonHandler::Config calib_cfg, report_cfg;
                        calib_cfg.gpio_chip  = g_ctx->btn_calib_chip;
                        calib_cfg.line       = g_ctx->btn_calib_line;
                        calib_cfg.long_press_ms = g_ctx->btn_calib_long_press_ms;
                        report_cfg.gpio_chip = g_ctx->btn_report_chip;
                        report_cfg.line      = g_ctx->btn_report_line;
                        g_btn_handler.reset(new ButtonHandler(calib_cfg, report_cfg,
                                                               shm, motor_count,
                                                               request_shutdown));
                        ECO_INFO_NEW("[main] button handler started");
                    }
                }
                if (enable_sync)
                    ECO_INFO_NEW("[main] SYNC thread started (1KHz)");
                else
                    ECO_INFO_NEW("[main] SYNC thread disabled (sync_enable=false)");
            }
        }
    }

    /* RT + SYNC 就绪后立即激活 mailbox 处理 (对齐 daemon 行为),
     *   不等待校准 — 电机使能由控制接口内部自动处理 */
    if (g_rt_worker && g_rt_worker->IsRunning() && sync_started &&
        !g_rt_worker->IsActive()) {
        g_rt_worker->SetActive(true);
        ECO_INFO_NEW("[main] mailbox processing activated (pre-calib)");
    }

    if (all_motors_online(shm, motor_count)) {
        ECO_INFO_NEW("[main] all {} motors online (0x{:02X}), entering READY",
                     motor_count, shm->motor_online);
        g_boot_first_online_us = 0;
        state_transition(STATE_READY);
        return;
    }

    /* 超时降级: 等待 5s 后若仍有电机未上线, 以当前在线电机数进入 READY */
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
        if (now_us - g_boot_first_online_us > 5000000UL) {
            ECO_WARN_NEW("[main] boot timeout: {} of {} motors online (0x{:02X}), entering READY (degraded)",
                         __builtin_popcount(shm->motor_online), motor_count, shm->motor_online);
            g_boot_first_online_us = 0;
            state_transition(STATE_READY);
        }
    }
}

/*
 * poll_ready — READY 状态逻辑
 *   待算法校准 + 控制. 检测到电机使能 → RUNNING
 */

static void poll_ready(motor_hal_t* hal, stark_shm_t* shm, int motor_count)
{
    if (any_motor_op_enabled(hal, shm, motor_count)) {
        ECO_INFO_NEW("[main] motor enabled by algorithm, entering RUNNING");
        state_transition(STATE_RUNNING);
    }
}

/*
 * poll_fault — FAULT 状态逻辑
 *   故障消除自动回 READY
 */

static void poll_fault(motor_hal_t* hal, stark_shm_t* shm, int motor_count)
{
    if (!poll_fault_check(hal, shm, motor_count)) {
        ECO_INFO_NEW("[main] fault cleared, returning READY");
        if (g_rt_worker) g_rt_worker->SetActive(true);  /* 恢复控制 */
        state_transition(STATE_READY);
    }
}

/*
 * poll_rt_pending — RT 状态切换请求处理
 */

static void poll_rt_pending(stark_shm_t* shm)
{
    if (!g_rt_worker) return;

    stark_state_t pending = g_rt_worker->GetPendingState();

    if (pending == STATE_FAULT && g_stark_state != STATE_FAULT) {
        state_transition(STATE_FAULT);

        if (g_dispatcher) {
            auto* ctrl = g_dispatcher->GetCtrl();
            if (ctrl) {
                for (uint8_t id = 1; id <= g_ctx->motor_count; id++) {
                    if (shm->motor_online & (1 << (id - 1))) {
                        int ret = ctrl->SdoWrite(id, 0x6040, 0, 0x0006, 2);
                        ECO_INFO_NEW("[main] SDO Shutdown motor {}: {}",
                                     id, (ret == 0 ? "OK" : "FAIL"));
                    }
                }
            }
        }
    }
}

/*
 * poll_sdo_commands — 处理算法端 SDO 控制请求 (非 RT, 主循环中)
 *   检查 sdo_seq != sdo_ack, 调用 StarkMotorCtrl 处理
 */

static void poll_sdo_commands(motor_hal_t* hal, stark_shm_t* shm)
{
    if (!shm || !hal) return;
    auto* ctrl = g_dispatcher->GetCtrl();
    if (!ctrl) return;

    for (int i = 0; i < STARK_MAX_MOTORS; i++) {
        uint8_t seq = __atomic_load_n(&shm->sdo_seq[i], __ATOMIC_ACQUIRE);
        uint8_t ack = __atomic_load_n(&shm->sdo_ack[i], __ATOMIC_RELAXED);
        if (seq == ack) continue;

        sdo_cmd_slot_t* s = &shm->sdo_cmds[i];
        uint8_t id = s->motor_id;
        if (id < 1) { __atomic_store_n(&shm->sdo_ack[i], seq, __ATOMIC_RELEASE); continue; }

        switch (s->cmd) {
        case STARK_CMD_SDO_CUR:
            ctrl->Torque(id, s->value);
            ECO_INFO_NEW("[SDO] motor {}: cur={}mA", id, s->value);
            break;
        case STARK_CMD_SDO_POS:
            ctrl->AbsPositionEx(id, (float)s->value / 100.0f,
                                (uint16_t)(s->value2 / 100),
                                (uint16_t)(s->feedforward / 100));
            ECO_INFO_NEW("[SDO] motor {}: pos={:.2f}deg accel={} vel={}",
                         id, (float)s->value / 100.0f,
                         (int)(s->value2 / 100), (int)(s->feedforward / 100));
            break;
        case STARK_CMD_SDO_VEL:
            ctrl->SpeedEx(id, (int32_t)(s->value / 100),
                          (int32_t)(s->value2 / 100),
                          (int32_t)(s->value2 / 100));
            ECO_INFO_NEW("[SDO] motor {}: vel={}RPM accel={}",
                         id, (int)(s->value / 100), (int)(s->value2 / 100));
            break;
        case STARK_CMD_SDO_TORQUE_CALIB: {
            int32_t torque_mNm = s->value;
            int ret = motor_hal_torque_calib(hal, id, torque_mNm);
            ECO_INFO_NEW("[SDO] motor {}: torque_calib={:.2f}Nm ret={}",
                         id, (float)torque_mNm / 1000.0f, ret);
            break;
        }
        case STARK_CMD_SDO_MIT_MIGRATE: {
            int ret = motor_hal_mit_migrate_scales(hal, id);
            ECO_INFO_NEW("[SDO] motor {}: MIT migrate Tmax=20 ret={}", id, ret);
            break;
        }
        default:
            break;
        }

        __atomic_store_n(&shm->sdo_ack[i], seq, __ATOMIC_RELEASE);
    }
}

/*
 * poll_led_commands — LED 灯控制 (非 RT, 主循环中)
 */

static void poll_led_commands(motor_hal_t* hal, stark_shm_t* shm, int led_motor_id)
{
    if (!hal || !shm) return;
    if (led_motor_id < 1 || led_motor_id > STARK_MAX_MOTORS) return;

    int i = led_motor_id - 1;
    uint8_t seq = __atomic_load_n(&shm->led_seq[i], __ATOMIC_ACQUIRE);
    uint8_t ack = __atomic_load_n(&shm->led_ack[i], __ATOMIC_RELAXED);
    if (seq == ack) return;

    led_config_t *cfg = &shm->led_cfg[i];

    int ret = motor_hal_led_set(hal, (uint8_t)led_motor_id, cfg);
    if (ret == 0) {
        ECO_INFO_NEW("[LED] motor {} mask=0x{:02X} mode={} R={} G={} B={}",
                     led_motor_id, cfg->enable_mask, (int)cfg->mode,
                     (int)cfg->r, (int)cfg->g, (int)cfg->b);
    } else {
        ECO_WARN_NEW("[LED] motor {} SDO write failed ret={}", led_motor_id, ret);
    }

    __atomic_store_n(&shm->led_ack[i], seq, __ATOMIC_RELEASE);
}

/*
 * main_loop_run() — 主循环 (非阻塞, 状态分发)
 */

void main_loop_run(motor_hal_t* hal, stark_shm_t* shm,
                   int motor_count, CanDispatcher* dispatcher,
                   StarkRtWorker* rt_worker, bool enable_rt, bool enable_sync)
{
    (void)dispatcher;  /* 通过 g_dispatcher 全局变量访问 */
    (void)rt_worker;   /* 通过 g_rt_worker 全局变量访问 */

    bool sync_started = false;

    ECO_INFO_NEW("[main] entering main loop (non-blocking)");

    while (g_running) {
#ifdef ENABLE_ROS
        ros::spinOnce();
#endif

        /* 公共: auto-startup + sensor 配置 */
        poll_common(hal, shm, (uint8_t)motor_count);

        /* 状态分发 */
        switch (g_stark_state) {
        case STATE_BOOTING:
            poll_booting(shm, motor_count, enable_rt, enable_sync, sync_started);
            break;
        case STATE_READY:
            poll_ready(hal, shm, motor_count);
            break;
        case STATE_RUNNING:
            break;
        case STATE_FAULT:
            poll_fault(hal, shm, motor_count);
            break;
        default:
            break;
        }

        /* 故障检测: READY/RUNNING 态检测到故障 → FAULT */
        if ((g_stark_state == STATE_READY || g_stark_state == STATE_RUNNING) &&
            poll_fault_check(hal, shm, motor_count)) {
            ECO_WARN_NEW("[main] fault detected, entering FAULT");
            enter_fault_actions(shm, motor_count);
            state_transition(STATE_FAULT);
        }

        /* 状态变化时刷新灯效 */
        if (g_stark_state != g_prev_state) {
            set_state_led(hal, g_ctx->led_motor_id, g_stark_state);
            g_prev_state = g_stark_state;
        }

        /* RT 线程状态切换请求 */
        poll_rt_pending(shm);

        /* SDO 控制命令 (算法写, 主循环处理) */
        poll_sdo_commands(hal, shm);

        /* LED 灯控制 */
        poll_led_commands(hal, shm, g_ctx->led_motor_id);

        /* 按键 */
        if (g_btn_handler) g_btn_handler->poll();

        /* 同步 SHM node_state */
        if (shm) {
            shm->node_state = g_stark_state;
        }

        usleep(50000);  /* 50ms 轮询 */
    }
}
