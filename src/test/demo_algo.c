/*
 * demo_algo.c -- 外骨骼算法控制示例
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 用法:
 *   ./demo_algo help / -h / --help  帮助信息
 *
 * PDO 连续控制 (算法无需 enable/set_mode):
 *   ./demo_algo torque <mA>            电流控制, 正弦波
 *   ./demo_algo speed <rpm>            速度控制 (CSV), 梯形波
 *   ./demo_algo pos <deg>              位置控制 (CSP), 方波
 *   ./demo_algo pp <deg> [acc] [vel]   轮廓位置 PP, 方波
 *   ./demo_algo pv <rpm> [acc]         轮廓速度 PV, 梯形波
 *   ./demo_algo mit <kp> <kd> [pos] [vel] [tq]  MIT 阻抗控制
 *   ./demo_algo mit_multi <kp> <kd> [pos] [vel] [tq]  MIT 多轴广播
 *   ./demo_algo multi_cur <ma1> <ma2>  多轴电流环 (0x200)
 *   ./demo_algo multi_csv <rpm1> <rpm2> 多轴速度环 (0x200)
 *   ./demo_algo multi_csp <deg1> <deg2> 多轴位置环 (0x200)
 *   ./demo_algo multi_tq <val1> <val2>  多轴力矩环 (0x200, 0.05N.m)
 *   ./demo_algo torque_ctrl <val>       力矩环控制, 正弦 (val=0.05N.m)
 *
 * 校准 / 迁移:
 *   ./demo_algo calib                 编码器零位校准
 *   ./demo_algo calib_torque <id> <Nm> 力矩传感器标定 (理论力矩 Nm)
 *   ./demo_algo calib_torque_zero <id>  力矩传感器零漂标定 (理论力矩=0)
 *   ./demo_algo mit_migrate <id>       MIT缩放迁移 (Tmax→20Nm + 保存Flash)
 *
 * SDO 单帧控制 (通过 mailbox → main_loop):
 *   ./demo_algo sdo cur <id> <mA>                    单电机电流
 *   ./demo_algo sdo cur <id1> <id2> <mA>              双电机同值电流
 *   ./demo_algo sdo cur <id1> <id2> <mA1> <mA2>       双电机不同值电流
 *   ./demo_algo sdo pos <id> <deg>                    单电机位置 (PP)
 *   ./demo_algo sdo pos <id1> <id2> <deg>             双电机同值位置
 *   ./demo_algo sdo pos <id1> <id2> <deg1> <deg2>     双电机不同值位置
 *   ./demo_algo sdo vel <id> <rpm>                    单电机速度 (PV)
 *   ./demo_algo sdo vel <id1> <id2> <rpm>             双电机同值速度
 *   ./demo_algo sdo vel <id1> <id2> <rpm1> <rpm2>     双电机不同值速度
 *
 * PDO 单帧控制 (通过 mailbox → RT 线程):
 *   ./demo_algo pdo cur <id> <mA>                    单电机电流
 *   ./demo_algo pdo cur <id1> <id2> <mA>              双电机同值电流
 *   ./demo_algo pdo cur <id1> <id2> <mA1> <mA2>       双电机不同值电流
 *   ./demo_algo pdo pos <id> <deg>                    单电机位置 (PP)
 *   ./demo_algo pdo pos <id1> <id2> <deg>             双电机同值位置
 *   ./demo_algo pdo pos <id1> <id2> <deg1> <deg2>     双电机不同值位置
 *   ./demo_algo pdo vel <id> <rpm>                    单电机速度 (PV)
 *   ./demo_algo pdo vel <id1> <id2> <rpm>             双电机同值速度
 *   ./demo_algo pdo vel <id1> <id2> <rpm1> <rpm2>     双电机不同值速度
 *   ./demo_algo pdo tq <id> <val>                       单电控力矩(0.05N.m)
 *   ./demo_algo pdo tq <id1> <id2> <val>                双电机同值力矩
 *
 * 管理/状态:
 *   ./demo_algo enable/disable/estop/clearf <id>
 *   ./demo_algo stat / report
 *
 * 编译: gcc -O2 demo_algo.c -lpthread -lrt -lm -o demo_algo
 */

#include "../stark_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

static volatile int g_running = 1;

/* 独立 trace SHM 句柄 (上行反馈打点, 失败不阻塞) */
static stark_trace_t g_trace;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static float counts_to_deg(int16_t counts)
{
    return (float)counts * 360.0f / 65536.0f;
}

/* 力矩控制: 正弦波扫描 */
static void run_torque(stark_client_t* c, int32_t amplitude_ma)
{
    printf("[torque] 正弦波扫描, 振幅=%d mA, 周期=2s\n", amplitude_ma);

    uint64_t t0 = now_ms();
    int32_t last = 0;
    uint32_t rpt_ver = 0;

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float phase = (float)(t % 2000) / 2000.0f * 2.0f * M_PI;
        int32_t ma = (int32_t)((float)amplitude_ma * sinf(phase));

        if (stark_online(c, 1)) stark_torque(c, 1, ma);
        if (stark_online(c, 2)) stark_torque(c, 2, ma);

        if (ma != last) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%4lus] target=%5d mA  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), ma, counts_to_deg(fb.position), fb.current_iq);
            last = ma;
        }

        /* 周期上报数据: stark_report_try_read 一行获取 */
        {
            const PeriodicUploadData* d;
            if (stark_report_try_read(c, &rpt_ver, &d)) {
                printf("[rpt] cyc=%u ts=%u | "
                       "IMU roll=%.1f pitch=%.1f yaw=%.1f | "
                       "R: vel=%dRPM ang=%.1fdeg cur=%dmA temp=%.2fC "
                       "L: vel=%dRPM ang=%.1fdeg cur=%dmA temp=%.2fC | "
                       "S1: hall=%u,%u,%u tor=%u kne=%d "
                       "S2: hall=%u,%u,%u tor=%u kne=%d\n",
                       d->frame_cycle, d->imu_ts_us,
                       d->gyro_roll, d->gyro_pitch, d->gyro_yaw,
                       d->RealtimeVelocity, d->motor_abs_angle / 10.0f,
                       d->cal_Iq_current, d->motor_temp / 100.0f,
                       d->RealtimeVelocity_left, d->motor_abs_angle_left / 10.0f,
                       d->cal_Iq_current_left, d->motor_temp_left / 100.0f,
                       d->hall_a_data, d->hall_b_data, d->hall_c_data,
                       d->df181_torque, d->knee_hall,
                       d->hall_a_data_left, d->hall_b_data_left, d->hall_c_data_left,
                       d->df181_torque_left, d->knee_hall_left);
            }
        }

        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}

/* 速度控制: 梯形波 */
static void run_speed(stark_client_t* c, float max_rpm)
{
    printf("[speed] 梯形波, ±%.0f RPM, 每段1s\n", max_rpm);

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float rpm;
        uint64_t phase = t % 4000;

        if (phase < 1000) {
            rpm = max_rpm * (float)phase / 1000.0f;
        } else if (phase < 2000) {
            rpm = max_rpm;
        } else if (phase < 3000) {
            rpm = max_rpm - max_rpm * (float)(phase - 2000) / 1000.0f;
        } else {
            rpm = 0.0f - max_rpm * (float)(phase - 3000) / 1000.0f;
        }

        if (stark_online(c, 1)) stark_speed(c, 1, rpm);
        if (stark_online(c, 2)) stark_speed(c, 2, rpm);

        if (t % 200 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f RPM  fb_vel=%d RPM  fb_pos=%.1f deg\n",
                   (unsigned long)(t / 1000), rpm, fb.velocity, counts_to_deg(fb.position));
        }
        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(5000);
    }
}

/* 位置控制: 方波 (PP 模式, 电机不支持 CSP) */
static void run_position(stark_client_t* c, float amplitude_deg)
{
    float accel = 2000.0f;
    float vel   = 10.0f;
    printf("[pos] 方波 (PP), ±%.1f deg, accel=%.0f vel=%.0f, 2s/拍\n", amplitude_deg, accel, vel);

    uint64_t t0 = now_ms();

    while (g_running) {
        uint64_t t = now_ms() - t0;
        float target = ((t / 2000) % 2 == 0) ? amplitude_deg : -amplitude_deg;

        if (stark_online(c, 1)) stark_pp(c, 1,  target, accel, vel);
        if (stark_online(c, 2)) stark_pp(c, 2, -target, accel, vel);

        if (t % 500 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f deg  fb_pos=%.1f deg  fb_cur=%d mA\n",
                   (unsigned long)(t / 1000), target,
                   counts_to_deg(fb.position), fb.current_iq);
        }
        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}

/* MIT 阻抗控制: mit <kp> <kd> [pos_deg] [vel_rpm] [torque_nm] */
static void run_mit(stark_client_t* c, float kp, float kd,
                    float pos_deg, float vel_rpm, float torque_nm)
{
    printf("[MIT] pos=%.1f° vel=%.0fRPM kp=%.1f kd=%.1f tq=%.1fNm\n",
           pos_deg, vel_rpm, kp, kd, torque_nm);

    while (g_running) {
        if (stark_online(c, 1)) stark_mit(c, 1, pos_deg, vel_rpm, kp, kd, torque_nm);
        if (stark_online(c, 2)) stark_mit(c, 2, pos_deg, vel_rpm, kp, kd, torque_nm);

        static int cnt = 0;
        if (++cnt % 50 == 0) {
            motor_data_t fb1 = stark_fb(c, 1);
            motor_data_t fb2 = stark_fb(c, 2);
            printf("[MIT] M1: pos=%.1f deg cur=%d mA  M2: pos=%.1f deg cur=%d mA\n",
                   counts_to_deg(fb1.position), fb1.current_iq,
                   counts_to_deg(fb2.position), fb2.current_iq);
        }
        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}

/* 轮廓位置 PP: 方波 */
static void run_pp(stark_client_t* c, float amplitude_deg, float accel, float vel)
{
    printf("[PP] 轮廓位置, ±%.1f deg, accel=%.0fRPM/s vel=%.0fRPM\n", amplitude_deg, accel, vel);

    uint64_t t0 = now_ms();
    while (g_running) {
        uint64_t t = now_ms() - t0;
        float target = ((t / 2000) % 2 == 0) ? amplitude_deg : -amplitude_deg;

        if (stark_online(c, 1)) stark_pp(c, 1,  target, accel, vel);
        if (stark_online(c, 2)) stark_pp(c, 2, -target, accel, vel);

        if (t % 500 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f deg  fb_pos=%.1f deg\n",
                   (unsigned long)(t / 1000), target, counts_to_deg(fb.position));
        }
        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}

/* 轮廓速度 PV: 梯形波 */
static void run_pv(stark_client_t* c, float max_rpm, float accel)
{
    printf("[PV] 轮廓速度, ±%.0f RPM, accel=%.0fRPM/s\n", max_rpm, accel);

    uint64_t t0 = now_ms();
    while (g_running) {
        uint64_t t = now_ms() - t0;
        float rpm;
        uint64_t phase = t % 4000;

        if (phase < 1000)
            rpm = max_rpm * (float)phase / 1000.0f;
        else if (phase < 2000)
            rpm = max_rpm;
        else if (phase < 3000)
            rpm = max_rpm - max_rpm * (float)(phase - 2000) / 1000.0f;
        else
            rpm = 0.0f - max_rpm * (float)(phase - 3000) / 1000.0f;

        if (stark_online(c, 1)) stark_pv(c, 1,  rpm, accel);
        if (stark_online(c, 2)) stark_pv(c, 2, -rpm, accel);

        if (t % 200 < 10) {
            motor_data_t fb = stark_fb(c, 1);
            printf("[t=%3lus] target=%.0f RPM  fb_vel=%d RPM\n",
                   (unsigned long)(t / 1000), rpm, fb.velocity);
        }
        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(5000);
    }
}

/* ===== 多轴控制 (0x200, stark_multi) ===== */

static const char* _multi_mode_name(int mode)
{
    switch (mode) {
    case STARK_MODE_CURRENT: return "CURRENT";
    case STARK_MODE_CSV:     return "CSV";
    case STARK_MODE_CSP:     return "CSP";
    case STARK_MODE_TORQUE:  return "TORQUE";
    default:                 return "?";
    }
}

static const char* _multi_mode_unit(int mode)
{
    switch (mode) {
    case STARK_MODE_CURRENT: return "mA";
    case STARK_MODE_CSV:     return "RPM";
    case STARK_MODE_CSP:     return "deg";
    case STARK_MODE_TORQUE:  return "0.05N.m";
    default:                 return "?";
    }
}

static void run_multi_ctrl(stark_client_t* c, int mode, float v1, float v2)
{
    const char* nm = _multi_mode_name(mode);
    const char* un = _multi_mode_unit(mode);
    printf("[multi] %s mode, M1=%.1f%s M2=%.1f%s\n", nm, v1, un, v2, un);
    printf("  mode=%d, 0x200 多轴广播 (stark_multi)\n", mode);

    int32_t last_t1 = 0, last_t2 = 0;
    int cnt = 0;

    while (g_running) {
        int32_t t1, t2;
        if (mode == STARK_MODE_CSP) {
            /* CSP: 输入角度, 转 counts */
            t1 = (int32_t)(v1 * 65536.0f / 360.0f);
            t2 = (int32_t)(v2 * 65536.0f / 360.0f);
        } else {
            t1 = (int32_t)v1;
            t2 = (int32_t)v2;
        }

        stark_multi(c, mode, t1, 0, 0, t2, 0, 0);

        if (++cnt % 200 == 0 || t1 != last_t1 || t2 != last_t2) {
            motor_data_t fb1 = stark_fb(c, 1);
            motor_data_t fb2 = stark_fb(c, 2);
            printf("[multi %s] M1: pos=%.1f° cur=%dmA tq=%.2fNm  "
                   "M2: pos=%.1f° cur=%dmA tq=%.2fNm\n",
                   nm,
                   counts_to_deg(fb1.position), fb1.current_iq,
                   (float)fb1.torque_nm * 0.05f,
                   counts_to_deg(fb2.position), fb2.current_iq,
                   (float)fb2.torque_nm * 0.05f);
            last_t1 = t1; last_t2 = t2;
        }
        stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}

/* 力矩环连续控制, val=0.05N.m 幅值, 正弦波 */
static void run_torque_ctrl(stark_client_t* c, int32_t val)
{
    printf("[torque_ctrl] 力矩环正弦, 幅值=%d (%.1fN.m)\n", val, (float)val * 0.05f);
    printf("  按 Ctrl+C 停止\n");

    uint64_t t0 = now_ms();
    uint64_t last = now_ms();
    int cnt = 0;

    while (g_running) {
        uint64_t now = now_ms();
        uint64_t elapsed = now - t0;
        int32_t target = (int32_t)((float)val * sinf((float)(elapsed % 2000) / 2000.0f * 2.0f * M_PI));

        if (stark_online(c, 1)) stark_torque_ctrl(c, 1, target);
        if (stark_online(c, 2)) stark_torque_ctrl(c, 2, target);

        if (now - last >= 500) {
            motor_data_t fb1 = stark_fb(c, 1);
            motor_data_t fb2 = stark_fb(c, 2);
            printf("[%4lu] cmd=%.1fN.m  M1: tq=%.2fN.m Iq=%dmA  M2: tq=%.2fN.m Iq=%dmA\n",
                   elapsed / 100, (float)target * 0.05f,
                   (float)fb1.torque_nm * 0.05f, fb1.current_iq,
                   (float)fb2.torque_nm * 0.05f, fb2.current_iq);
            last = now;
        }

        if (++cnt % 200 == 0) stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}


/* MIT 多轴广播控制 (0x210, 双电机一帧) */
static void run_mit_multi(stark_client_t* c, float kp, float kd,
                          float pos_deg, float vel_rpm, float torque_nm)
{
    printf("[MIT multi] pos=%.1f° vel=%.0fRPM kp=%.1f kd=%.1f tq=%.1fNm\n",
           pos_deg, vel_rpm, kp, kd, torque_nm);
    printf("  按 Ctrl+C 停止\n");

    uint64_t last = now_ms();

    while (g_running) {
        if (stark_online(c, 1) && stark_online(c, 2)) {
            stark_mit_multi(c,
                pos_deg, vel_rpm, kp, kd, torque_nm,
                pos_deg, vel_rpm, kp, kd, torque_nm);
        }

        uint64_t now = now_ms();
        if (now - last >= 500) {
            motor_data_t fb1 = stark_fb(c, 1);
            motor_data_t fb2 = stark_fb(c, 2);
            printf("[MIT multi] M1: pos=%.1f cur=%dmA  M2: pos=%.1f cur=%dmA\n",
                   counts_to_deg(fb1.position), fb1.current_iq,
                   counts_to_deg(fb2.position), fb2.current_iq);
            last = now;
        }

        static int cnt = 0;
        if (++cnt % 200 == 0) stark_heartbeat(c);
        stark_trace_fb_update(c, &g_trace);
        usleep(1000);
    }
}

/* PeriodicUploadData display */
static void run_report_loop(stark_client_t* c)
{
    printf("[report] waiting for data stream...\n");

    while (!stark_report_data(c)) {
        usleep(100000);
    }

    uint32_t rpt_ver = 0;
    while (g_running) {
        const PeriodicUploadData* d;
        if (!stark_report_try_read(c, &rpt_ver, &d)) { usleep(1000); continue; }

        printf("=== [%ums] ver=%u frame=%u m_ts=%u i_ts=%u s_ts=%u ===\n",
               d->timestamp_ms, rpt_ver,
               d->frame_cycle, d->motor_ts_us, d->imu_ts_us, d->sensor_ts_us);

        /* IMU */
        printf("IMU  gyro(x=%.2f y=%.2f z=%.2f)dps  "
               "quat(w=%.4f x=%.4f y=%.4f z=%.4f)  "
               "euler(roll=%.1f pitch=%.1f yaw=%.1f)deg  "
               "acc(x=%.3f y=%.3f z=%.3f)g  press=%.1fhPa\n",
               d->gyro_dps_x, d->gyro_dps_y, d->gyro_dps_z,
               d->quat_w, d->quat_x, d->quat_y, d->quat_z,
               d->gyro_roll, d->gyro_pitch, d->gyro_yaw,
               d->acc_x, d->acc_y, d->acc_z, d->air_pressure);

        /* M1 */
        printf("M1   vel=%7dRPM  ang=%6.1fdeg  Iq=%5dmA  "
               "busI=%4dmA  temp=%6.2fC  fault=0x%04X  state=0x%02X\n",
               d->RealtimeVelocity,
               d->motor_abs_angle / 10.0f,
               d->cal_Iq_current,
               d->cal_bus_current * 10,
               d->motor_temp / 100.0f,
               d->fault_code, d->motor_state);

        /* M2 */
        printf("M2   vel=%7dRPM  ang=%6.1fdeg  Iq=%5dmA  "
               "busI=%4dmA  temp=%6.2fC  fault=0x%04X  state=0x%02X\n",
               d->RealtimeVelocity_left,
               d->motor_abs_angle_left / 10.0f,
               d->cal_Iq_current_left,
               d->cal_bus_current_left * 10,
               d->motor_temp_left / 100.0f,
               d->fault_code_left, d->motor_state_left);

        /* S1 */
        printf("S1   hall(a=%u b=%u c=%u)  torque=%u  knee=%d  land=%u  valid=%u\n",
               d->hall_a_data, d->hall_b_data, d->hall_c_data,
               d->df181_torque, d->knee_hall,
               d->key_landing, d->torque_valid);

        /* S2 */
        printf("S2   hall(a=%u b=%u c=%u)  torque=%u  knee=%d  land=%u  valid=%u\n\n",
               d->hall_a_data_left, d->hall_b_data_left, d->hall_c_data_left,
               d->df181_torque_left, d->knee_hall_left,
               d->key_landing_left, d->torque_valid_left);

        /* 0x6B0 力矩原始计数, 已并入 PeriodicUploadData, 直接从 d 取 */
        printf("SPI  M1[torque=%.2f valid=%u err=%u]  M2[torque=%.2f valid=%u err=%u]  (0x6B0 SPI力矩)\n",
               d->spi_torque, d->spi_valid, d->spi_error,
               d->spi_torque_left, d->spi_valid_left, d->spi_error_left);

        printf("TQ   M1[tq=%.2fN.m]  M2[tq=%.2fN.m]  (0x300 驱动力矩)\n",
               (float)d->torque_feedback * 0.05f,
               (float)d->torque_feedback_left * 0.05f);

        /* 足底压力 */
        printf("FOOT L:%4u %4u %4u  R:%4u %4u %4u  ts=%llu\n",
               d->foot_pressure.left.adc[0], d->foot_pressure.left.adc[1],
               d->foot_pressure.left.adc[2],
               d->foot_pressure.right.adc[0], d->foot_pressure.right.adc[1],
               d->foot_pressure.right.adc[2],
               (unsigned long long)d->foot_pressure.timestamp_us);

        printf("\n");
    }
}

/* 足底压力数据 */
static void run_foot_loop(stark_client_t* c)
{
    printf("[foot] 每帧只打印一条 (新帧=timetamp_us变化)\n");

    uint64_t last_ts = 0;
    while (g_running) {
        foot_pressure_data_t fp = stark_foot_pressure(c);

        if (fp.timestamp_us && fp.timestamp_us != last_ts) {
            last_ts = fp.timestamp_us;
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            printf("[%lld.%06ld] ts=%llu L:%4u %4u %4u  R:%4u %4u %4u\n",
                   (long long)ts.tv_sec, ts.tv_nsec / 1000,
                   (unsigned long long)fp.timestamp_us,
                   fp.left.adc[0], fp.left.adc[1], fp.left.adc[2],
                   fp.right.adc[0], fp.right.adc[1], fp.right.adc[2]);
        }
        usleep(1000);
    }
}

/* 只读反馈, 不发控制 */
static void run_stat_loop(stark_client_t* c)
{
    printf("[stat] 只读反馈, 不发控制命令\n");

    while (g_running) {
        motor_data_t fb1 = stark_fb(c, 1);
        motor_data_t fb2 = stark_fb(c, 2);
        imu_data_t imu = stark_imu(c);
        barometer_data_t baro = stark_baro(c);

        printf("[stat] M1: pos=%.1f deg vel=%d RPM cur=%d mA temp=%.1f C  "
               "M2: pos=%.1f deg vel=%d RPM cur=%d mA tq=%.2fN.m  "
               "IMU: yaw=%.1f pitch=%.1f roll=%.1f\n",
               counts_to_deg(fb1.position), fb1.velocity, fb1.current_iq, (float)fb1.temperature * 0.1f,
               counts_to_deg(fb2.position), fb2.velocity, fb2.current_iq,
               (float)fb2.torque_nm * 0.05f,
               imu.yaw, imu.pitch, imu.roll);

        printf("[stat] BARO: press=%.2fhPa temp=%.1fC alt=%.2fm\n",
               baro.pressure_hpa, baro.temperature_c, baro.altitude_m);

        usleep(200000);  /* 5Hz */
    }
}

/* 等待 SHM active_idx 翻转 min_flips 次后读取反馈
 * 等多次翻转 = 电机电流/位置/速度有时间稳定到目标值
 * 返回 1=等到, 0=超时; out 始终填充最新反馈 */
static int _wait_new_fb_frame(stark_client_t* c, int id, motor_data_t* out,
                               int max_ms, int interval_us, int min_flips)
{
    uint32_t idx_old = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    int flips = 0;
    motor_data_t last = {0};

    int loops = max_ms * 1000 / interval_us;
    for (int i = 0; i < loops; i++) {
        usleep(interval_us);
        uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
        if (idx != idx_old) {
            flips++;
            idx_old = idx;
            last = c->shm->fb_buffer[idx].motor[id - 1];
            if (flips >= min_flips) {
                *out = last;
                return 1;
            }
        }
    }
    uint32_t idx = __atomic_load_n(&c->shm->active_idx, __ATOMIC_ACQUIRE);
    *out = c->shm->fb_buffer[idx].motor[id - 1];
    return 0;
}

static void usage(void)
{
    printf("用法: ./demo_algo <mode> [args...]\n\n");
    printf("PDO 连续控制 (算法无需 enable/set_mode):\n");
    printf("  torque <mA>           电流环, 正弦波\n");
    printf("  multi_cur <ma1> <ma2>  多轴电流环 (0x200, stark_multi)\n");
    printf("  multi_csv <rpm1> <rpm2> 多轴速度环 (0x200)\n");
    printf("  multi_csp <deg1> <deg2> 多轴位置环 (0x200)\n");
    printf("  multi_tq  <val1> <val2> 多轴力矩环 (0x200, 0.05N.m)\n");
    printf("  speed  <rpm>          速度控制 (CSV), 梯形波\n");
    printf("  csv    <rpm>          CSV 速度, 梯形波\n");
    printf("  pv     <rpm> [acc]    轮廓速度 PV, 梯形波\n");
    printf("  pos    <deg>          位置控制 (PP), 方波 (电机不支持 CSP)\n");
    printf("  csp    <deg>          PP 位置 (同 pos)\n");
    printf("  pp     <deg> [acc] [v] 轮廓位置 PP, 方波\n");
    printf("  mit    <kp> <kd> [pos_deg] [vel_rpm] [torque_Nm]  MIT 阻抗\n");
    printf("  mit_multi <kp> <kd> [pos_deg] [vel_rpm] [tq_Nm]  MIT 多轴广播\n");
    printf("\nSDO 单帧控制 (sdo cur/pos/vel, 支持单/双电机):\n");
    printf("  sdo cur <id> <mA>                    单电机电流\n");
    printf("  sdo cur <id1> <id2> <mA>             双电机同值电流\n");
    printf("  sdo cur <id1> <id2> <mA1> <mA2>      双电机不同值电流\n");
    printf("  sdo pos <id> <deg>                   单电机位置 (PP)\n");
    printf("  sdo pos <id1> <id2> <deg>            双电机同值位置 (PP)\n");
    printf("  sdo pos <id1> <id2> <deg1> <deg2>    双电机不同值位置 (PP)\n");
    printf("  sdo vel <id> <rpm>                   单电机速度 (PV)\n");
    printf("  sdo vel <id1> <id2> <rpm>            双电机同值速度 (PV)\n");
    printf("  sdo vel <id1> <id2> <rpm1> <rpm2>    双电机不同值速度 (PV)\n");
    printf("\nPDO 单帧控制 (pdo cur/pos/vel, 支持单/双电机):\n");
    printf("  pdo cur <id> <mA>                    单电机电流\n");
    printf("  pdo cur <id1> <id2> <mA>             双电机同值电流\n");
    printf("  pdo cur <id1> <id2> <mA1> <mA2>      双电机不同值电流\n");
    printf("  pdo pos <id> <deg> [acc] [vel]       单电机位置 (PP)\n");
    printf("  pdo pos <id1> <id2> <deg> [acc] [vel]双电机同值位置 (PP)\n");
    printf("  pdo pos <id1> <id2> <d1> <d2> [a] [v]双电机不同值位置 (PP)\n");
    printf("  pdo vel <id> <rpm> [acc]             单电机速度 (PV)\n");
    printf("  pdo vel <id1> <id2> <rpm> [acc]      双电机同值速度 (PV)\n");
    printf("  pdo vel <id1> <id2> <r1> <r2> [acc]  双电机不同值速度 (PV)\n");
    printf("  pdo mit <id> <pos> <vel> <kp> <kd> [tq]  单电机MIT单帧调试\n");
    printf("  pdo mit <id1> <id2> <pos> <vel> <kp> <kd> [tq] 双电机MIT单帧\n");
    printf("\n管理命令:\n");
    printf("  enable  <id>          使能电机\n");
    printf("  disable <id>          失能电机\n");
    printf("  estop   <id>          急停\n");
    printf("  clearf  <id>          清故障\n");
    printf("  calib                 零位校准 (先 SDO 失能再下发零位, 全在线电机)\n");
    printf("  calib_torque <id> <Nm> 力矩传感器标定 (理论力矩 Nm)\n");
    printf("  calib_torque_zero <id>  力矩传感器零漂标定 (理论力矩=0)\n");
    printf("  mit_migrate <id>        MIT缩放迁移: 写 Tmax=20 并保存 Flash\n");
    printf("  led  <id> <mask> <mode> [r] [g] [b]  LED 灯控制\n");
    printf("  btn                   读取按键上报状态\n");
    printf("\n状态:\n");
    printf("  stat                  只读反馈\n");
    printf("  report                周期上报数据\n");
    printf("  foot                  足底压力数据\n");
    printf("\n示例:\n");
    printf("  ./demo_algo torque 200            # 电流 ±200mA 正弦波\n");
    printf("  ./demo_algo speed 10              # 速度 ±10RPM 梯形波\n");
    printf("  ./demo_algo pv 30 1000            # PV ±30RPM acc=1000\n");
    printf("  ./demo_algo pos 15                # 位置 ±15° 方波\n");
    printf("  ./demo_algo pp 15 2000 10         # PP ±15° 方波\n");
    printf("  ./demo_algo mit 30 5              # MIT Kp=30 Kd=5 零目标位置\n");
    printf("  ./demo_algo mit 30 5 0 0 10       # MIT Kp=30 Kd=5 10Nm前馈力矩\n");
    printf("  ./demo_algo mit 0 0 0 0 15         # MIT 纯力矩控制 15Nm\n");
    printf("  ./demo_algo mit_multi 30 5          # MIT 多轴广播 Kp=30 Kd=5\n");
    printf("  ./demo_algo mit_multi 50 10 0 0 8  # MIT 多轴 8Nm双电机\n");
    printf("  ./demo_algo multi_cur 500 300      # 多轴电流 M1=500mA M2=300mA\n");
    printf("  ./demo_algo multi_cur 0 0          # 多轴电流 双电机0mA\n");
    printf("  ./demo_algo multi_csv 10 -10       # 多轴速度 M1=10 M2=-10RPM\n");
    printf("  ./demo_algo multi_csp 15 -15       # 多轴位置 ±15°\n");
    printf("  ./demo_algo multi_tq 200 -200      # 多轴力矩 ±200(±10Nm)\n");
    printf("  ./demo_algo calib_torque 1 0        # M1 力矩零漂标定(零负载)\n");
    printf("  ./demo_algo calib_torque 2 17.15    # M2 力矩标定(挂5kg×0.35m)\n");
    printf("  ./demo_algo calib_torque_zero 1     # M1 零漂标定(快捷, tau=0)\n");
    printf("  ./demo_algo mit_migrate 1           # M1 MIT缩放迁移(Tmax→20)\n");
    printf("  ./demo_algo sdo cur 1 500         # SDO M1=500mA\n");
    printf("  ./demo_algo sdo cur 1 2 500       # SDO M1=M2=500mA\n");
    printf("  ./demo_algo sdo cur 1 2 500 300   # SDO M1=500 M2=300mA\n");
    printf("  ./demo_algo sdo pos 1 30          # SDO M1=30°\n");
    printf("  ./demo_algo sdo pos 1 2 30        # SDO M1=M2=30°\n");
    printf("  ./demo_algo sdo pos 1 2 30 20     # SDO M1=30° M2=20°\n");
    printf("  ./demo_algo sdo vel 1 10          # SDO M1=10RPM\n");
    printf("  ./demo_algo sdo vel 1 2 15        # SDO M1=M2=15RPM\n");
    printf("  ./demo_algo pdo cur 1 500         # PDO M1=500mA\n");
    printf("  ./demo_algo pdo cur 1 2 500 300   # PDO M1=500 M2=300mA\n");
    printf("  ./demo_algo pdo pos 1 30          # PDO M1=30° (PP)\n");
    printf("  ./demo_algo pdo pos 1 2 30 500 10 # PDO M1=M2=30° acc=500 vel=10\n");
    printf("  ./demo_algo pdo vel 1 10 500      # PDO M1=10RPM acc=500\n");
    printf("  ./demo_algo pdo mit 1 0 0 5 2     # PDO M1 MIT单帧 kp=5 kd=2\n");
    printf("  ./demo_algo pdo mit 1 2 0 0 30 5  # PDO 双MIT单帧 kp=30 kd=5\n");
    printf("  ./demo_algo report                # 周期上报\n");
    printf("  ./demo_algo stat                  # 只读反馈\n");
    printf("  ./demo_algo led 1 0xF0 0 255 0 0  # M1 红灯全亮常亮\n");
    printf("  ./demo_algo led 1 0x10 1 0 0 255  # M1 LED1 蓝色闪烁\n");
    printf("  ./demo_algo led 1 0 0 0 0 0 0     # M1 全灭\n");
    printf("  ./demo_algo btn                    # 读取按键上报状态\n");
}


int main(int argc, char** argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    if (argc < 2) { usage(); return 1; }

    const char* mode = argv[1];

    if (strcmp(mode, "help") == 0 || strcmp(mode, "-h") == 0 || strcmp(mode, "--help") == 0) {
        usage();
        return 0;
    }

    /* 连接 SHM (允许 stark_node 后启动) */
    stark_client_t c;
    printf("[init] 等待 stark_node...\n");
    while (stark_open(&c) != 0) {
        usleep(100000);
    }
    printf("[init] SHM 已连接\n");

    /* 打开独立 trace SHM (上行耗时跟踪, 失败不阻塞控制) */
    stark_trace_open(&g_trace);

    /* stat/report/btn/led/mgmt 模式不需要校准, 等电机在线即可 */
    int need_calib = (strcmp(mode, "torque") == 0 || strcmp(mode, "speed") == 0 ||
                      strcmp(mode, "pos") == 0    || strcmp(mode, "pp") == 0 ||
                      strcmp(mode, "pv") == 0    || strcmp(mode, "mit") == 0 ||
                      strcmp(mode, "multi_cur") == 0 ||
                      strcmp(mode, "multi_csv") == 0 ||
                      strcmp(mode, "multi_csp") == 0 ||
                      strcmp(mode, "multi_tq") == 0 ||
                      strcmp(mode, "torque_ctrl") == 0 ||
                      strcmp(mode, "mit_multi") == 0);

    if (need_calib) {
        printf("[init] 等待校准完成...\n");
        while (!stark_ready(&c)) {
            if (stark_state(&c) == 3) {
                printf("ERR: FAULT 状态, 退出\n");
                stark_close(&c);
                return 1;
            }
            usleep(100000);
        }
        printf("[init] 校准完成, 电机在线: %d %d\n", stark_online(&c, 1), stark_online(&c, 2));
    } else {
        printf("[init] 等待电机在线...\n");
        while (!stark_online(&c, 1) && !stark_online(&c, 2)) {
            if (stark_state(&c) == 3) {
                printf("ERR: FAULT 状态, 退出\n");
                stark_close(&c);
                return 1;
            }
            usleep(100000);
        }
        printf("[init] 电机在线: %d %d\n", stark_online(&c, 1), stark_online(&c, 2));
    }

    if (strcmp(mode, "stat") == 0) {
        run_stat_loop(&c);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "report") == 0) {
        run_report_loop(&c);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "foot") == 0) {
        run_foot_loop(&c);
        stark_close(&c);
        return 0;
    }

    /* 管理命令: 直接执行后退出 */
    if (strcmp(mode, "enable") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_enable(&c, id);
        printf("motor %d enabled\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "disable") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_disable(&c, id);
        printf("motor %d disabled\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "estop") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_estop(&c, id);
        printf("motor %d emergency stopped\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "clearf") == 0) {
        if (argc < 3) { printf("ERR: need motor id\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        stark_clear_fault(&c, id);
        printf("motor %d fault cleared\n", id);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "calib_torque") == 0) {
        if (argc < 4) {
            printf("Usage: calib_torque <id> <torque_Nm>\n");
            return 1;
        }
        int id = atoi(argv[2]);
        float torque_nm = (float)atof(argv[3]);
        if (id < 1 || id > 2) {
            printf("calib_torque: id must be 1 or 2\n");
            return 1;
        }
        if (torque_nm < -100.0f || torque_nm > 100.0f) {
            printf("calib_torque: torque out of range [-100, 100] Nm\n");
            return 1;
        }
        int32_t torque_mNm = (int32_t)(torque_nm * 1000.0f);
        printf("Torque calib: M%d = %.2f Nm (%d mNm)\n", id, torque_nm, torque_mNm);
        printf("Ensure: motor DISABLED, joint STATIONARY, load STABLE.\n");
        printf("Proceed? (y/N): ");
        char confirm = (char)getchar();
        if (confirm != 'y' && confirm != 'Y') {
            printf("Cancelled.\n");
            stark_close(&c);
            return 0;
        }
        stark_sdo_torque_calib(&c, id, torque_mNm);
        printf("Sent. Wait 2s, then check 0x6077 for verification.\n");
        usleep(100000);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "calib_torque_zero") == 0) {
        if (argc < 3) { printf("Usage: calib_torque_zero <id>\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        if (id < 1 || id > 2) { printf("calib_torque_zero: id must be 1 or 2\n"); stark_close(&c); return 1; }
        printf("Torque zero calib: M%d (tau=0Nm)\n", id);
        printf("Ensure: motor DISABLED, joint at mechanical ZERO, no external load.\n");
        printf("Proceed? (y/N): ");
        char confirm = (char)getchar();
        if (confirm != 'y' && confirm != 'Y') { printf("Cancelled.\n"); stark_close(&c); return 0; }
        stark_sdo_torque_calib(&c, id, 0);
        printf("Sent. Wait 2s, then check 0x6077 (should read ~0Nm).\n");
        usleep(100000);
        /* 等待主循环处理 */
        usleep(100000);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "mit_migrate") == 0) {
        if (argc < 3) { printf("Usage: mit_migrate <id>\n"); stark_close(&c); return 1; }
        int id = atoi(argv[2]);
        if (id < 1 || id > 2) { printf("mit_migrate: id must be 1 or 2\n"); stark_close(&c); return 1; }
        printf("MIT migrate M%d: write 0x2546=20(Tmax) + save to Flash\n", id);
        printf("Ensure: motor DISABLED. After migration, REBOOT and verify.\n");
        printf("Proceed? (y/N): ");
        char confirm = (char)getchar();
        if (confirm != 'y' && confirm != 'Y') { printf("Cancelled.\n"); stark_close(&c); return 0; }
        stark_sdo_mit_migrate(&c, id);
        printf("Sent. Reboot the driver, then verify with motor_hal_read_mit_scales.\n");
        usleep(100000);
        stark_close(&c);
        return 0;
    }
    if (strcmp(mode, "calib") == 0) {
        printf("Zero calibration: SDO disable + set zero on all online motors...\n");
        stark_request_calib(&c);
        /* 主循环异步处理 (失能 + 下发零位), 等一段让 SDO 完成 */
        usleep(600000);
        printf("Calibration triggered.\n");
        stark_close(&c);
        return 0;
    }

    /* LED 灯控制 */
    if (strcmp(mode, "btn") == 0) {
        uint8_t  st = stark_btn_state(&c);
        uint32_t sq = stark_btn_seq(&c);
        printf("BTN report: state=%u (%s) seq=%u\n",
               st, st ? "按下" : "松开", sq);
        stark_close(&c);
        return 0;
    }

    /* LED 灯控制 */
    if (strcmp(mode, "led") == 0) {
        if (argc < 4) {
            printf("usage: led <motor_id> <mask> <mode> [r] [g] [b]\n");
            printf("  mask: 0x10=LED1 0x20=LED2 0x40=LED3 0x80=LED4 0xF0=ALL 0=off\n");
            printf("  mode: 0=常亮 1=闪烁 2=呼吸 3=流水\n");
            stark_close(&c); return 1;
        }
        int id   = atoi(argv[2]);
        int mask = (int)strtol(argv[3], NULL, 0);
        int mode_led = (argc >= 5) ? atoi(argv[4]) : 0;
        int r = (argc >= 6) ? atoi(argv[5]) : 0;
        int g = (argc >= 7) ? atoi(argv[6]) : 0;
        int b = (argc >= 8) ? atoi(argv[7]) : 0;
        stark_led_ctrl(&c, id, (uint8_t)mask, (uint8_t)mode_led,
                       (uint8_t)r, (uint8_t)g, (uint8_t)b);
        printf("LED M%d: mask=0x%02X mode=%d R=%d G=%d B=%d\n",
               id, mask, mode_led, r, g, b);
        usleep(100000);
        stark_close(&c);
        return 0;
    }

    /* ================================================================
     * SDO / PDO 单帧控制 (sdo cur/pos/vel, pdo cur/pos/vel)
     * 支持 4 种模式: 单电机 / 双电机同值 / 双电机不同值
     * ================================================================ */
    if (strcmp(mode, "sdo") == 0 || strcmp(mode, "pdo") == 0) {
        int is_pdo = (mode[0] == 'p');
        if (argc < 4) {
            printf("ERR: usage: %s cur/pos/vel <id> <val> [...]\n", mode);
            stark_close(&c); return 1;
        }
        const char* sub = argv[2];

        /* 解析电机 ID: argv[3]=id1, 若 argv[4] 为合法 ID 则双电机 */
        int n_motor = (int)c.shm->motor_count;
        int id1 = atoi(argv[3]);
        int id2 = 0, dual = 0;
        int val_idx = 4;
        if (argc > 5) {
            int m = atoi(argv[4]);
            if ((m >= 1 && m <= n_motor) && m != id1) {
                id2 = m;
                dual = 1;
                val_idx = 5;
            }
        }
        if (id1 < 1 || id1 > n_motor) { printf("ERR: invalid motor id=%d\n", id1); stark_close(&c); return 1; }

        if (strcmp(sub, "cur") == 0) {
            if (argc < val_idx + 1) { printf("ERR: need mA value\n"); stark_close(&c); return 1; }
            int ma1 = atoi(argv[val_idx]);
            int ma2 = (dual && argc >= val_idx + 2) ? atoi(argv[val_idx + 1]) : ma1;

            motor_data_t b1 = stark_fb(&c, id1);
            motor_data_t b2; if (dual) b2 = stark_fb(&c, id2);

            uint64_t t0 = now_us();
            if (is_pdo) {
                stark_torque(&c, id1, ma1);
                if (dual) stark_torque(&c, id2, ma2);
                printf("PDO cur: M%d=%dmA", id1, ma1);
            } else {
                stark_sdo_cur(&c, id1, ma1);
                if (dual) stark_sdo_cur(&c, id2, ma2);
                printf("SDO cur: M%d=%dmA", id1, ma1);
            }
            if (dual) printf(" M%d=%dmA", id2, ma2);
            printf("\n");

            motor_data_t a1, a2;
            int ch1 = _wait_new_fb_frame(&c, id1, &a1,
                                          is_pdo ? 200 : 500, is_pdo ? 500 : 5000,
                                          1);
            uint64_t t1 = now_us();
            const char *rt = c.shm->rt_mode ? "RT" : "NRT";
            printf("  M%d: I %d : %dmA (cmd=%dmA) [%s %s]  %luus\n",
                   id1, b1.current_iq, a1.current_iq, ma1,
                   rt, ch1 ? "new_frame" : "timeout", t1 - t0);
            if (dual) {
                int ch2 = _wait_new_fb_frame(&c, id2, &a2,
                                              is_pdo ? 200 : 500, is_pdo ? 500 : 5000,
                                              1);
                uint64_t t2 = now_us();
                printf("  M%d: I %d : %dmA (cmd=%dmA) [%s %s]  %luus\n",
                       id2, b2.current_iq, a2.current_iq, ma2,
                       rt, ch2 ? "new_frame" : "timeout", t2 - t0);
            }
            stark_close(&c); return 0;
        }

        if (strcmp(sub, "pos") == 0) {
            if (argc < val_idx + 1) { printf("ERR: need deg value\n"); stark_close(&c); return 1; }
            float deg1 = (float)atof(argv[val_idx]);
            float deg2 = (dual && argc >= val_idx + 2 && (atof(argv[val_idx + 1]) != 0.0f || argv[val_idx + 1][0] == '0'))
                         ? (float)atof(argv[val_idx + 1]) : deg1;
            /* dual diff 时有 2 个值后可能的 acc/vel */
            int opt_idx = dual ? val_idx + 2 : val_idx + 1;
            float acc = (argc > opt_idx) ? (float)atof(argv[opt_idx]) : 500.0f;
            float vel = (argc > opt_idx + 1) ? (float)atof(argv[opt_idx + 1]) : 10.0f;

            motor_data_t b1 = stark_fb(&c, id1);
            motor_data_t b2; if (dual) b2 = stark_fb(&c, id2);

            uint64_t t0 = now_us();
            if (is_pdo) {
                stark_pp(&c, id1, deg1, acc, vel);
                if (dual) stark_pp(&c, id2, deg2, acc, vel);
                printf("PDO pos (PP): M%d=%.2f°", id1, deg1);
            } else {
                stark_sdo_pos(&c, id1, deg1, acc, vel);
                if (dual) stark_sdo_pos(&c, id2, deg2, acc, vel);
                printf("SDO pos (PP): M%d=%.2f°", id1, deg1);
            }
            if (dual) printf(" M%d=%.2f°", id2, deg2);
            printf(" accel=%.0f vel=%.0f\n", acc, vel);

            motor_data_t a1, a2;
            int ch1 = _wait_new_fb_frame(&c, id1, &a1,
                                          is_pdo ? 1000 : 2000, is_pdo ? 500 : 5000,
                                          1);
            uint64_t t1 = now_us();
            const char *rt = c.shm->rt_mode ? "RT" : "NRT";
            printf("  M%d: pos %.1f : %.1f° (cmd=%.1f°) [%s %s]  %luus\n",
                   id1, counts_to_deg(b1.position), counts_to_deg(a1.position),
                   deg1, rt, ch1 ? "new_frame" : "timeout", t1 - t0);
            if (dual) {
                int ch2 = _wait_new_fb_frame(&c, id2, &a2,
                                              is_pdo ? 1000 : 2000, is_pdo ? 500 : 5000,
                                              1);
                uint64_t t2 = now_us();
                printf("  M%d: pos %.1f : %.1f° (cmd=%.1f°) [%s %s]  %luus\n",
                       id2, counts_to_deg(b2.position), counts_to_deg(a2.position),
                       deg2, rt, ch2 ? "new_frame" : "timeout", t2 - t0);
            }
            stark_close(&c); return 0;
        }

        if (strcmp(sub, "vel") == 0) {
            if (argc < val_idx + 1) { printf("ERR: need rpm value\n"); stark_close(&c); return 1; }
            int rpm1 = atoi(argv[val_idx]);
            int rpm2 = (dual && argc >= val_idx + 2) ? atoi(argv[val_idx + 1]) : rpm1;
            int opt_idx = dual ? val_idx + 2 : val_idx + 1;
            int acc = (argc > opt_idx) ? atoi(argv[opt_idx]) : 500;

            motor_data_t b1 = stark_fb(&c, id1);
            motor_data_t b2; if (dual) b2 = stark_fb(&c, id2);

            uint64_t t0 = now_us();
            if (is_pdo) {
                stark_pv(&c, id1, (float)rpm1, (float)acc);
                if (dual) stark_pv(&c, id2, (float)rpm2, (float)acc);
                printf("PDO vel (PV): M%d=%dRPM", id1, rpm1);
            } else {
                stark_sdo_vel(&c, id1, rpm1, acc);
                if (dual) stark_sdo_vel(&c, id2, rpm2, acc);
                printf("SDO vel (PV): M%d=%dRPM", id1, rpm1);
            }
            if (dual) printf(" M%d=%dRPM", id2, rpm2);
            printf(" accel=%d\n", acc);

            motor_data_t a1, a2;
            int ch1 = _wait_new_fb_frame(&c, id1, &a1,
                                          is_pdo ? 1000 : 2000, is_pdo ? 500 : 5000,
                                          1);
            uint64_t t1 = now_us();
            const char *rt = c.shm->rt_mode ? "RT" : "NRT";
            printf("  M%d: vel %d : %dRPM (cmd=%dRPM) [%s %s]  %luus\n",
                   id1, b1.velocity, a1.velocity, rpm1,
                   rt, ch1 ? "new_frame" : "timeout", t1 - t0);
            if (dual) {
                int ch2 = _wait_new_fb_frame(&c, id2, &a2,
                                              is_pdo ? 1000 : 2000, is_pdo ? 500 : 5000,
                                              1);
                uint64_t t2 = now_us();
                printf("  M%d: vel %d : %dRPM (cmd=%dRPM) [%s %s]  %luus\n",
                       id2, b2.velocity, a2.velocity, rpm2,
                       rt, ch2 ? "new_frame" : "timeout", t2 - t0);
            }
            stark_close(&c); return 0;
        }

        if (strcmp(sub, "tq") == 0) {
            /* PDO 力矩环控制: pdo tq <id> <val> 或 pdo tq <id1> <id2> <val> */
            int n_motor = (int)c.shm->motor_count;
            int id1 = atoi(argv[3]);
            int id2 = 0, dual = 0, val_idx = 4;
            if (argc > 5) {
                int m = atoi(argv[4]);
                if ((m >= 1 && m <= n_motor) && m != id1) { id2 = m; dual = 1; val_idx = 5; }
            }
            if (argc < val_idx + 1) { printf("ERR: pdo tq <id> <val0.05N.m>\n"); stark_close(&c); return 1; }
            int val = atoi(argv[val_idx]);
            uint64_t t0 = now_us();
            stark_torque_ctrl(&c, id1, val);
            printf("PDO tq: M%d=%d(%.1fN.m)", id1, val, (float)val * 0.05f);
            if (dual) { stark_torque_ctrl(&c, id2, val); printf(" M%d=%d", id2, val); }
            printf("\n");

            motor_data_t a1;
            int ch1 = _wait_new_fb_frame(&c, id1, &a1, 1000, 500, 1);
            printf("  M%d: tq=%.2fN.m Iq=%dmA [%s] %luus\n",
                   id1, (float)a1.torque_nm * 0.05f, a1.current_iq,
                   ch1 ? "ok" : "timeout", (unsigned long)(now_us() - t0));
            if (dual) {
                motor_data_t a2;
                int ch2 = _wait_new_fb_frame(&c, id2, &a2, 1000, 500, 1);
                printf("  M%d: tq=%.2fN.m Iq=%dmA [%s] %luus\n",
                       id2, (float)a2.torque_nm * 0.05f, a2.current_iq,
                       ch2 ? "ok" : "timeout", (unsigned long)(now_us() - t0));
            }
            stark_close(&c); return 0;
        }

        if (strcmp(sub, "mit") == 0) {
            /* PDO MIT 单帧控制: pdo mit <id> <pos> <vel> <kp> <kd> <tq>
               双电机: pdo mit <id1> <id2> <pos> <vel> <kp> <kd> <tq> */
            int n_motor = (int)c.shm->motor_count;
            int has_dual = 0, id1, id2;
            {
                id1 = atoi(argv[3]);
                int v = atoi(argv[4]);
                if ((v >= 1 && v <= n_motor) && v != id1) {
                    has_dual = 1; id2 = v;
                }
            }
            int off = has_dual ? 5 : 4;
            if (argc < off + 4) {
                printf("ERR: pdo mit <id> <pos_deg> <vel_rpm> <kp> <kd> [tq_Nm]\n");
                stark_close(&c); return 1;
            }
            float p = (float)atof(argv[off]);
            float v = (float)atof(argv[off+1]);
            float kp= (float)atof(argv[off+2]);
            float kd= (float)atof(argv[off+3]);
            float tq= (argc >= off+5) ? (float)atof(argv[off+4]) : 0.0f;
            uint64_t t0 = now_us();

            stark_mit(&c, id1, p, v, kp, kd, tq);
            printf("PDO mit: M%d pos=%.1f vel=%.1f kp=%.1f kd=%.1f tq=%.1f",
                   id1, p, v, kp, kd, tq);
            if (has_dual) {
                stark_mit(&c, id2, p, v, kp, kd, tq);
                printf(" M%d pos=%.1f vel=%.1f", id2, p, v);
            }
            printf("\n");

            /* 等反馈 */
            motor_data_t a1;
            int ch1 = _wait_new_fb_frame(&c, id1, &a1, 1000, 500, 1);
            uint64_t t1 = now_us();
            const char *rt = c.shm->rt_mode ? "RT" : "NRT";
            printf("  M%d: pos=%.1f cur=%dmA [%s %s]  %luus\n",
                   id1, counts_to_deg(a1.position), a1.current_iq,
                   rt, ch1 ? "new_frame" : "timeout", t1 - t0);
            if (has_dual) {
                motor_data_t a2;
                int ch2 = _wait_new_fb_frame(&c, id2, &a2, 1000, 500, 1);
                printf("  M%d: pos=%.1f cur=%dmA [%s %s]  %luus\n",
                       id2, counts_to_deg(a2.position), a2.current_iq,
                       rt, ch2 ? "new_frame" : "timeout", (uint64_t)(now_us() - t0));
            }
            stark_close(&c); return 0;
        }

        printf("ERR: unknown %s sub-command: %s\n", mode, sub);
        stark_close(&c); return 1;
    }


    /* 分发 PDO 连续控制模式 */
    if (strcmp(mode, "torque") == 0) {
        if (argc < 3) { printf("ERR: 需要指定电流 mA\n"); stark_close(&c); return 1; }
        int32_t ma = atoi(argv[2]);
        run_torque(&c, ma);

    } else if (strcmp(mode, "speed") == 0 || strcmp(mode, "csv") == 0) {
        if (argc < 3) { printf("ERR: 需要指定速度 rpm\n"); stark_close(&c); return 1; }
        float rpm = (float)atof(argv[2]);
        run_speed(&c, rpm);

    } else if (strcmp(mode, "pos") == 0 || strcmp(mode, "csp") == 0) {
        if (argc < 3) { printf("ERR: 需要指定角度 deg\n"); stark_close(&c); return 1; }
        float deg = (float)atof(argv[2]);
        run_position(&c, deg);

    } else if (strcmp(mode, "pp") == 0) {
        if (argc < 3) { printf("ERR: 需要指定角度 deg\n"); stark_close(&c); return 1; }
        float deg   = (float)atof(argv[2]);
        float acc   = (argc >= 4) ? (float)atof(argv[3]) : 2000.0f;
        float vel   = (argc >= 5) ? (float)atof(argv[4]) : 10.0f;
        run_pp(&c, deg, acc, vel);

    } else if (strcmp(mode, "pv") == 0) {
        if (argc < 3) { printf("ERR: 需要指定速度 rpm\n"); stark_close(&c); return 1; }
        float rpm  = (float)atof(argv[2]);
        float acc  = (argc >= 4) ? (float)atof(argv[3]) : 1000.0f;
        run_pv(&c, rpm, acc);

    } else if (strcmp(mode, "mit") == 0) {
        if (argc < 4) { printf("ERR: 需要 kp kd\n"); stark_close(&c); return 1; }
        float kp       = (float)atof(argv[2]);
        float kd       = (float)atof(argv[3]);
        float pos_deg  = (argc >= 5) ? (float)atof(argv[4]) : 0.0f;
        float vel_rpm  = (argc >= 6) ? (float)atof(argv[5]) : 0.0f;
        float torque_nm = (argc >= 7) ? (float)atof(argv[6]) : 0.0f;
        run_mit(&c, kp, kd, pos_deg, vel_rpm, torque_nm);

    } else if (strcmp(mode, "mit_multi") == 0) {
        if (argc < 4) { printf("ERR: 需要 kp kd\n"); stark_close(&c); return 1; }
        float kp_m      = (float)atof(argv[2]);
        float kd_m      = (float)atof(argv[3]);
        float pos_deg_m = (argc >= 5) ? (float)atof(argv[4]) : 0.0f;
        float vel_rpm_m = (argc >= 6) ? (float)atof(argv[5]) : 0.0f;
        float tq_nm_m   = (argc >= 7) ? (float)atof(argv[6]) : 0.0f;
        run_mit_multi(&c, kp_m, kd_m, pos_deg_m, vel_rpm_m, tq_nm_m);

    } else if (strcmp(mode, "multi_cur") == 0) {
        if (argc < 4) { printf("ERR: multi_cur <mA1> <mA2>\n"); stark_close(&c); return 1; }
        run_multi_ctrl(&c, STARK_MODE_CURRENT, (float)atof(argv[2]), (float)atof(argv[3]));
    } else if (strcmp(mode, "multi_csv") == 0) {
        if (argc < 4) { printf("ERR: multi_csv <rpm1> <rpm2>\n"); stark_close(&c); return 1; }
        run_multi_ctrl(&c, STARK_MODE_CSV, (float)atof(argv[2]), (float)atof(argv[3]));
    } else if (strcmp(mode, "multi_csp") == 0) {
        if (argc < 4) { printf("ERR: multi_csp <deg1> <deg2>\n"); stark_close(&c); return 1; }
        run_multi_ctrl(&c, STARK_MODE_CSP, (float)atof(argv[2]), (float)atof(argv[3]));
    } else if (strcmp(mode, "multi_tq") == 0) {
        if (argc < 4) { printf("ERR: multi_tq <val1> <val2> (0.05N.m)\n"); stark_close(&c); return 1; }
        run_multi_ctrl(&c, STARK_MODE_TORQUE, (float)atof(argv[2]), (float)atof(argv[3]));

    } else if (strcmp(mode, "torque_ctrl") == 0) {
        if (argc < 3) { printf("ERR: 需要力矩值 (0.05N.m单位)\n"); stark_close(&c); return 1; }
        int32_t val = atoi(argv[2]);
        run_torque_ctrl(&c, val);

    } else {
        printf("未知模式: %s\n", mode);
        usage();
    }

    printf("\n[done] 停止, 失能电机...\n");
    if (stark_online(&c, 1)) stark_estop(&c, 1);
    if (stark_online(&c, 2)) stark_estop(&c, 2);
    usleep(10000);
    stark_trace_close(&g_trace);
    stark_close(&c);
    return 0;
}
