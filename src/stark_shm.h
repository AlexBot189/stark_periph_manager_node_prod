/*
 * stark_shm.h — 共享内存布局
 * 版本: V2.3 | 日期: 2026-07-13 | 维护: zhiqiang.yang
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 数据方向: motor_node ,  fb_buffer ,  算法/ROS/Web
 *           算法 ,  mailbox ,  motor_node
 *
 * 并发: 单写多读, 无锁.
 *   fb_buffer: 双 Buffer, RT 线程写, 多读者
 *   mailbox:   算法写, RT 读 (环形缓冲, SPSC 无锁)
 *   status:    motor_node 写, 算法只读
 */
#pragma once

#include <stdint.h>

/* LED 配置 */
#ifndef LED_CONFIG_T_DEFINED
#define LED_CONFIG_T_DEFINED
typedef struct {
    uint8_t enable_mask;
    uint8_t mode;
    uint8_t r, g, b;
} led_config_t;
#endif

#define STARK_SHM_NAME    "/stark_shm"
#define STARK_SHM_SIZE    (64 * 1024)
#define STARK_MAX_MOTORS  2               /* SHM 数组最大维度, 运行时 motor_count ≤ 此值 */

#ifdef __cplusplus
extern "C" {
#endif

/* 电机反馈 (CAN 反馈帧 0x300, 大端) */
typedef struct {
    int16_t  position;          /* 编码器角度, counts [-32768,32767] ,  [-180°,180°]   */
    int16_t  velocity;          /* 转速, RPM                                          */
    int16_t  current_iq;        /* Q轴电流, mA                                        */
    int16_t  temperature;       /* 温度, 0.1°C                                       */
    uint8_t  status_byte;       /* bit7:使能 bit6:抱闸 bit5:错误 bit4:到位             */
    uint8_t  mode;              /* 当前控制模式 (CiA 402)                              */
    uint16_t error_code;        /* 故障码, 与 motor_feedback_t 类型一致                  */
    /* V2 扩展 (0x300 DLC=16) */
    int16_t  torque_nm;         /* 力矩反馈, 0.05N.m (Byte[10-11]) */
} motor_data_t;

/* 传感器透传 (CAN 0x680 帧, 小端 bit-packed) */
typedef struct {
    uint16_t hall_adc0;         /* 线性霍尔 A, 0~4095                                */
    uint16_t hall_adc1;         /* 线性霍尔 B, 0~4095                                */
    uint16_t hall_adc2;         /* 线性霍尔 C, 0~4095                                */
    uint16_t force_raw;         /* DF181 力矩, 0~16383                              */
    uint16_t knee_hall;          /* 膝关节霍尔, 0~4095                              */
    uint8_t  key_landing;       /* 着地开关, 0=低 1=高                               */
    uint8_t  data_valid;        /* 力矩数据有效标志                                    */
} stark_sensor_data_t;

/* IMU 数据 (ICM45608, 9轴融合) */
typedef struct {
    /* 原始传感器数据 */
    float    roll;              /* 横滚角, °                                         */
    float    pitch;             /* 俯仰角, °                                         */
    float    yaw;               /* 偏航角, °                                         */
    float    acc_x;             /* 校准加速度 X, g                                    */
    float    acc_y;             /* 校准加速度 Y, g                                    */
    float    acc_z;             /* 校准加速度 Z, g                                    */
    float    gyro_x;            /* 校准角速度 X, dps                                  */
    float    gyro_y;            /* 校准角速度 Y, dps                                  */
    float    gyro_z;            /* 校准角速度 Z, dps                                  */
    /* 9轴融合输出 */
    float    quat_w;            /* 9轴四元数 W                                       */
    float    quat_x;            /* 9轴四元数 X                                       */
    float    quat_y;            /* 9轴四元数 Y                                       */
    float    quat_z;            /* 9轴四元数 Z                                       */
    float    mag_x;             /* 校准磁力计 X, uT                                   */
    float    mag_y;             /* 校准磁力计 Y, uT                                   */
    float    mag_z;             /* 校准磁力计 Z, uT                                   */
    float    heading_deg;       /* 航向角, °                                          */
    float    temp_c;            /* IMU 芯片温度, °C                                   */
    int      stationary;        /* 静止检测: 0=运动, 2=静止                            */
    int      gyr_accuracy;      /* 陀螺校准精度: 0=未校准, 3=精校准                     */
    int      mag_accuracy;      /* 磁力计校准精度: 0=未校准, 3=精校准                   */
    uint64_t timestamp_us;      /* 融合输出时刻, μs                                    */
} imu_data_t;

/* 气压计数据 (QMP6990) */
typedef struct {
    float    pressure_hpa;      /* 气压, hPa                                         */
    float    temperature_c;     /* 温度, °C                                          */
    float    altitude_m;        /* 估算海拔, m                                        */
    uint64_t timestamp_us;      /* 采样时刻, μs                                       */
} barometer_data_t;

/* 足底压力数据 */
typedef struct {
    uint16_t adc[3];            /* AD值, 0~4095: 前掌/足弓/后跟 */
} foot_pad_t;

typedef struct {
    foot_pad_t  left;           /* 左脚 */
    foot_pad_t  right;          /* 右脚 */
    uint32_t    update_cycle;   /* RT周期号, 比对防重复 */
    uint64_t    timestamp_us;   /* 串口收到该帧的时刻 (CLOCK_MONOTONIC) */
} foot_pressure_data_t;

/* 反馈帧 — 写入 SHM 双 Buffer */
typedef struct {
    motor_data_t      motor[STARK_MAX_MOTORS];   /* [0]=右髋(ID=1) [1]=左髋(ID=2)       */
    stark_sensor_data_t sensor[STARK_MAX_MOTORS];  /* 传感器透传, 与电机一一对应              */
    imu_data_t        imu;                      /* IMU 姿态数据                          */
    barometer_data_t  baro;                     /* 气压计数据                            */

    /* 端到端延迟追踪 (μs) */
    uint64_t ts_can_rx;          /* T0: CANFD 反馈帧到达 */
    uint64_t ts_cache_write;     /* T1: fb_cache 写入完成 */
    uint64_t ts_shm_read;        /* T2: RT 线程读 fb_cache */
    uint64_t ts_shm_write;       /* T3: SHM 双 Buffer 切换前 */
    uint64_t ts_algo_read;       /* T4: 算法读 SHM (算法填充) */
    uint64_t ts_algo_done;       /* T5: 算法计算完成 (算法填充) */
    uint64_t ts_mailbox_read;    /* T7: RT 读 mailbox (RT 填充) */
    uint64_t ts_pdo_sent;        /* T8: PDO 发送完成 (RT 填充) */
    uint64_t ts_frame_assembly;  /* 反馈帧组装完成 */

    uint64_t timestamp_us;       /* 组装时刻, μs                                        */

    foot_pressure_data_t foot_pressure;   /* 足底压力数据 */
    uint64_t             ts_foot_rx;      /* 串口收到足底压力帧的时刻 */
} feedback_frame_t;

/* 电机控制命令 */
typedef enum {
    STARK_CMD_TORQUE   = 1,       /* 电流控制 (CURRENT mode), value=mA                         */
    STARK_CMD_SPEED    = 2,       /* 速度模式, value=RPM×100                              */
    STARK_CMD_POS      = 3,       /* 位置模式, value=°×100                                */
    STARK_CMD_MIT      = 4,       /* MIT 阻抗控制 (0x110 独立帧)                             */
    STARK_CMD_PP       = 5,       /* 轮廓位置模式 PP                                       */
    STARK_CMD_CSV      = 6,       /* 循环同步速度 CSV, value=RPM×100                          */
    /* 多轴广播: 两电机 cmd 都为 MULTI 时, 打包一帧 64B CANFD 发出 */
    STARK_CMD_MULTI    = 7,       /* 多轴广播, mode/value/value2/feedforward 字段有效            */
    STARK_CMD_PV       = 8,       /* 轮廓速度模式 PV, value=RPM×100 value2=accel×100         */
    STARK_CMD_MIT_MULTI = 9,       /* MIT 多轴广播 (0x210, 64Byte), mit_* 字段有效               */
    /* PDO Byte0 位控制 (不发 target) */
    STARK_CMD_ENABLE   = 10,      /* PDO使能 (Byte0 bit7=1)                                */
    STARK_CMD_DISABLE  = 11,      /* PDO失能 (Byte0 bit7=0)                                */
    STARK_CMD_ESTOP    = 12,      /* 急停: enable=0 + bus=OFF */
    STARK_CMD_RECOVER  = 13,      /* 恢复: enable=1 + bus=ON */
    STARK_CMD_SET_MODE = 14,      /* 切换 PDO 控制模式 */
    /* PDO Byte0 bit5 清错 */
    STARK_CMD_CLEAR_FAULT = 15,
    /* 力矩环控制 (V2 新增, 0x100+mode=6), value=0.05N.m */
    STARK_CMD_TORQUE_CTRL = 16,

    /* SDO 控制命令 (算法写, 主循环处理, 非 RT) */
    STARK_CMD_SDO_CUR         = 20,      /* SDO 电流, value=mA                                    */
    STARK_CMD_SDO_POS         = 21,      /* SDO 轮廓位置(PP), value=deg×100 value2=accel ff=vel  */
    STARK_CMD_SDO_VEL         = 22,      /* SDO 轮廓速度(PV), value=RPM×100 value2=accel          */
    STARK_CMD_SDO_TORQUE_CALIB = 23,     /* SDO 力矩标定, value=torque_mNm                      */
    STARK_CMD_SDO_MIT_MIGRATE  = 24,     /* SDO MIT缩放迁移, value=0x2546值 (默认20)               */
} stark_cmd_type_t;

typedef struct {
    uint8_t  motor_id;          /* 1=右髋 2=左髋                                       */
    uint8_t  cmd;               /* stark_cmd_type_t                                       */
    int32_t  value;             /* target1: 主目标值 (位置/速度/电流)                     */
    int32_t  value2;            /* target2: 加减速(PP/PV模式, RPM/s)                     */
    int32_t  feedforward;       /* 前馈 (PP模式轮廓速度 RPM, 其他模式填0)                */
    /* MIT 模式专用 */
    uint16_t mit_pos;           /* 目标位置 [0-65535]                                  */
    int16_t  mit_vel;           /* 目标速度 RPM (V1 直接存储)                          */
    uint16_t mit_kp;            /* 位置刚度 [0-4095]                                   */
    uint16_t mit_kd;            /* 速度阻尼 [0-4095]                                   */
    int16_t  mit_torque;        /* 前馈力矩 Nm (V1 直接存储)                            */
    uint8_t  multi_mode;        /* MULTI 模式 (PP=1,PV=2,CSP=3,CSV=4,CUR=5,TQ=7)      */
    uint8_t  _pad[5];           /* 对齐到 offset 32, sizeof=40 */
    uint64_t timestamp_us;      /* 算法下发时刻                                          */
} motor_command_t;

/* Mailbox 环形缓冲 — 算法写, RT 读, SPSC 无锁, 不丢帧 */
#define STARK_MBOX_DEPTH 8

/* SDO 控制命令槽 (算法写, 主循环处理) */
typedef struct {
    uint8_t  motor_id;
    uint8_t  cmd;               /* stark_cmd_type_t (SDO 命令) */
    int32_t  value;
    int32_t  value2;
    int32_t  feedforward;
} sdo_cmd_slot_t;

typedef struct {
    motor_command_t cmd[STARK_MAX_MOTORS];
} mailbox_frame_t;

typedef struct {
    volatile uint64_t seq_write;   /* 算法写入游标, 单调递增 */
    volatile uint64_t seq_read;    /* RT 消费游标, 单调递增 */
    mailbox_frame_t   frames[STARK_MBOX_DEPTH];
    uint8_t           _pad_mbox[16];
} stark_mailbox_t;

/* 故障级别 */
typedef enum {
    MOTOR_OK    = 0,            /* 正常运行                                            */
    MOTOR_WARN  = 1,            /* 降额: torque=0, 保持使能 (可自动恢复)                 */
    MOTOR_FAULT = 2,            /* 停机: DS402 Shutdown (需人工干预)                    */
} motor_severity_t;

/* 故障原因 */
typedef enum {
    FAULT_NONE             = 0,
    FAULT_CMD_STALL,            /* 输出停滞 (同一 cmd 重复 500ms)                       */
    FAULT_CAN_OFFLINE,          /* CAN 断线 (2s 无帧)                                  */
    FAULT_ENCODER_FAULT,        /* 编码器异常 (position 恒定 3s)                         */
    FAULT_OVERTEMP,             /* 驱动器过温 (> 80°C)                                 */
    FAULT_CALIB_TIMEOUT,        /* 校准超时                                             */
} fault_reason_t;

/* 节点状态机 — 精简4状态 */
typedef enum {
    STATE_BOOTING = 0,
    STATE_READY   = 1,
    STATE_RUNNING = 2,
    STATE_FAULT   = 3,
} stark_state_t;

#define STARK_STATE_COUNT  4

/* ================================================================
 * 周期上报数据 — IMU + 双电机 + 传感器透传 统一结构
 * ================================================================ */

typedef struct {
    /* IMU 段 */
    float    gyro_dps_x;       /* 角速度 X, dps                              */
    float    gyro_dps_y;       /* 角速度 Y, dps                              */
    float    gyro_dps_z;       /* 角速度 Z, dps                              */
    float    quat_w;           /* 四元数 W                                   */
    float    quat_x;           /* 四元数 X                                   */
    float    quat_y;           /* 四元数 Y                                   */
    float    quat_z;           /* 四元数 Z                                   */
    float    gyro_roll;        /* 横滚角, -180~+180                          */
    float    gyro_pitch;       /* 俯仰角, -90~+90                            */
    float    gyro_yaw;         /* 偏航角, -180~+180                          */
    float    acc_x;            /* 加速度 X, g                                */
    float    acc_y;            /* 加速度 Y, g                                */
    float    acc_z;            /* 加速度 Z, g                                */
    float    air_pressure;     /* 气压, hPa (预留)                           */

    /* 右电机 (ID=1) */
    int32_t  RealtimeVelocity;      /* 实时转速, RPM×10                        */
    int16_t  motor_abs_angle;       /* 绝对角度, °×10                          */
    int16_t  cal_Iq_current;        /* Q轴电流, A×100                          */
    int16_t  cal_bus_current;       /* 母线电流, A×100 (预留, SDO轮询)           */
    int16_t  temp_data;             /* 占位符                                    */
    int32_t  motor_temp;            /* 电机温度, °C×100                         */
    int16_t  fault_code;            /* 故障码                                    */
    int16_t  motor_state;           /* 电机状态 (status_byte 零扩展)             */
    uint16_t hall_a_data;           /* 霍尔传感器 A                              */
    uint16_t hall_b_data;           /* 霍尔传感器 B                              */
    uint16_t hall_c_data;           /* 霍尔传感器 C                              */
    uint16_t df181_torque;          /* DF181 力矩                                */
    int16_t  knee_hall;            /* 膝关节霍尔                              */
    uint8_t  key_landing;           /* 着地开关                                  */
    uint8_t  torque_valid;          /* 力矩数据有效                              */
    int16_t  torque_feedback;       /* 驱动力矩反馈, 0.05N.m (0x300 Byte[10-11])   */

    /* 左电机 (ID=2) */
    int32_t  RealtimeVelocity_left;
    int16_t  motor_abs_angle_left;
    int16_t  cal_Iq_current_left;
    int16_t  cal_bus_current_left;
    int16_t  temp_data_left;
    int32_t  motor_temp_left;
    int16_t  fault_code_left;
    int16_t  motor_state_left;
    uint16_t hall_a_data_left;
    uint16_t hall_b_data_left;
    uint16_t hall_c_data_left;
    uint16_t df181_torque_left;
    int16_t  knee_hall_left;
    uint8_t  key_landing_left;
    uint8_t  torque_valid_left;
    int16_t  torque_feedback_left;   /* 驱动力矩反馈, 0.05N.m */

    /* 时间戳 */
    uint32_t timestamp_ms;      /* CLOCK_REALTIME, ms since epoch */

    /* 帧同步: 同一 RT 周期号下所有数据来源一致 */
    uint32_t frame_cycle;       /* RT cycle number, monotonic, 1ms/tick */
    uint32_t motor_ts_us;       /* 0x300 CAN RX timestamp, μs (min of all motors) */
    uint32_t imu_ts_us;         /* IMU GAF output timestamp, μs */
    uint32_t sensor_ts_us;      /* 0x680 CAN RX timestamp, μs */

    /* 0x6B0 力矩 (原始计数/100) */
    float    spi_torque;              /* 右 id=1 */
    uint8_t  spi_valid;               /* 右 valid (byte4.bit0) */
    uint8_t  spi_error;               /* 右 error (byte5) */
    uint8_t  _pad_spi_r[2];
    float    spi_torque_left;         /* 左 id=2 */
    uint8_t  spi_valid_left;
    uint8_t  spi_error_left;
    uint8_t  _pad_spi_l[2];

    foot_pressure_data_t foot_pressure;  /* 足底压力数据 */
} PeriodicUploadData;

/* 共享内存总结构 (64KB) */

typedef struct {
    /* 双 Buffer 反馈区 (motor_node 写, 算法/ROS/Web 读) */
    uint32_t          active_idx;            /* 0 或 1, atomic release/acquire          */
    feedback_frame_t  fb_buffer[2];          /* 双 Buffer 防撕裂                         */

    /* Mailbox 命令区 (算法写, motor_node 读) */
    stark_mailbox_t     mailbox;

    /* 状态区 (motor_node 写, 算法只读) */
    uint8_t   motor_online;               /* bit0=右(1) bit1=左(2)                       */
    uint8_t   calib_state;                /* 0=空闲 1=校准中 2=完成 3=超时                */
    uint8_t   motor_enabled;              /* 每 bit 对应电机使能状态                       */
    uint8_t   motor_severity;             /* 0=OK 1=WARN 2=FAULT                        */
    uint8_t   fault_reason;               /* fault_reason_t 枚举                         */
    uint8_t   node_state;                 /* stark_state_t                                 */
    uint8_t   calib_requested;            /* 算法写: 1=请求复杂校准 (按键/命令触发)        */
    uint8_t   _pad_state[1];              /* 对齐                                         */

    /* 耗时追踪 (μs) */
    /* 反馈路径 */
    uint16_t  fb_read_avg_us;
    uint16_t  fb_read_max_us;
    uint16_t  fb_total_avg_us;       /* T1, T4 反馈总延迟 */
    uint16_t  fb_total_max_us;
    uint16_t  fb_total_min_us;
    uint16_t  fb_age_max_us;        /* 反馈数据年龄 (CAN收帧→RT读) */
    uint16_t  fb_age_avg_us;
    uint16_t  fb_age_min_us;
    /* 控制路径 */
    uint16_t  ctrl_total_avg_us;     /* T5, T6 控制总延迟 */
    uint16_t  ctrl_total_max_us;
    uint16_t  ctrl_total_min_us;
    /* 命令通道 */
    uint16_t  mbox_age_max_us;       /* 算法写 mailbox → RT 读到 */
    uint16_t  mbox_age_avg_us;
    uint16_t  mbox_age_min_us;
    /* 统计 */
    uint32_t  trace_cycle_count;     /* 已采样周期数 */
    uint32_t  shm_write_avg_us;      /* SHM 写入耗时 */
    uint32_t  cycle_overrun_count;   /* 周期超限次数 (uint32_t: 长期压测不回绕) */

    /* 周期上报区 (motor_node 写, 算法/Web 读) */
    uint8_t   periodic_enabled;       /* 上报总开关: 0=关 1=开 */
    uint8_t   rt_mode;               /* 0=SCHED_OTHER 1=SCHED_FIFO */
    uint16_t  period_us;             /* RT 周期 (us) */
    uint8_t   rt_priority;           /* RT线程优先级 (SCHED_FIFO prio) */
    uint8_t   rt_cpu;                /* RT线程主绑核 */
    uint8_t   perf_trace_enabled;    /* 实时性能统计开关 0=关 1=开 */
    uint8_t   perf_reset_request;    /* web/算法写1: 清零本轮max, RT消费后清零 */
    uint32_t  periodic_period_ms;     /* 上报周期 ms, 默认 5 */
    uint32_t  periodic_version;       /* 写入版本号, 递增 */
    PeriodicUploadData periodic_data;  /* 周期上报数据 */
    uint8_t   _pad_rpt[16];           /* 预留扩展 */

    /* 管理命令通道 (每电机独立 slot, 不和算法 mailbox 竞争) */
    volatile uint8_t mgmt_cmd[STARK_MAX_MOTORS];   /* stark_cmd_type_t, 0=idle */
    volatile uint8_t mgmt_seq[STARK_MAX_MOTORS];   /* 写入递增, RT 处理完拷贝到 ack */
    volatile uint8_t mgmt_ack[STARK_MAX_MOTORS];   /* RT 确认 */

    /* SDO 控制通道 (算法写, 主循环处理, 每电机独立 slot) */
    sdo_cmd_slot_t     sdo_cmds[STARK_MAX_MOTORS];
    volatile uint8_t   sdo_seq[STARK_MAX_MOTORS];
    volatile uint8_t   sdo_ack[STARK_MAX_MOTORS];

    /* LED 灯控制 (算法写, 主循环处理, 每电机独立) */
    led_config_t       led_cfg[STARK_MAX_MOTORS];
    volatile uint8_t   led_seq[STARK_MAX_MOTORS];  /* 算法递增, 触发主循环处理 */
    volatile uint8_t   led_ack[STARK_MAX_MOTORS];  /* 主循环确认, =seq 表示已处理 */

    /* 双向心跳 */
    volatile uint32_t algo_heartbeat;              /* 算法递增, RT 检测超时脱使能 */
    uint32_t          algo_heartbeat_timeout_ms;   /* stark_node 启动写入, 默认 1000 */
    volatile uint32_t rt_cycle;                    /* RT 每 1ms 递增, 算法检测存活 */

    /* 按键 B 上报 (GPIO 线程写, 算法只读) */
    volatile uint8_t  btn_report_state;            /* 0=松开 1=按下 */
    volatile uint32_t btn_report_seq;              /* 每次按下递增, 算法比对检测边沿 */

    /* 端到端耗时 (μs) */
    uint16_t  ctrl_e2e_avg_us;       /* 算法下发 → PDO 发出 */
    uint16_t  ctrl_e2e_min_us;
    uint32_t  ctrl_e2e_max_us;       /* 非RT下可能超65535, 用32位 */
    uint16_t  fb_e2e_avg_us;         /* CAN收帧 → SHM写完 */
    uint16_t  fb_e2e_min_us;
    uint32_t  fb_e2e_max_us;         /* 非RT下可能超65535, 用32位 */
    /* RT 周期抖动 (μs) */
    uint16_t  cycle_jitter_avg_us;   /* 实际唤醒 - 理想目标 */
    uint16_t  cycle_jitter_min_us;
    uint32_t  cycle_jitter_max_us;   /* 非RT下可能超65535, 用32位 */

    uint8_t   _pad[3015];
} stark_shm_t;

/* 编译期校验 struct 大小, 字段变更时更新此值 */
/* 编译期校验, ARM64/x86_64 对齐差异需分别适配, 暂时注释 */
#if 0
#ifdef __cplusplus
static_assert(sizeof(stark_shm_t) == 4648, "stark_shm_t size changed, update _pad and this assert");
#else
_Static_assert(sizeof(stark_shm_t) == 4648, "stark_shm_t size changed, update _pad and this assert");
#endif
#endif

#ifdef __cplusplus
}
#endif
