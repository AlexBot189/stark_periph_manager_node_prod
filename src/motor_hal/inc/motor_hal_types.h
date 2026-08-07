/**
 * @file motor_hal_types.h
 * @brief 关节模组 HAL 层 - 类型与常量定义
 *
 * 适配: 无锡巨蟹智能驱动一体化通讯模组
 * 协议: CANopen CiA 402 / CANFD
 * 物理: CANFD 标准帧 11bit, 仲裁段1Mbps, 数据段5Mbps
 */

#ifndef MOTOR_HAL_TYPES_H
#define MOTOR_HAL_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 常量
 * ============================================================================ */

/** 编码器分辨率 */
#define ENCODER_MOTOR_RES       (16384)   /* 电机端: 14bit */
#define ENCODER_LOAD_RES        (65536)   /* 负载端: 16bit */
#define LOAD_POS_MIN            (-32768)
#define LOAD_POS_MAX            (32767)
#define LOAD_ANGLE_MIN          (-180.0f)
#define LOAD_ANGLE_MAX          (180.0f)

/** 最大节点数 */
#define MOTOR_HAL_MAX_MOTORS    (4)   /* 髋关节×2 + 膝关节×2 */

/** 默认超时 (ms) */
#define MOTOR_SDO_TIMEOUT_MS    (200)
#define MOTOR_BOOTUP_TIMEOUT_MS (3000)
#define MOTOR_ENABLE_STEP_MS    (500)

/* ============================================================================
 * CANFD 帧
 * ============================================================================ */

#define CANFD_MAX_DLC (64)

typedef struct {
    uint32_t id;                     /* 11-bit 标准帧 ID */
    uint8_t  dlc;                    /* 数据长度 */
    uint8_t  data[CANFD_MAX_DLC];    /* 数据 */
    bool     is_fd;                  /* FD 帧标志 */
    bool     use_brs;                /* 数据段切换高速率 (CANFD BRS), SDO=false, PDO=true */
} canfd_frame_t;

/* ============================================================================
 * COB-ID 定义
 * ============================================================================ */

#define COB_NMT              (0x000)   /* 网络管理 */
#define COB_SYNC             (0x080)   /* 同步报文 */
#define COB_EMCY_BASE        (0x080)   /* 紧急报文: 0x080 + node_id */
#define COB_SDO_TX_BASE      (0x600)   /* SDO 请求 (主, 从) */
#define COB_SDO_RX_BASE      (0x580)   /* SDO 应答 (从, 主) */
#define COB_BOOTUP_BASE      (0x700)   /* Bootup / Heartbeat */
#define COB_TPDO1_BASE       (0x180)   /* 标准 TPDO1 */
#define COB_RPDO1_BASE       (0x200)   /* 标准 RPDO1 */
#define COB_PDO_CTRL_BASE    (0x100)   /* 自定义单轴 PDO */
#define COB_PDO_MIT_BASE     (0x110)   /* MIT 模式 PDO (单轴) */
#define COB_MULTI_MIT        (0x210)   /* MIT 多轴广播 (64Byte, 最多6电机) */
#define COB_MULTI_CTRL       (0x200)   /* 多轴广播 */
#define COB_FEEDBACK_BASE    (0x300)   /* 反馈帧 */

/* ============================================================================
 * SDO 命令码
 * ============================================================================ */

#define SDO_CC_DOWNLOAD_1B    (0x2F)   /* 写 1 字节 */
#define SDO_CC_DOWNLOAD_2B    (0x2B)   /* 写 2 字节 */
#define SDO_CC_DOWNLOAD_4B    (0x23)   /* 写 4 字节 */
#define SDO_CC_UPLOAD_REQ     (0x40)   /* 读请求 */
#define SDO_CC_UPLOAD_RSP_1B  (0x4F)   /* 响应 1 字节 */
#define SDO_CC_UPLOAD_RSP_2B  (0x4B)   /* 响应 2 字节 */
#define SDO_CC_UPLOAD_RSP_4B  (0x43)   /* 响应 4 字节 */
#define SDO_CC_ABORT          (0x80)   /* 错误响应 */

/** SDO Abort Code (CiA 301, Table 52) */
#define SDO_ABORT_TOGGLE_BIT         (0x05030000UL)  /* Toggle bit not alternated */
#define SDO_ABORT_TIMEOUT            (0x05040000UL)  /* SDO protocol timed out */
#define SDO_ABORT_CMD_UNKNOWN        (0x05040001UL)  /* Command specifier not valid */
#define SDO_ABORT_BLK_SIZE           (0x05040005UL)  /* Block size not valid */
#define SDO_ABORT_UNSUPPORTED        (0x06010000UL)  /* Unsupported access */
#define SDO_ABORT_WRITE_ONLY         (0x06010001UL)  /* Read access to write-only */
#define SDO_ABORT_READ_ONLY          (0x06010002UL)  /* Write access to read-only */
#define SDO_ABORT_NO_OBJECT          (0x06020000UL)  /* Object does not exist */
#define SDO_ABORT_NO_PDO_MAP         (0x06040041UL)  /* Cannot be mapped to PDO */
#define SDO_ABORT_PDO_LEN            (0x06040042UL)  /* PDO length exceeded */
#define SDO_ABORT_PARAM_INCOMPAT     (0x06040043UL)  /* General parameter incompatibility */
#define SDO_ABORT_HW_ERR             (0x06060000UL)  /* Hardware error */
#define SDO_ABORT_TYPE_MISMATCH      (0x06070010UL)  /* Data type mismatch */
#define SDO_ABORT_DATA_LEN           (0x06070012UL)  /* Data length too large */
#define SDO_ABORT_DATA_LEN_SHORT     (0x06070013UL)  /* Data length too short */
#define SDO_ABORT_NO_SUBINDEX        (0x06090011UL)  /* Sub-index does not exist */
#define SDO_ABORT_VALUE_RANGE        (0x06090030UL)  /* Value out of range */
#define SDO_ABORT_VALUE_TOO_HIGH     (0x06090031UL)  /* Value too high */
#define SDO_ABORT_VALUE_TOO_LOW      (0x06090032UL)  /* Value too low */
#define SDO_ABORT_NO_STORE           (0x08000020UL)  /* Cannot store to EEPROM */
#define SDO_ABORT_LOCAL_ONLY         (0x08000023UL)  /* Local control by application */

/** NMT (Heartbeat) 状态码 */
#define NMT_STATE_INITIALISING  (0x00)
#define NMT_STATE_STOPPED       (0x04)
#define NMT_STATE_OPERATIONAL   (0x05)
#define NMT_STATE_PRE_OP        (0x7F)

/** EMCY (Emergency) 错误码 (CiA 301) */
#define EMCY_GENERIC             (0x1000)  /* Generic error */
#define EMCY_CURRENT             (0x2000)  /* Current */
#define EMCY_VOLTAGE             (0x3000)  /* Voltage */
#define EMCY_TEMPERATURE         (0x4000)  /* Temperature */
#define EMCY_HARDWARE            (0x5000)  /* Hardware */
#define EMCY_SOFTWARE            (0x6000)  /* Software internal */
#define EMCY_COMMUNICATION       (0x8000)  /* Communication */
#define EMCY_POSITION_ERROR      (0x8300)  /* Position error */
#define EMCY_CAN_OVERRUN         (0x8110)  /* CAN overrun */
#define EMCY_CAN_PASSIVE         (0x8120)  /* CAN passive mode */
#define EMCY_HEARTBEAT           (0x8130)  /* Heartbeat lost */
#define EMCY_SYNC_TIMEOUT        (0x8140)  /* SYNC timeout */

/* ============================================================================
 * NMT 命令码
 * ============================================================================ */

#define NMT_CMD_START         (0x01)
#define NMT_CMD_STOP          (0x02)
#define NMT_CMD_PRE_OP        (0x80)
#define NMT_CMD_RESET_NODE    (0x81)
#define NMT_CMD_RESET_COMM    (0x82)

/* ============================================================================
 * 控制模式 (Modes of Operation, 0x6060)
 * ============================================================================ */

typedef enum {
    MOTOR_MODE_PROFILE_POS   = 1,  /* PP:  轮廓位置 */
    MOTOR_MODE_PROFILE_VEL   = 2,  /* PV:  轮廓速度 */
    MOTOR_MODE_CSP           = 3,  /* CSP: 循环同步位置 */
    MOTOR_MODE_CSV           = 4,  /* CSV: 循环同步速度 */
    MOTOR_MODE_CURRENT       = 5,  /* 电流环 */
    MOTOR_MODE_MIT           = 6,  /* MIT: 阻抗控制, 走 0x110 独立帧. 仅力矩环对外骨骼有效 */
    MOTOR_MODE_TORQUE        = 7,  /* 力矩环, 走 0x100/0x200+mode=7. 协议定稿确认 反馈=0x7 */
} motor_mode_t;

/* ============================================================================
 * PDO Byte0 — 自定义PDO和MIT PDO共用控制字节
 *
 * Byte0: [7]Enable [6]BUS_ON [5]ClearErr [4:1]Mode [0]Reserved
 *
 * 与 SDO Controlword (0x6040) 不同:
 *   - SDO 走 CANopen 协议, 改 DS402 状态机
 *   - PDO Byte0 走 CANFD 实时帧, 直接控制电机使能/母线/模式
 *   两者独立, 互不耦合
 *
 * bit5 清错: 上层先读 fb->error_code 判断错误类型 (温度/过流/编码器等),
 *   根据具体错误决定是否清除。调用 motor_hal_pdo_clear_fault 置位,
 *   下一帧自动清0 (脉冲语义)。
 *
 * bit6 母线: 控制逆变器供电。当前电机无机械抱闸, 此位预留。
 * ============================================================================ */

#define PDO_BYTE0_ENABLE     (0x80)   /* bit7: PDO使能 (1=响应PDO控制, 0=不响应) */
#define PDO_BYTE0_BUS_ON     (0x40)   /* bit6: 母线电压 (1=接通 0=断开, 预留, 当前电机不实现抱闸) */
#define PDO_BYTE0_CLR_ERR    (0x20)   /* bit5: 清除错误标志 (脉冲, 单帧后自动清0) */
#define PDO_BYTE0_MODE_SHIFT (1)      /* bit4:1 控制模式字段 */
#define PDO_BYTE0_MODE_MASK  (0x1E)   /* 0b00011110 */
#define PDO_BYTE0_RSVD       (0x01)   /* bit0: 预留 */

/* 预设值 */
#define PDO_BYTE0_ESTOP      (0x00)   /* 急停: enable=0 + bus=OFF */

/* 从模式构建 PDO Byte0 mode 字段 */
static inline uint8_t pdo_byte0_mode_part(motor_mode_t m) {
    return ((uint8_t)(m) & 0x0F) << PDO_BYTE0_MODE_SHIFT;
}

/* 构建完整 Byte0 */
static inline uint8_t pdo_byte0_build(bool enable, bool bus_on,
                                       bool clear_err, motor_mode_t mode)
{
    return (enable   ? PDO_BYTE0_ENABLE  : 0)
         | (bus_on   ? PDO_BYTE0_BUS_ON  : 0)
         | (clear_err ? PDO_BYTE0_CLR_ERR : 0)
         | pdo_byte0_mode_part(mode);
}

/* 解析 Byte0 各字段 */
static inline bool pdo_byte0_get_enable(uint8_t b)   { return (b & PDO_BYTE0_ENABLE) != 0; }
static inline bool pdo_byte0_get_bus_on(uint8_t b)   { return (b & PDO_BYTE0_BUS_ON) != 0; }
static inline bool pdo_byte0_get_clr_err(uint8_t b)  { return (b & PDO_BYTE0_CLR_ERR) != 0; }
static inline uint8_t pdo_byte0_get_mode(uint8_t b)  { return (b & PDO_BYTE0_MODE_MASK) >> PDO_BYTE0_MODE_SHIFT; }

/* ============================================================================
 * DS402 状态字 / 控制字
 * ============================================================================ */

/* Statusword (0x6041) bit 定义 */
#define SW_FAULT              (0x0008)
#define SW_VOLTAGE_ENABLED    (0x0010)
#define SW_QUICK_STOP         (0x0020)
#define SW_SWITCH_ON_DISABLED (0x0040)
#define SW_WARNING            (0x0080)
#define SW_TARGET_REACHED     (0x0400)
#define SW_INTERNAL_LIMIT     (0x0800)
#define SW_STATE_MASK         (0x006F)

/* Statusword 组合值 */
#define SW_NOT_READY          (0x0000)
#define SW_ON_DISABLED        (0x0060)
#define SW_READY_TO_SW_ON     (0x0021)
#define SW_SWITCHED_ON        (0x0023)
#define SW_OP_ENABLED         (0x0027)
#define SW_FAULT_STATE        (0x0008)
#define SW_QUICK_STOP_STATE   (0x0007)

/* Controlword (0x6040) 命令值 */
#define CW_SHUTDOWN           (0x0006)
#define CW_SWITCH_ON          (0x0007)
#define CW_DISABLE_VOLTAGE    (0x0000)
#define CW_QUICK_STOP         (0x0002)
#define CW_DISABLE_OP         (0x0007)
#define CW_ENABLE_OP          (0x000F)
#define CW_FAULT_RESET        (0x0080)
#define CW_NEW_SET_POINT      (0x004F)
#define CW_STOP               (0x000F)

/* ============================================================================
 * DS402 状态
 * ============================================================================ */

typedef enum {
    MOTOR_STATE_NOT_READY       = 0,
    MOTOR_STATE_SWITCH_ON_DIS   = 1,
    MOTOR_STATE_READY_TO_SW_ON  = 2,
    MOTOR_STATE_SWITCHED_ON     = 3,
    MOTOR_STATE_OP_ENABLED      = 4,
    MOTOR_STATE_QUICK_STOP      = 5,
    MOTOR_STATE_FAULT           = 6,
    MOTOR_STATE_UNKNOWN         = 99,
} motor_state_t;

static inline const char* motor_state_str(motor_state_t s) {
    switch (s) {
        case MOTOR_STATE_NOT_READY:      return "NOT_READY";
        case MOTOR_STATE_SWITCH_ON_DIS:  return "SWITCH_ON_DISABLED";
        case MOTOR_STATE_READY_TO_SW_ON: return "READY_TO_SWITCH_ON";
        case MOTOR_STATE_SWITCHED_ON:    return "SWITCHED_ON";
        case MOTOR_STATE_OP_ENABLED:     return "OPERATION_ENABLED";
        case MOTOR_STATE_QUICK_STOP:     return "QUICK_STOP";
        case MOTOR_STATE_FAULT:          return "FAULT";
        default: return "UNKNOWN";
    }
}

/* ============================================================================
 * 错误码
 * ============================================================================ */

#define ERR_OVER_VOLTAGE      (0x0001)
#define ERR_UNDER_VOLTAGE     (0x0002)
#define ERR_OVER_TEMP         (0x0004)
#define ERR_STALL             (0x0008)
#define ERR_OVERLOAD          (0x0010)
#define ERR_CURRENT_SAMPLE    (0x0020)
#define ERR_POS_LIMIT         (0x0040)
#define ERR_NEG_LIMIT         (0x0080)
#define ERR_ENCODER_TIMEOUT   (0x0100)
#define ERR_OVER_MAX_SPEED    (0x0200)
#define ERR_ELEC_ANGLE_FAIL   (0x0400)
#define ERR_POS_ERROR_LARGE   (0x1000)
#define ERR_ENCODER_FAULT     (0x2000)

/* ============================================================================
 * 反馈数据
 * ============================================================================ */

typedef struct {
    int16_t  position;          /* 负载端位置 (-32768~32767, 映射 -180°~180°) */
    int16_t  velocity;          /* 电机端速度 (RPM) */
    int16_t  current_iq;        /* Iq 电流 (mA) */
    uint16_t error_code;        /* 错误码 */
    int16_t  temperature;       /* 线圈温度 (0.1°C) */
    uint8_t  mode;              /* 当前控制模式 */
    uint8_t  status_byte;       /* bit7:使能 bit6:抱闸 bit5:错误 bit4:到位 */
    uint64_t timestamp_us;      /* 接收时间戳 */
    /* V2 扩展 (0x300 DLC=16 有力矩传感器版本) */
    int16_t  torque_nm;         /* 力矩反馈, 单位 0.05N.m (Byte[10-11]) */
} motor_feedback_t;

/* 外设传感器透传 COB-ID */
#define COB_SENSOR_BASE       (0x680)   /* 透传数据: 0x680 + node_id */
#define COB_RUN_FB1_BASE      (0x6A0)   /* 运行反馈帧1: Iq/母线电流/温度/错误码 */
#define COB_SPI_TORQUE_BASE   (0x6B0)   /* SPI 力矩帧基址: 0x6B0 + node_id */
#define OD_SENSOR_CONFIG      (0x5503)  /* 透传配置对象 */
#define OD_SENSOR_CONFIG_SUB  (0x04)    /* 透传配置子索引 */
#define OD_LED_CTRL_SUB       (0x06)    /* RGB 灯控制子索引 */

/* 透传力矩来源模块 (配置字 bit20~21) */
typedef enum {
    FORCE_MODULE_CAN = 0,   /* 力矩走 0x680 force_raw */
    FORCE_MODULE_SPI = 1,   /* 力矩走 0x6B0 高精度帧 (默认) */
} force_module_t;

/* 透传配置字位定义 (OD 0x5503:04, 32bit)
 * cfg = period_div | (bus_format<<16) | (mode<<18) | (force_module<<20) */
#define SENSOR_CFG_PERIOD_DIV_SHIFT   (0)    /* [15:0]  period_div, 1tick=0.25ms (V1.2基准) */
#define SENSOR_CFG_BUS_FORMAT_SHIFT   (16)   /* [17:16] bus_format: 3=CANFD BRS 0=Classic (不变) */
#define SENSOR_CFG_MODE_SHIFT         (18)   /* [19:18] mode: 0=兼容 1=运行反馈 2=全部 3=聚合 */
#define SENSOR_MODE_AGGREGATED         (3)    /* mode=3: 32Byte CAN FD 聚合透传 (0x6C0) */
#define SENSOR_CFG_FORCE_MODULE_SHIFT (20)   /* [21:20] force_module: 0=CAN 1=SPI */
#define SENSOR_CFG_FIELD_MASK         (0x3U)
#define SENSOR_MODE_ALL               (2)    /* mode=2: 0x680+0x690+0x6A0+0x6B0 全发 */

/* 聚合透传 COB-ID (mode=3) */
#define COB_SENSOR_AGGR_BASE  (0x6C0)   /* 32Byte CAN FD 聚合帧: 0x6C0+NodeID */

/* 透传数据 (8字节, 小端, bit-packed) */
typedef struct {
    uint16_t hall_adc0;        /* Hall 传感器 ADC0, U12, 0~4095 */
    uint16_t hall_adc1;        /* Hall 传感器 ADC1, U12 */
    uint16_t hall_adc2;        /* Hall 传感器 ADC2, U12 */
    uint16_t force_raw;        /* DF181 力/力矩原始值, U14, 0~16383 */
    uint16_t knee_hall;         /* 膝关节霍尔, U12 */
    uint8_t  hw_sw_pc9;        /* PC9 硬件开关, 0=低 1=高 */
    uint8_t  data_valid;       /* 1=数据有效 (SPI 力矩模式下固定 0) */
    uint64_t timestamp_us;     /* 0x680 接收时间戳 */
    /* --- 尾部追加字段 (保持现有偏移不变) --- */
    int32_t  spi_force_raw_s24; /* 0x6B0 力矩原始计数, 24bit 有符号小端, 未做物理换算 */
    uint8_t  spi_valid;         /* 0x6B0 数据有效标志 */
    uint8_t  spi_error;         /* 0x6B0 错误码 (10=传感器校验失败等) */
    uint8_t  _pad_spi[2];
    uint64_t spi_timestamp_us;  /* 0x6B0 接收时间戳 */
    /* --- 0x6A0 运行反馈帧1 --- */
    int16_t  motor_temp_x10;         /* 电机线圈温度, 0.1°C */
    int16_t  iq_current;             /* Iq电流, mA (0x6A0 Byte0-1) */
    int32_t  bus_current;            /* 母线电流, mA (0x6A0 Byte2-3, S16→S32扩展) */
    uint16_t error_code;             /* 错误码 (0x6A0 Byte6-7, 与 0x603F 一致) */
    uint64_t run_fb_timestamp_us;    /* 0x6A0 接收时间戳 */
    /* --- 0x690 运行反馈帧0 --- */
    int32_t  motor_pos_raw;          /* 电机端实际位置 S32 LE, cnt */
    int32_t  motor_vel_raw;          /* 电机端实际速度 S32 LE, RPM */
} motor_sensor_t;

/* 0x6B0 力矩帧解析结果
 * byte0~2: 力矩 24bit 有符号小端; byte4: valid; byte5: error; 帧内无 CRC */
typedef struct {
    int32_t  force_raw_s24;    /* byte0~2 小端 24bit 有符号, 符号扩展到 int32 */
    uint8_t  valid;            /* byte4 bit0 */
    uint8_t  error;            /* byte5 错误码 */
} spi_force_frame_t;

/* ============================================================================
 * 对象字典索引
 * ============================================================================ */

/* 厂商特定 */
#define OD_NODE_ID            (0x2530)  /* 电机 ID */
#define OD_ZERO_POSITION      (0x2531)  /* 零位标定 */
#define OD_CURRENT_P          (0x2532)
#define OD_CURRENT_I          (0x2533)
#define OD_VELOCITY_P         (0x2534)
#define OD_VELOCITY_I         (0x2535)
#define OD_POSITION_P         (0x2536)
#define OD_POSITION_I         (0x2537)
#define OD_MAX_CURRENT        (0x2538)
#define OD_SAVE_FLASH         (0x2539)
#define OD_CANFD_BAUD         (0x2540)
#define OD_WATCHDOG_LIMIT     (0x2650)  /* 看门狗/限位控制 */

/* MIT 快控缩放 (KWS CANFD V2, 启动时 SDO 读取) */
#define OD_MIT_POS_SCALE      (0x2542)  /* Pmax × 0.01 rad, 默认 314 */
#define OD_MIT_VEL_SCALE      (0x2543)  /* Vmax × 0.01 rad/s, 默认 314 */
#define OD_MIT_KP_SCALE       (0x2544)  /* Kpmax Nm/rad, 默认 50 */
#define OD_MIT_KD_SCALE       (0x2545)  /* Kdmax Nm·s/rad, 默认 50 */
#define OD_MIT_TQ_SCALE       (0x2546)  /* Tmax Nm, 默认 20 */

/* MIT 缩放参数, 每个节点独立 */
typedef struct {
    float pmax;    /* 位置量程, rad, 默认 3.14 */
    float vmax;    /* 速度量程, rad/s, 默认 3.14 */
    float kpmax;   /* 刚度量程, Nm/rad, 默认 50 */
    float kdmax;   /* 阻尼量程, Nm·s/rad, 默认 50 */
    float tmax;    /* 力矩量程, Nm, 默认 20 */
} mit_scales_t;

/* CiA 402 标准 */
#define OD_CONTROLWORD        (0x6040)
#define OD_STATUSWORD         (0x6041)
#define OD_MODE_OF_OP         (0x6060)
#define OD_MODE_OF_OP_DISP    (0x6061)
#define OD_POSITION_ACTUAL    (0x6064)
#define OD_VELOCITY_ACTUAL    (0x606C)
#define OD_TORQUE_ACTUAL      (0x6077)
#define OD_CURRENT_ACTUAL     (0x6078)
#define OD_TARGET_POS         (0x607A)
#define OD_TARGET_VELOCITY    (0x60FF)
#define OD_TARGET_TORQUE      (0x6071)
#define OD_PROFILE_VEL        (0x6081)
#define OD_PROFILE_ACCEL      (0x6083)
#define OD_PROFILE_DECEL      (0x6084)
#define OD_POS_LIMIT          (0x607D)  /* sub0x01=负, sub0x02=正 */
#define OD_HEARTBEAT          (0x1017)
#define OD_MANUFACTURER       (0x1008)  /* 厂家名称 */
#define OD_HARDWARE_VER       (0x1009)
#define OD_FIRMWARE_VER       (0x100A)
#define OD_MOTOR_MODEL        (0x1000)

/* ============================================================================
 * 电机配置
 * ============================================================================ */

typedef struct {
    uint8_t  node_id;             /* CAN节点 ID (1~127) */
    uint32_t heartbeat_ms;        /* 心跳周期 (默认 2000) */
    uint16_t profile_accel;       /* 加速度 RPM/s (电机端, 0~10000) */
    uint16_t profile_decel;       /* 减速度 RPM/s (电机端, 0~10000) */
    uint16_t profile_velocity;    /* 轨迹速度 RPM (输出端, 0~30) */
    float    pos_limit_pos;       /* 正限位 (度) */
    float    pos_limit_neg;       /* 负限位 (度) */
    bool     disable_watchdog;    /* 上电关闭看门狗 */
    bool     auto_enable;         /* startup 自动使能 */
    int      bootup_timeout_ms;   /* 启动超时 */
    uint8_t  tpdo_sync_count;     /* TPDO 同步上报: 1~240=每N个SYNC发一次, 0=不配置 */
} motor_config_t;

/* ============================================================================
 * PID 参数
 * ============================================================================ */

typedef struct {
    uint16_t current_p;
    uint16_t current_i;
    uint16_t velocity_p;
    uint16_t velocity_i;
    uint16_t position_p;
    uint16_t position_i;
} motor_pid_t;

/* ============================================================================
 * 多轴广播命令
 * ============================================================================ */

typedef struct {
    uint8_t   node_id;
    motor_mode_t mode;
    bool      enable;
    bool      release_brake;
    bool      clear_error;
    int16_t   target1;
    uint16_t  target2;
    int16_t   feedforward;
} multi_axis_cmd_t;

/* ============================================================================
 * PDO 映射配置
 * ============================================================================ */

/** PDO 映射条目 — 用户自定义哪些 OD 映射到 RPDO/TPDO */
typedef struct {
    uint16_t index;     /* 对象字典索引, 如 0x6040 */
    uint8_t  subidx;    /* 子索引, 通常 0x00 */
    uint8_t  bitlen;    /* 位宽: 8 / 16 / 32 */
} pdo_map_entry_cfg_t;

/* PDO 类型 */
typedef enum {
    PDO_TYPE_RPDO = 0,  /* 主站, 从站 (控制) */
    PDO_TYPE_TPDO = 1,  /* 从站, 主站 (上报) */
} pdo_type_t;

/* ============================================================================
 * 回调
 * ============================================================================ */

typedef void (*motor_feedback_cb_t)(uint8_t node_id, const motor_feedback_t *fb, void *ctx);
typedef void (*motor_error_cb_t)(uint8_t node_id, uint16_t error_code, void *ctx);
typedef void (*motor_state_cb_t)(uint8_t node_id, motor_state_t old_state, motor_state_t new_state, void *ctx);
typedef void (*motor_sensor_cb_t)(uint8_t node_id, const motor_sensor_t *s, void *ctx);
typedef void (*motor_tpdo_raw_cb_t)(uint8_t node_id, const canfd_frame_t *f, void *ctx);

/* ============================================================================
 * LED 灯控制 (OD 0x5503:06, SDO 读写)
 * ============================================================================ */

/* 灯效模式 */
typedef enum {
    LED_STEADY = 0,   /* 常亮 */
    LED_BLINK  = 1,   /* 闪烁 */
    LED_BREATH = 2,   /* 呼吸 */
    LED_CHASE  = 3,   /* 流水 */
} led_mode_t;

/* 使能掩码 (bit[7:4]) */
#define LED1_MASK  0x10
#define LED2_MASK  0x20
#define LED3_MASK  0x40
#define LED4_MASK  0x80
#define LED_ALL     0xF0

/* LED 配置 */
#ifndef LED_CONFIG_T_DEFINED
#define LED_CONFIG_T_DEFINED
typedef struct {
    uint8_t enable_mask;
    uint8_t mode;         /* led_mode_t */
    uint8_t r, g, b;      /* 0-255 */
} led_config_t;
#endif

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HAL_TYPES_H */
