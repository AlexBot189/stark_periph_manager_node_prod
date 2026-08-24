/**
 * @file canopen_frames.c
 * @brief CANopen 帧构造与解析实现
 */

#include "canopen_frames.h"
#include <time.h>

/* =====================================================
 * 时间戳
 * ===================================================== */

static uint64_t _now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
}

/* =====================================================
 * SDO 帧
 * ===================================================== */

void canopen_sdo_write_build(uint8_t node, uint16_t index, uint8_t subidx,
                              uint32_t value, uint8_t size_bytes,
                              canfd_frame_t *f)
{
    uint8_t cmd;
    switch (size_bytes) {
        case 1: cmd = SDO_CC_DOWNLOAD_1B; break;
        case 2: cmd = SDO_CC_DOWNLOAD_2B; break;
        case 4:
        default:cmd = SDO_CC_DOWNLOAD_4B; break;
    }

    memset(f, 0, sizeof(*f));
    f->id    = COB_SDO_TX_BASE + node;
    f->dlc   = 8;
    f->is_fd = false;

    f->data[0] = cmd;
    f->data[1] = (uint8_t)(index & 0xFF);
    f->data[2] = (uint8_t)((index >> 8) & 0xFF);
    f->data[3] = subidx;
    f->data[4] = (uint8_t)(value & 0xFF);
    f->data[5] = (uint8_t)((value >> 8) & 0xFF);
    f->data[6] = (uint8_t)((value >> 16) & 0xFF);
    f->data[7] = (uint8_t)((value >> 24) & 0xFF);
}

void canopen_sdo_read_build(uint8_t node, uint16_t index, uint8_t subidx,
                             canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_SDO_TX_BASE + node;
    f->dlc   = 8;
    f->is_fd = false;

    f->data[0] = SDO_CC_UPLOAD_REQ;
    f->data[1] = (uint8_t)(index & 0xFF);
    f->data[2] = (uint8_t)((index >> 8) & 0xFF);
    f->data[3] = subidx;
}

bool canopen_sdo_parse_response(const canfd_frame_t *f,
                                uint32_t *value, uint16_t *out_index,
                                uint8_t *out_subidx, uint32_t *abort_code)
{
    if (!f) return false;

    uint8_t cmd = f->data[0];

    /* 先提取索引/子索引, ABORT 也需要 */
    uint16_t idx = (uint16_t)f->data[1] | ((uint16_t)f->data[2] << 8);
    uint8_t  sub = f->data[3];

    if (cmd == SDO_CC_ABORT) {
        if (out_index)  *out_index  = idx;
        if (out_subidx) *out_subidx = sub;
        if (abort_code) {
            *abort_code = (uint32_t)f->data[4]
                        | ((uint32_t)f->data[5] << 8)
                        | ((uint32_t)f->data[6] << 16)
                        | ((uint32_t)f->data[7] << 24);
        }
        return false;
    }

    if (out_index)  *out_index  = idx;
    if (out_subidx) *out_subidx = sub;

    switch (cmd) {
        case SDO_CC_UPLOAD_RSP_1B:  /* 0x4F */
            if (value) *value = f->data[4];
            return true;
        case SDO_CC_UPLOAD_RSP_2B:  /* 0x4B */
            if (value) *value = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8);
            return true;
        case SDO_CC_UPLOAD_RSP_4B:  /* 0x43 */
        case 0x41:                  /* 0x41: expedited 4B, e=0 variant (巨蟹固件) */
        case 0x47:                  /* 0x47: expedited 3B, e=0 variant */
            if (value) {
                *value = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8)
                       | ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24);
            }
            return true;
        case 0x60:  /* SDO 写确认 (从站, 主站) */
            return true;
        case SDO_CC_DOWNLOAD_1B:
        case SDO_CC_DOWNLOAD_2B:
        case SDO_CC_DOWNLOAD_4B:
            /* 从站回显下载命令码 (某些驱动用这种方式确认) */
            return true;
        default:
            return false;
    }
}

/* =====================================================
 * NMT / SYNC
 * ===================================================== */

void canopen_nmt_build(uint8_t cmd, uint8_t node, canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_NMT;
    f->dlc   = 2;
    f->is_fd = false;
    f->data[0] = cmd;
    f->data[1] = node;
}

void canopen_sync_build(canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_SYNC;
    f->dlc   = 0;
    f->is_fd   = true;   /* CANFD, 驱动板据此决定 0x300 数据完整度 */
    f->use_brs = true;   /* 开启 BRS */
}

/* =====================================================
 * 自定义单轴 PDO (u8 Byte0 版本 — 控制循环推荐)
 * ===================================================== */

void canopen_custom_pdo_build_u8(uint8_t node, uint8_t byte0,
                                  int16_t target1, uint16_t target2,
                                  int16_t feedforward,
                                  canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id     = COB_PDO_CTRL_BASE + node;
    f->dlc    = 7;
    f->is_fd   = true;
    f->use_brs = true;

    f->data[0] = byte0;
    f->data[1] = (uint8_t)((target1 >> 8) & 0xFF);  /* 大端 */
    f->data[2] = (uint8_t)(target1 & 0xFF);
    f->data[3] = (uint8_t)((target2 >> 8) & 0xFF);
    f->data[4] = (uint8_t)(target2 & 0xFF);
    f->data[5] = (uint8_t)((feedforward >> 8) & 0xFF);
    f->data[6] = (uint8_t)(feedforward & 0xFF);
}

void canopen_mit_pdo_build_u8(uint8_t node, uint8_t byte0,
                               uint16_t position, uint16_t velocity,
                               uint16_t kp, uint16_t kd, int16_t torque,
                               canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id     = COB_PDO_MIT_BASE + node;
    f->dlc    = 9;
    f->is_fd   = true;
    f->use_brs = true;

    f->data[0] = byte0;
    f->data[1] = (uint8_t)((position >> 8) & 0xFF);
    f->data[2] = (uint8_t)(position & 0xFF);
    f->data[3] = (uint8_t)((velocity >> 4) & 0xFF);
    f->data[4] = (uint8_t)(((velocity & 0x0F) << 4) | ((kp >> 8) & 0x0F));
    f->data[5] = (uint8_t)(kp & 0xFF);
    f->data[6] = (uint8_t)((kd >> 4) & 0xFF);
    f->data[7] = (uint8_t)(((kd & 0x0F) << 4) | ((torque >> 8) & 0x0F));
    f->data[8] = (uint8_t)(torque & 0xFF);
}

void canopen_custom_pdo_build(uint8_t node, motor_mode_t mode,
                               bool enable, bool release_brake, bool clear_err,
                               int16_t target1, uint16_t target2,
                               int16_t feedforward,
                               canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_PDO_CTRL_BASE + node;
    f->dlc   = 7;
    f->is_fd  = true;
    f->use_brs = true;  /* PDO 数据段切 5Mbps */

    uint8_t flags = 0;
    if (enable)        flags |= (1 << 7);
    if (release_brake) flags |= (1 << 6);
    if (clear_err)     flags |= (1 << 5);
    flags |= ((uint8_t)mode & 0x0F) << 1;

    /* 文档要求大端: Byte[N]高8位, Byte[N+1]低8位 */
    f->data[0] = flags;
    f->data[1] = (uint8_t)((target1 >> 8) & 0xFF);     /* 高8位 */
    f->data[2] = (uint8_t)(target1 & 0xFF);            /* 低8位 */
    f->data[3] = (uint8_t)((target2 >> 8) & 0xFF);     /* 高8位 */
    f->data[4] = (uint8_t)(target2 & 0xFF);            /* 低8位 */
    f->data[5] = (uint8_t)((feedforward >> 8) & 0xFF); /* 高8位 */
    f->data[6] = (uint8_t)(feedforward & 0xFF);        /* 低8位 */
}

/* =====================================================
 * MIT 模式 PDO
 * ===================================================== */

void canopen_mit_pdo_build(uint8_t node, motor_mode_t mode,
                            bool enable, bool release_brake, bool clear_err,
                            uint16_t position, uint16_t velocity,
                            uint16_t kp, uint16_t kd, int16_t torque,
                            canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_PDO_MIT_BASE + node;
    f->dlc   = 9;
    f->is_fd  = true;
    f->use_brs = true;  /* PDO 数据段切 5Mbps */

    uint8_t flags = 0;
    if (enable)        flags |= (1 << 7);
    if (release_brake) flags |= (1 << 6);
    if (clear_err)     flags |= (1 << 5);
    flags |= ((uint8_t)mode & 0x0F) << 1;

    f->data[0] = flags;
    /* 文档大端: [1]=位置高8, [2]=位置低8 */
    f->data[1] = (uint8_t)((position >> 8) & 0xFF);
    f->data[2] = (uint8_t)(position & 0xFF);
    /* 速度12bit[11:4], [3], 速度低4bit[3:0], [4][7:4] */
    f->data[3] = (uint8_t)((velocity >> 4) & 0xFF);
    /* Kp高4bit[11:8], [4][3:0], Kp低8bit[7:0], [5] */
    f->data[4] = (uint8_t)(((velocity & 0x0F) << 4) | ((kp >> 8) & 0x0F));
    f->data[5] = (uint8_t)(kp & 0xFF);
    /* Kd高8bit[11:4], [6] */
    f->data[6] = (uint8_t)((kd >> 4) & 0xFF);
    /* Kd低4bit[3:0], [7][7:4], 力矩高4bit[11:8], [7][3:0] */
    f->data[7] = (uint8_t)(((kd & 0x0F) << 4) | ((torque >> 8) & 0x0F));
    /* 力矩低8bit[7:0], [8] */
    f->data[8] = (uint8_t)(torque & 0xFF);
}

/* =====================================================
 * MIT 多轴广播 (0x210, 64字节, 最多6电机)
 * 每 slot 9 字节 = 控制字+pos(16b)+vel(12b)+kp(12b)+kd(12b)+tq(12b)
 * ===================================================== */

void canopen_mit_multi_build(const multi_mit_cmd_t *cmds, uint8_t count,
                              canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_MULTI_MIT;
    f->dlc   = 64;
    f->is_fd  = true;
    f->use_brs = true;

    if (count > 6) count = 6;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t base = i * 9;

        uint8_t flags = 0;
        if (cmds[i].enable)        flags |= (1 << 7);
        if (cmds[i].release_brake) flags |= (1 << 6);
        if (cmds[i].clear_error)   flags |= (1 << 5);
        flags |= ((uint8_t)6 & 0x0F) << 1;  /* mode=MIT */

        f->data[base + 0] = flags;
        f->data[base + 1] = (uint8_t)((cmds[i].position >> 8) & 0xFF);
        f->data[base + 2] = (uint8_t)(cmds[i].position & 0xFF);
        f->data[base + 3] = (uint8_t)((cmds[i].velocity >> 4) & 0xFF);
        f->data[base + 4] = (uint8_t)(((cmds[i].velocity & 0x0F) << 4) | ((cmds[i].kp >> 8) & 0x0F));
        f->data[base + 5] = (uint8_t)(cmds[i].kp & 0xFF);
        f->data[base + 6] = (uint8_t)((cmds[i].kd >> 4) & 0xFF);
        f->data[base + 7] = (uint8_t)(((cmds[i].kd & 0x0F) << 4) | ((cmds[i].torque >> 8) & 0x0F));
        f->data[base + 8] = (uint8_t)(cmds[i].torque & 0xFF);
    }

    /* ID 映射: Byte[56-61] */
    for (uint8_t i = 0; i < count; i++) {
        f->data[56 + i] = cmds[i].node_id;
    }
}

/* =====================================================
 * 多轴广播 (0x200)
 * ===================================================== */

void canopen_multi_ctrl_build(const multi_axis_cmd_t *cmds, uint8_t count,
                               canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id    = COB_MULTI_CTRL;
    f->dlc   = 64;
    f->is_fd  = true;
    f->use_brs = true;  /* PDO 数据段切 5Mbps */

    if (count > 8) count = 8;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t base = i * 7;

        uint8_t flags = 0;
        if (cmds[i].enable)        flags |= (1 << 7);
        if (cmds[i].release_brake) flags |= (1 << 6);
        if (cmds[i].clear_error)   flags |= (1 << 5);
        flags |= ((uint8_t)cmds[i].mode & 0x0F) << 1;

        f->data[base + 0] = flags;
        /* 大端: Byte[N+1]高8位, Byte[N+2]低8位 */
        f->data[base + 1] = (uint8_t)((cmds[i].target1 >> 8) & 0xFF);
        f->data[base + 2] = (uint8_t)(cmds[i].target1 & 0xFF);
        f->data[base + 3] = (uint8_t)((cmds[i].target2 >> 8) & 0xFF);
        f->data[base + 4] = (uint8_t)(cmds[i].target2 & 0xFF);
        f->data[base + 5] = (uint8_t)((cmds[i].feedforward >> 8) & 0xFF);
        f->data[base + 6] = (uint8_t)(cmds[i].feedforward & 0xFF);
    }

    /* Byte[56-63]: ID 映射 */
    for (uint8_t i = 0; i < count; i++) {
        f->data[56 + i] = cmds[i].node_id;
    }
}

/* =====================================================
 * 反馈帧解析
 * ===================================================== */

void canopen_parse_feedback(const canfd_frame_t *f, motor_feedback_t *fb)
{
    memset(fb, 0, sizeof(*fb));
    /* 优先用 can_driver_recv 的 read 打点 (can0 读帧时刻), 否则回退到解析时刻 */
    fb->timestamp_us = (f && f->rx_timestamp_us) ? f->rx_timestamp_us : _now_us();

    if (!f || f->dlc < 12) return;

    /* 文档大端: Byte[N]高8位, Byte[N+1]低8位 */
    /* Byte[0-1]: 位置 (int16) */
    fb->position = (int16_t)(((uint16_t)f->data[0] << 8) | (uint16_t)f->data[1]);

    /* Byte[2-3]: 速度 (int16, RPM) */
    fb->velocity = (int16_t)(((uint16_t)f->data[2] << 8) | (uint16_t)f->data[3]);

    /* Byte[4-5]: Iq 电流 (int16, mA) */
    fb->current_iq = (int16_t)(((uint16_t)f->data[4] << 8) | (uint16_t)f->data[5]);

    /* Byte[6-7]: 错误码 */
    fb->error_code = ((uint16_t)f->data[6] << 8) | (uint16_t)f->data[7];

    /* Byte[8-9]: 温度 (int16, 0.1°C) */
    fb->temperature = (int16_t)(((uint16_t)f->data[8] << 8) | (uint16_t)f->data[9]);

    /* V2 扩展: DLC=16 (带力矩传感器) vs DLC=12 (无力矩传感器) */
    if (f->dlc >= 16) {
        /* Byte[10-11]: 力矩反馈 (int16, 0.05N.m) */
        fb->torque_nm = (int16_t)(((uint16_t)f->data[10] << 8) | (uint16_t)f->data[11]);
        /* Byte[12-14]: 预留 (int24 电机端编码器位置), 跳过 */
        /* Byte[15] bit7-4: 状态 
           Byte[15] bit3-0: 控制模式反馈 */
        fb->status_byte = f->data[15];
        fb->mode        = f->data[15] & 0x0F;
    } else {
        /* DLC=12: 无力矩传感器版本 */
        /* Byte[10]: 控制模式 */
        fb->mode = f->data[10];
        /* Byte[11]: 状态字节 */
        fb->status_byte = f->data[11];
    }
}

/* =====================================================
 * 0x6B0 力矩帧解析
 * byte0~2 = 力矩 24bit 有符号小端, byte4 = valid, byte5 = error, 其余保留
 * 帧内无 CRC (设备侧已校验, 结果体现在 valid/error)
 * ===================================================== */

void canopen_parse_spi_force(const canfd_frame_t *f, spi_force_frame_t *st)
{
    if (!st) return;
    memset(st, 0, sizeof(*st));
    if (!f || f->dlc < 6) return;

    /* byte0~2 小端 24bit 有符号, 符号扩展到 int32 */
    int32_t raw = (int32_t)f->data[0]
                | ((int32_t)f->data[1] << 8)
                | ((int32_t)f->data[2] << 16);
    if (raw & 0x00800000) raw |= (int32_t)0xFF000000;  /* bit23 符号扩展 */
    st->force_raw_s24 = raw;

    st->valid = (uint8_t)(f->data[4] & 0x01U);  /* byte4.bit0 */
    st->error = f->data[5];                      /* byte5 设备错误码 */
}
