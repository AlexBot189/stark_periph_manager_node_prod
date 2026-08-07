/**
 * @file pdo_handler.c
 * @brief PDO 构造与反馈解析 - 高层封装
 *
 * 封装 canopen_frames 的 PDO 相关函数,
 * 提供更简洁的调用接口供 motor_hal.c 使用。
 */

#include "canopen_frames.h"
#include "can_driver_internal.h"
#include <stdio.h>

/* ---------- 诊断: hex dump (仅调试模式) ---------- */
#ifdef MOTOR_DEBUG_HEX
static void _dump_hex(const char *dir, const canfd_frame_t *f)
{
    fprintf(stderr, "[PDO %s] id=0x%03X dlc=%d :", dir, f->id, f->dlc);
    for (int i = 0; i < f->dlc && i < 64; i++) {
        fprintf(stderr, " %02X", f->data[i]);
    }
    fprintf(stderr, "\n");
}
#define DUMP(dir, f) _dump_hex(dir, f)
#else
#define DUMP(dir, f) ((void)0)
#endif

/* ---------- MIT cansend 报文 (调试: 验证 control word / raw payload / padding) ---------- */

/* CAN FD 有效字节数 → 线缆字节数 (DLC 编码查表) */
static int _canfd_wire_len(int len)
{
    if (len <= 8)  return len;
    if (len <= 12) return 12;
    if (len <= 16) return 16;
    if (len <= 20) return 20;
    if (len <= 24) return 24;
    if (len <= 32) return 32;
    if (len <= 48) return 48;
    return 64;
}

/* 输出格式: [MIT_CANSEND] can0 <ID>##<flags><hex_data>
 * 示例: [MIT_CANSEND] can0 111##18C800080000000080000000
 *        ##1 = CAN FD BRS=1, 后面 12 字节 hex (DLC=9 → 12 wire bytes)
 */
static void _mit_cansend_dump(const canfd_frame_t *f)
{
    int wl = _canfd_wire_len(f->dlc);
    char buf[512];
    int off = snprintf(buf, sizeof(buf), "[MIT_CANSEND] can0 %03X##%d",
                       (int)f->id, f->use_brs ? 1 : 0);
    for (int i = 0; i < wl && off < (int)sizeof(buf) - 3; i++) {
        off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                        "%02X", f->data[i]);
    }
    fprintf(stderr, "%s\n", buf);
}

/* ---------- 控制 PDO 发送 ---------- */

void pdo_ctrl_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                   bool enable, bool release_brake, bool clear_err,
                   int16_t target1, uint16_t target2, int16_t feedforward)
{
    canfd_frame_t f;
    canopen_custom_pdo_build(node, mode, enable, release_brake, clear_err,
                             target1, target2, feedforward, &f);
    DUMP("TX", &f);
    can_driver_send(drv, &f);
}

void pdo_ctrl_send_raw(can_driver_t *drv, uint8_t node, uint8_t byte0,
                        int16_t target1, uint16_t target2, int16_t feedforward)
{
    canfd_frame_t f;
    canopen_custom_pdo_build_u8(node, byte0, target1, target2, feedforward, &f);
    DUMP("TX", &f);
    can_driver_send(drv, &f);
}

void pdo_mit_send(can_driver_t *drv, uint8_t node, motor_mode_t mode,
                  bool enable, bool release_brake, bool clear_err,
                  uint16_t position, uint16_t velocity,
                  uint16_t kp, uint16_t kd, int16_t torque)
{
    canfd_frame_t f;
    canopen_mit_pdo_build(node, mode, enable, release_brake, clear_err,
                          position, velocity, kp, kd, torque, &f);
    DUMP("TX", &f);
    can_driver_send(drv, &f);
}

void pdo_mit_send_raw(can_driver_t *drv, uint8_t node, uint8_t byte0,
                       uint16_t position, uint16_t velocity,
                       uint16_t kp, uint16_t kd, int16_t torque)
{
    canfd_frame_t f;
    canopen_mit_pdo_build_u8(node, byte0, position, velocity, kp, kd, torque, &f);
    DUMP("TX", &f);
    _mit_cansend_dump(&f);
    can_driver_send(drv, &f);
}

void pdo_multi_send(can_driver_t *drv, const multi_axis_cmd_t *cmds, uint8_t count)
{
    canfd_frame_t f;
    canopen_multi_ctrl_build(cmds, count, &f);
    DUMP("TX", &f);
    can_driver_send(drv, &f);
}

void pdo_mit_multi_send(can_driver_t *drv, const multi_mit_cmd_t *cmds, uint8_t count)
{
    canfd_frame_t f;
    canopen_mit_multi_build(cmds, count, &f);
    DUMP("TX", &f);
    _mit_cansend_dump(&f);
    can_driver_send(drv, &f);
}

/* ---------- SYNC ---------- */

void pdo_sync_send(can_driver_t *drv)
{
    canfd_frame_t f;
    canopen_sync_build(&f);
    DUMP("TX", &f);
    can_driver_send(drv, &f);
}

/* ---------- 反馈解析 ---------- */

void pdo_feedback_parse(const canfd_frame_t *f, motor_feedback_t *fb)
{
    canopen_parse_feedback(f, fb);
}
