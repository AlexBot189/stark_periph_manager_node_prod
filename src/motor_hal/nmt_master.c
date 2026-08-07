/**
 * @file nmt_master.c
 * @brief NMT 主站 - 网络管理命令
 *
 * NMT (Network Management) 用于控制从站状态:
 *   - Start Remote Node (0x01): 从站进入 Operational
 *   - Stop Remote Node  (0x02): 从站进入 Stopped
 *   - Enter Pre-Op      (0x80): 从站进入 Pre-Operational
 *   - Reset Node        (0x81): 复位从站应用
 *   - Reset Communication (0x82): 复位从站通信
 */

#include "canopen_frames.h"
#include "can_driver_internal.h"

void nmt_send(can_driver_t *drv, uint8_t cmd, uint8_t node)
{
    canfd_frame_t f;
    canopen_nmt_build(cmd, node, &f);
    can_driver_send(drv, &f);
}

void nmt_start(can_driver_t *drv, uint8_t node)
{
    nmt_send(drv, NMT_CMD_START, node);
}

void nmt_stop(can_driver_t *drv, uint8_t node)
{
    nmt_send(drv, NMT_CMD_STOP, node);
}

void nmt_pre_operational(can_driver_t *drv, uint8_t node)
{
    nmt_send(drv, NMT_CMD_PRE_OP, node);
}

void nmt_reset_node(can_driver_t *drv, uint8_t node)
{
    nmt_send(drv, NMT_CMD_RESET_NODE, node);
}

void nmt_reset_comm(can_driver_t *drv, uint8_t node)
{
    nmt_send(drv, NMT_CMD_RESET_COMM, node);
}

void nmt_broadcast(can_driver_t *drv, uint8_t cmd)
{
    nmt_send(drv, cmd, 0);  /* node=0 ,  广播 */
}
