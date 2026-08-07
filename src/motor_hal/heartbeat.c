/**
 * @file heartbeat.c
 * @brief 心跳 / 看门狗管理
 *
 * 巨蟹驱动板:
 *   - 上电自锁, 看门狗间隔 < 500ms
 *   - 可关闭看门狗 (0x2650=1)
 *   - 可设心跳周期 (0x1017=ms)
 *
 * 策略: 默认关闭看门狗 (最省事); 若保留, 则每 100ms 发 SYNC 喂狗。
 */

#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"

void heartbeat_set_period(can_driver_t *drv, uint8_t node, uint32_t ms)
{
    canfd_frame_t f;
    canopen_sdo_write_build(node, OD_HEARTBEAT, 0x00, ms, 2, &f);
    can_driver_send(drv, &f);
}

void heartbeat_disable_watchdog(can_driver_t *drv, uint8_t node)
{
    sdo_write_simple(drv, node, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
}

/* 喂狗: 发送 SYNC 帧 (0x080, 0字节, 效率极高) */
void heartbeat_feed(can_driver_t *drv)
{
    canfd_frame_t f;
    canopen_sync_build(&f);
    can_driver_send(drv, &f);
}
