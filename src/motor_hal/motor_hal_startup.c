/**
 * @file motor_hal_startup.c
 * @brief 电机启动流程 (v4: Bootup 快速试探 + SDO 同步验证 + 固件读非阻塞)
 *
 * 完整启动序列:
 *   1. 快速试探 Bootup (500ms, 不论结果都继续)
 *   2. SDO 同步配置心跳 真正的电机在线验证
 *   3. 关看门狗 (推荐)
 *   4. 读固件版本 (best-effort, 失败不阻塞后续)
 *   5. 使能 (DS402 状态机: Shutdown, SwitchOn, EnableOp)
 *   6. 延时 120ms (抱闸释放)
 *
 * 改动:
 *   - 步骤1: bootup 等不到不报错, 继续往下走
 *   - 步骤2: fire-and-forget ,  改同步 SDO, 真正的"电机在线"验证
 *   - 步骤4: 固件读失败不阻塞, 打印日志继续
 */

#include "motor_hal_types.h"
#include "canopen_frames.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"
#include <errno.h>
#include <time.h>
#include <stdio.h>

/* ---------- 快速试探 Bootup (v4: 非硬门槛, 不论结果都返回 0) ---------- */

int motor_startup_wait_bootup(can_driver_t *drv __attribute__((unused)),
                              uint8_t node_id __attribute__((unused)),
                              int timeout_ms,
                              const volatile bool *bootup_flag)
{
    int elapsed = 0;
    while (!*bootup_flag && elapsed < timeout_ms) {
        usleep(10000);
        elapsed += 10;
    }
    /* Bootup 是一次性事件, 错过了就错过了。
       不论有没有收到, 都返回 0 继续 — 真正的验证走步骤 2 的 SDO。 */
    return 0;
}

/* ---------- 使能 (DS402 状态机) ---------- */

int motor_startup_enable(can_driver_t *drv, uint8_t node_id)
{
    int ret;

    /* Step A: Shutdown ,  Ready to Switch On */
    ret = sdo_write_simple(drv, node_id, OD_CONTROLWORD, 0x00, CW_SHUTDOWN, 2);
    if (ret != 0) return ret;
    usleep(20000);

    /* Step B: Switch On ,  Switched On */
    ret = sdo_write_simple(drv, node_id, OD_CONTROLWORD, 0x00, CW_SWITCH_ON, 2);
    if (ret != 0) return ret;
    usleep(20000);

    /* Step C: Enable Operation ,  Operation Enabled */
    ret = sdo_write_simple(drv, node_id, OD_CONTROLWORD, 0x00, CW_ENABLE_OP, 2);
    if (ret != 0) return ret;

    /* 等待抱闸释放 (100ms + 余量) */
    usleep(120000);

    return 0;
}

/* ---------- 完整启动 (v4) ---------- */

int motor_startup_full(can_driver_t *drv, const motor_config_t *cfg,
                       const volatile bool *bootup_flag)
{
    int ret;

    /* 1. 快速试探 Bootup (500ms, 不论结果都继续) */
    motor_startup_wait_bootup(drv, cfg->node_id, 500, bootup_flag);

    /* 1.5 NMT Start — 从 Stopped/Pre-Operational 切到 Operational
     *     必须在所有 SDO 操作之前, 否则驱动板在 Stopped 状态下不响应 SDO */
    {
        canfd_frame_t nmt_f;
        canopen_nmt_build(NMT_CMD_START, cfg->node_id, &nmt_f);
        can_driver_send(drv, &nmt_f);
        fprintf(stderr, "[startup] motor %d NMT Start ,  Operational\n", cfg->node_id);
        usleep(10000);  /* 等驱动板切状态 */
    }

    /* 2. SDO 同步配置心跳 用同步 SDO 等响应:
         有响应 ,  电机确认在线 + 心跳周期生效
         超时   ,  电机不在线, 返回错误 */
    ret = sdo_write_simple(drv, cfg->node_id, OD_HEARTBEAT, 0x00,
                           cfg->heartbeat_ms, 2);
    if (ret != 0) return ret;

    /* 3. 关看门狗 (推荐) */
    if (cfg->disable_watchdog) {
        sdo_write_simple(drv, cfg->node_id, OD_WATCHDOG_LIMIT, 0x00, 1, 4);
    }

    /* 4. 读固件版本 (best-effort: 成功记录, 失败跳过不阻塞) */
    {
        uint32_t ver = 0;
        ret = sdo_read_simple(drv, cfg->node_id, OD_FIRMWARE_VER, 0x00, &ver);
        if (ret == 0) {
            fprintf(stderr, "[startup] motor %d firmware=0x%08X\n",
                    cfg->node_id, ver);
        } else {
            fprintf(stderr, "[startup] motor %d firmware read skipped (ret=%d)\n",
                    cfg->node_id, ret);
        }
    }

    /* 4.5 配置运动参数 (best-effort, CiA 402: 0x6081/6083/6084) */
    if (cfg->profile_velocity > 0) {
        sdo_write_simple(drv, cfg->node_id, OD_PROFILE_VEL, 0x00,
                         cfg->profile_velocity, 4);
    }
    if (cfg->profile_accel > 0) {
        sdo_write_simple(drv, cfg->node_id, OD_PROFILE_ACCEL, 0x00,
                         cfg->profile_accel, 4);
    }
    if (cfg->profile_decel > 0) {
        sdo_write_simple(drv, cfg->node_id, OD_PROFILE_DECEL, 0x00,
                         cfg->profile_decel, 4);
    } else if (cfg->profile_accel > 0) {
        sdo_write_simple(drv, cfg->node_id, OD_PROFILE_DECEL, 0x00,
                         cfg->profile_accel, 4);
    }

    /* 5. 使能 */
    if (cfg->auto_enable) {
        ret = motor_startup_enable(drv, cfg->node_id);
        if (ret != 0) return ret;
    }

    /* 6. 配置 TPDO 同步上报 (如果 tpdo_sync_count > 0) */
    if (cfg->tpdo_sync_count > 0) {
        uint32_t cob = (uint32_t)(0x180 + cfg->node_id);
        ret = sdo_write_simple(drv, cfg->node_id, 0x1800, 0x01,
                               cob | 0x80000000UL, 4);
        if (ret == 0) {
            sdo_write_simple(drv, cfg->node_id, 0x1A00, 0x00, 0, 1);
            sdo_write_simple(drv, cfg->node_id, 0x1A00, 0x01,
                             ((uint32_t)0x6041 << 16) | 16, 4);  /* Statusword 16b */
            sdo_write_simple(drv, cfg->node_id, 0x1A00, 0x02,
                             ((uint32_t)0x6064 << 16) | 32, 4);  /* Position 32b */
            sdo_write_simple(drv, cfg->node_id, 0x1A00, 0x03,
                             ((uint32_t)0x606C << 16) | 32, 4);  /* Velocity 32b */
            sdo_write_simple(drv, cfg->node_id, 0x1A00, 0x04,
                             ((uint32_t)0x6078 << 16) | 16, 4);  /* Current 16b */
            sdo_write_simple(drv, cfg->node_id, 0x1A00, 0x00, 4, 1);
            sdo_write_simple(drv, cfg->node_id, 0x1800, 0x02,
                             cfg->tpdo_sync_count, 1);
            sdo_write_simple(drv, cfg->node_id, 0x1800, 0x01, cob, 4);
            fprintf(stderr, "[startup] motor %d TPDO1 configured: sync_every=%d\n",
                    cfg->node_id, cfg->tpdo_sync_count);
        }
    }

    return 0;
}
