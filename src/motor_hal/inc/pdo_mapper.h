/**
 * @file pdo_mapper.h
 * @brief 标准 CANopen PDO 映射器
 *
 * 通过 SDO 配置从站的 RPDO/TPDO 映射和传输参数。
 * 模块化设计: 仅依赖 sdo_client, 可按需链接。
 */

#ifndef PDO_MAPPER_H
#define PDO_MAPPER_H

#include "motor_hal_types.h"
#include "can_driver_internal.h"
#include "sdo_client_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CANopen PDO 通信参数 (0x1400-0x15FF: RPDO, 0x1800-0x19FF: TPDO)
 * ============================================================================ */

/* 厂商默认 COB-ID */
#define PDO_DEFAULT_RPDO1_COB  (0x201)   /* RPDO1: 0x200 + node_id? 文档标注 */
#define PDO_DEFAULT_TPDO1_COB  (0x181)   /* TPDO1: 0x180 + node_id */

/* 标准 COB-ID */
#define PDO_TPDO1_COB(n)       (0x180 + (n))   /* TPDO1: 0x180 + node_id */
#define PDO_RPDO1_COB(n)       (0x200 + (n))   /* RPDO1: 0x200 + node_id */
#define PDO_TPDO2_COB(n)       (0x280 + (n))
#define PDO_RPDO2_COB(n)       (0x300 + (n))   /* 注意: 0x300 被反馈帧占用! 偏移 RPDO2 到 0x400 */
#define PDO_RPDO2_COB_SAFE(n)  (0x400 + (n))   /* 避免与 0x300 反馈帧冲突 */

/* RPDO 通信参数索引 */
#define OD_RPDO1_COMM    (0x1400)   /* sub0x01=COB-ID, sub0x02=传输类型 */
#define OD_RPDO2_COMM    (0x1401)
#define OD_RPDO1_MAP     (0x1600)   /* sub0x00=映射数量, sub0x01-08=映射条目 */
#define OD_RPDO2_MAP     (0x1601)

/* TPDO 通信参数索引 */
#define OD_TPDO1_COMM    (0x1800)
#define OD_TPDO2_COMM    (0x1801)
#define OD_TPDO1_MAP     (0x1A00)
#define OD_TPDO2_MAP     (0x1A01)

/* 传输类型 */
#define PDO_TTYPE_SYNC_ACYCLIC  (0)     /* 同步非周期 (收到SYNC且数据变) */
#define PDO_TTYPE_SYNC_CYCLIC(n) (n)    /* 同步周期: 1~240 (每n个SYNC发一次) */
#define PDO_TTYPE_ASYNC_MFG     (254)   /* 异步, 厂商事件触发 */
#define PDO_TTYPE_ASYNC_DEV     (255)   /* 异步, 设备子协议触发 */

/* ============================================================================
 * PDO 映射条目构造
 *
 * 映射条目: 32bit = [Index(16)] [SubIndex(8)] [BitLength(8)]
 * 示例: 0x60400010 = 索引0x6040, 子索引0x00, 长度16bit (Controlword)
 *       0x607A0020 = 索引0x607A, 子索引0x00, 长度32bit (Target Position)
 * ============================================================================ */

static inline uint32_t pdo_map_entry(uint16_t index, uint8_t subidx, uint8_t bitlen) {
    return ((uint32_t)index << 16) | ((uint32_t)subidx << 8) | (uint32_t)bitlen;
}

/* ============================================================================
 * API: 配置标准 RPDO (主站, 从站, 用于控制)
 * ============================================================================ */

/**
 * @brief 配置 RPDO1 为标准位置控制映射
 *
 * 映射:
 *   Byte0-1: Controlword (0x6040.00, 16bit)
 *   Byte2-5: Target Position (0x607A.00, 32bit)
 *
 * CAN 2.0 下需要 6 字节; CANFD 下可再追加更多映射。
 *
 * 使用后可直接发 RPDO1 帧 (COB=0x200+node) 来控制电机:
 *   f.id = 0x200 + node_id;
 *   f.dlc = 6;
 *   f.data[0-1] = controlword;   // 小端
 *   f.data[2-5] = target_pos;    // 小端
 */
int pdo_mapper_config_rpdo1_position(can_driver_t *drv, uint8_t node,
                                     uint32_t cob_id, uint8_t trans_type);

/**
 * @brief 配置 RPDO1 为 CANFD 扩展映射 (充分利用 64 字节)
 *
 * 映射:
 *   Controlword (16b) + Target Position (32b) + Profile Velocity (32b)
 *   + Profile Accel (32b) + Profile Decel (32b)
 *
 * 一帧 CANFD (约20字节) 搞定全部运动参数。
 */
int pdo_mapper_config_rpdo1_canfd_full(can_driver_t *drv, uint8_t node,
                                        uint32_t cob_id);

/* ============================================================================
 * API: 配置标准 TPDO (从站, 主站, 用于上报)
 * ============================================================================ */

/**
 * @brief 配置 TPDO1 为标准位置反馈映射
 *
 * 映射:
 *   Statusword (0x6041.00, 16bit)
 *   Position Actual (0x6064.00, 32bit)
 */
int pdo_mapper_config_tpdo1_position(can_driver_t *drv, uint8_t node,
                                     uint32_t cob_id, uint8_t trans_type);

/**
 * @brief 配置 TPDO1 为 CANFD 全状态映射
 *
 * 映射:
 *   Statusword (16b) + Position (32b) + Velocity (32b) + Current (16b)
 *   + Temperature (16b) + ErrorCode (16b)
 */
int pdo_mapper_config_tpdo1_canfd_full(can_driver_t *drv, uint8_t node,
                                        uint32_t cob_id);

/* ============================================================================
 * API: PDO 数据帧构造
 * ============================================================================ */

/**
 * @brief 构造标准 RPDO1 位置控制帧
 * @param cw    Controlword
 * @param pos   Target Position (counts)
 */
void pdo_build_rpdo1_position(uint8_t node, uint16_t cw, int32_t pos,
                              canfd_frame_t *f);

/** 构造标准 RPDO1 CANFD 全参数控制帧 */
void pdo_build_rpdo1_canfd_full(uint8_t node,
                                uint16_t cw, int32_t pos, int32_t vel,
                                int32_t accel, int32_t decel,
                                canfd_frame_t *f);

/** 解析标准 TPDO1 反馈帧 */
void pdo_parse_tpdo1_position(const canfd_frame_t *f,
                              uint16_t *sw, int32_t *pos);

#ifdef __cplusplus
}
#endif

#endif /* PDO_MAPPER_H */
