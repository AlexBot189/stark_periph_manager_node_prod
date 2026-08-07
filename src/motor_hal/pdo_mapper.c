/**
 * @file pdo_mapper.c
 * @brief 标准 CANopen PDO 映射器实现
 *
 * 通过 SDO 配置从站 PDO 映射表和通信参数。
 *
 * PDO 映射流程:
 *   1. 停用 PDO (写 comm sub0x01 bit31=1)
 *   2. 清空映射 (写 map sub0x00 = 0)
 *   3. 写入映射条目 (写 map sub0x01, sub0x02, ...)
 *   4. 设置映射数量 (写 map sub0x00 = N)
 *   5. 设置传输类型 (写 comm sub0x02)
 *   6. 启用 PDO (写 comm sub0x01 = COB-ID)
 */

#include "pdo_mapper.h"
#include <stdio.h>

/* =====================================================
 * 内部: 写 SDO + 日志
 * ===================================================== */

static int _sdo_w(can_driver_t *drv, uint8_t node,
                  uint16_t index, uint8_t subidx,
                  uint32_t value, uint8_t size, const char *desc)
{
    int ret = sdo_write_simple(drv, node, index, subidx, value, size);
    if (ret != 0) {
        fprintf(stderr, "PDO mapper: %s (0x%04X.%02X) failed: %d\n",
                desc, index, subidx, ret);
    }
    return ret;
}

#define SDO_W(idx, sub, val, sz) _sdo_w(drv, node, (idx), (sub), (uint32_t)(val), (sz), #idx)

/* =====================================================
 * RPDO1: 位置控制映射
 * ===================================================== */

int pdo_mapper_config_rpdo1_position(can_driver_t *drv, uint8_t node,
                                     uint32_t cob_id, uint8_t trans_type)
{
    int ret;

    /* 1. 停用 RPDO1 */
    ret = SDO_W(OD_RPDO1_COMM, 0x01, cob_id | 0x80000000UL, 4);
    if (ret) return ret;

    /* 2. 清映射 */
    ret = SDO_W(OD_RPDO1_MAP, 0x00, 0, 1);
    if (ret) return ret;

    /* 3. 映射条目: Controlword (0x6040.00, 16bit) */
    ret = SDO_W(OD_RPDO1_MAP, 0x01, pdo_map_entry(OD_CONTROLWORD, 0x00, 16), 4);
    if (ret) return ret;

    /* 4. 映射条目: Target Position (0x607A.00, 32bit) */
    ret = SDO_W(OD_RPDO1_MAP, 0x02, pdo_map_entry(OD_TARGET_POS, 0x00, 32), 4);
    if (ret) return ret;

    /* 5. 设置映射数量: 2 */
    ret = SDO_W(OD_RPDO1_MAP, 0x00, 2, 1);
    if (ret) return ret;

    /* 6. 设置传输类型 */
    ret = SDO_W(OD_RPDO1_COMM, 0x02, trans_type, 1);
    if (ret) return ret;

    /* 7. 启用 RPDO1 */
    ret = SDO_W(OD_RPDO1_COMM, 0x01, cob_id, 4);
    if (ret) return ret;

    printf("RPDO1 configured: COB=0x%03X, map=[Controlword@16b, TargetPos@32b]\n", cob_id);
    return 0;
}

/* =====================================================
 * RPDO1: CANFD 全参数映射
 * ===================================================== */

int pdo_mapper_config_rpdo1_canfd_full(can_driver_t *drv, uint8_t node,
                                        uint32_t cob_id)
{
    int ret;

    ret = SDO_W(OD_RPDO1_COMM, 0x01, cob_id | 0x80000000UL, 4);
    if (ret) return ret;

    ret = SDO_W(OD_RPDO1_MAP, 0x00, 0, 1);
    if (ret) return ret;

    /* 1. Controlword (0x6040, 16b) */
    ret = SDO_W(OD_RPDO1_MAP, 0x01, pdo_map_entry(OD_CONTROLWORD, 0x00, 16), 4);
    if (ret) return ret;

    /* 2. Target Position (0x607A, 32b) */
    ret = SDO_W(OD_RPDO1_MAP, 0x02, pdo_map_entry(OD_TARGET_POS, 0x00, 32), 4);
    if (ret) return ret;

    /* 3. Profile Velocity (0x6081, 32b) */
    ret = SDO_W(OD_RPDO1_MAP, 0x03, pdo_map_entry(OD_PROFILE_VEL, 0x00, 32), 4);
    if (ret) return ret;

    /* 4. Profile Acceleration (0x6083, 32b) */
    ret = SDO_W(OD_RPDO1_MAP, 0x04, pdo_map_entry(OD_PROFILE_ACCEL, 0x00, 32), 4);
    if (ret) return ret;

    /* 5. Profile Deceleration (0x6084, 32b) */
    ret = SDO_W(OD_RPDO1_MAP, 0x05, pdo_map_entry(OD_PROFILE_DECEL, 0x00, 32), 4);
    if (ret) return ret;

    /* 总共 16bit + 4×32bit = 144bit = 18字节 */
    ret = SDO_W(OD_RPDO1_MAP, 0x00, 5, 1);
    if (ret) return ret;

    ret = SDO_W(OD_RPDO1_COMM, 0x02, 255, 1);      /* 异步触发 */
    ret = SDO_W(OD_RPDO1_COMM, 0x01, cob_id, 4);

    printf("RPDO1 (CANFD) configured: COB=0x%03X, 5 entries, 18 bytes\n", cob_id);
    return ret;
}

/* =====================================================
 * TPDO1: 位置反馈映射
 * ===================================================== */

int pdo_mapper_config_tpdo1_position(can_driver_t *drv, uint8_t node,
                                     uint32_t cob_id, uint8_t trans_type)
{
    int ret;

    ret = SDO_W(OD_TPDO1_COMM, 0x01, cob_id | 0x80000000UL, 4);
    if (ret) return ret;

    ret = SDO_W(OD_TPDO1_MAP, 0x00, 0, 1);
    if (ret) return ret;

    /* 1. Statusword (0x6041, 16b) */
    ret = SDO_W(OD_TPDO1_MAP, 0x01, pdo_map_entry(OD_STATUSWORD, 0x00, 16), 4);
    if (ret) return ret;

    /* 2. Position Actual Value (0x6064, 32b) */
    ret = SDO_W(OD_TPDO1_MAP, 0x02, pdo_map_entry(OD_POSITION_ACTUAL, 0x00, 32), 4);
    if (ret) return ret;

    ret = SDO_W(OD_TPDO1_MAP, 0x00, 2, 1);
    if (ret) return ret;

    ret = SDO_W(OD_TPDO1_COMM, 0x02, trans_type, 1);
    ret = SDO_W(OD_TPDO1_COMM, 0x01, cob_id, 4);

    printf("TPDO1 configured: COB=0x%03X, map=[Statusword@16b, Position@32b]\n", cob_id);
    return ret;
}

/* =====================================================
 * TPDO1: CANFD 全状态映射
 * ===================================================== */

int pdo_mapper_config_tpdo1_canfd_full(can_driver_t *drv, uint8_t node,
                                        uint32_t cob_id)
{
    int ret;

    ret = SDO_W(OD_TPDO1_COMM, 0x01, cob_id | 0x80000000UL, 4);
    if (ret) return ret;

    ret = SDO_W(OD_TPDO1_MAP, 0x00, 0, 1);
    if (ret) return ret;

    /* 1. Statusword (16b) */
    ret = SDO_W(OD_TPDO1_MAP, 0x01, pdo_map_entry(OD_STATUSWORD, 0x00, 16), 4);
    if (ret) return ret;

    /* 2. Position Actual (32b) */
    ret = SDO_W(OD_TPDO1_MAP, 0x02, pdo_map_entry(OD_POSITION_ACTUAL, 0x00, 32), 4);
    if (ret) return ret;

    /* 3. Velocity Actual (32b) */
    ret = SDO_W(OD_TPDO1_MAP, 0x03, pdo_map_entry(OD_VELOCITY_ACTUAL, 0x00, 32), 4);
    if (ret) return ret;

    /* 4. Current Actual (16b) */
    ret = SDO_W(OD_TPDO1_MAP, 0x04, pdo_map_entry(OD_CURRENT_ACTUAL, 0x00, 16), 4);
    if (ret) return ret;

    /* 总共 16+32+32+16 = 96bit = 12字节 */
    ret = SDO_W(OD_TPDO1_MAP, 0x00, 4, 1);
    if (ret) return ret;

    ret = SDO_W(OD_TPDO1_COMM, 0x02, 254, 1);        /* 异步事件触发 */
    ret = SDO_W(OD_TPDO1_COMM, 0x01, cob_id, 4);

    printf("TPDO1 (CANFD) configured: COB=0x%03X, 4 entries, 12 bytes\n", cob_id);
    return ret;
}

/* =====================================================
 * PDO 数据帧构造 / 解析
 * ===================================================== */

void pdo_build_rpdo1_position(uint8_t node, uint16_t cw, int32_t pos,
                              canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id  = PDO_RPDO1_COB(node);
    f->dlc = 6;
    f->is_fd = true;

    /* Controlword (16bit, 小端) */
    f->data[0] = (uint8_t)(cw & 0xFF);
    f->data[1] = (uint8_t)((cw >> 8) & 0xFF);

    /* Target Position (32bit, 小端) */
    f->data[2] = (uint8_t)(pos & 0xFF);
    f->data[3] = (uint8_t)((pos >> 8) & 0xFF);
    f->data[4] = (uint8_t)((pos >> 16) & 0xFF);
    f->data[5] = (uint8_t)((pos >> 24) & 0xFF);
}

void pdo_build_rpdo1_canfd_full(uint8_t node,
                                uint16_t cw, int32_t pos, int32_t vel,
                                int32_t accel, int32_t decel,
                                canfd_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id  = PDO_RPDO1_COB(node);
    f->dlc = 18;
    f->is_fd = true;

    int off = 0;

    /* Controlword */
    f->data[off++] = (uint8_t)(cw & 0xFF);
    f->data[off++] = (uint8_t)((cw >> 8) & 0xFF);

    /* Target Position */
    f->data[off++] = (uint8_t)(pos & 0xFF);
    f->data[off++] = (uint8_t)((pos >> 8) & 0xFF);
    f->data[off++] = (uint8_t)((pos >> 16) & 0xFF);
    f->data[off++] = (uint8_t)((pos >> 24) & 0xFF);

    /* Profile Velocity */
    f->data[off++] = (uint8_t)(vel & 0xFF);
    f->data[off++] = (uint8_t)((vel >> 8) & 0xFF);
    f->data[off++] = (uint8_t)((vel >> 16) & 0xFF);
    f->data[off++] = (uint8_t)((vel >> 24) & 0xFF);

    /* Profile Acceleration */
    f->data[off++] = (uint8_t)(accel & 0xFF);
    f->data[off++] = (uint8_t)((accel >> 8) & 0xFF);
    f->data[off++] = (uint8_t)((accel >> 16) & 0xFF);
    f->data[off++] = (uint8_t)((accel >> 24) & 0xFF);

    /* Profile Deceleration */
    f->data[off++] = (uint8_t)(decel & 0xFF);
    f->data[off++] = (uint8_t)((decel >> 8) & 0xFF);
    f->data[off++] = (uint8_t)((decel >> 16) & 0xFF);
    f->data[off++] = (uint8_t)((decel >> 24) & 0xFF);
}

void pdo_parse_tpdo1_position(const canfd_frame_t *f,
                              uint16_t *sw, int32_t *pos)
{
    if (!f || f->dlc < 6) return;

    if (sw) *sw = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
    if (pos) {
        *pos = (int32_t)((uint32_t)f->data[2]
               | ((uint32_t)f->data[3] << 8)
               | ((uint32_t)f->data[4] << 16)
               | ((uint32_t)f->data[5] << 24));
    }
}
