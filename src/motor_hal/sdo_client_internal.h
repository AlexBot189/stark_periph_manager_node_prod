/**
 * @file sdo_client_internal.h
 * @brief SDO 客户端 - 内部头文件 (v2: 队列模式)
 */
#ifndef SDO_CLIENT_INTERNAL_H
#define SDO_CLIENT_INTERNAL_H

#include "motor_hal_types.h"
#include "can_driver_internal.h"

/* =====================================================
 * 队列管理 (接收线程调用)
 * ===================================================== */

void sdo_queue_init(void);
void sdo_queue_destroy(void);
void sdo_push_response(const canfd_frame_t *f);

/* =====================================================
 * SDO 读写 (不变)
 * ===================================================== */

int sdo_write(can_driver_t *drv, uint8_t node,
              uint16_t index, uint8_t subidx,
              uint32_t value, uint8_t size_bytes,
              int retry_count, int timeout_ms);

int sdo_read(can_driver_t *drv, uint8_t node,
             uint16_t index, uint8_t subidx,
             uint32_t *value,
             int retry_count, int timeout_ms);

int sdo_write_simple(can_driver_t *drv, uint8_t node,
                     uint16_t index, uint8_t subidx,
                     uint32_t value, uint8_t size_bytes);

int sdo_read_simple(can_driver_t *drv, uint8_t node,
                    uint16_t index, uint8_t subidx,
                    uint32_t *value);

#endif /* SDO_CLIENT_INTERNAL_H */
