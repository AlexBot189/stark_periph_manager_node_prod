/**
 * @file can_driver_internal.h
 * @brief SocketCAN 驱动 - 内部头文件 (仅 HAL 内部使用)
 */
#ifndef CAN_DRIVER_INTERNAL_H
#define CAN_DRIVER_INTERNAL_H

#include "motor_hal_types.h"

typedef struct can_driver_s can_driver_t;

int can_driver_open(const char *iface, uint32_t arb_bitrate, uint32_t data_bitrate,
                    can_driver_t **out);
void can_driver_close(can_driver_t *drv);
int can_driver_send(can_driver_t *drv, const canfd_frame_t *frame);
int can_driver_recv(can_driver_t *drv, canfd_frame_t *frame, int timeout_ms);
int can_driver_set_filter(can_driver_t *drv, uint32_t can_id, uint32_t can_mask);
int can_driver_fd(can_driver_t *drv);
const char* can_driver_iface(can_driver_t *drv);
void can_driver_stats(can_driver_t *drv, uint64_t *tx, uint64_t *rx,
                      uint64_t *txe, uint64_t *rxe);

#endif /* CAN_DRIVER_INTERNAL_H */
