/**
 * @file system_interface.h
 * @brief eMD SDK ,  Linux HAL 桥接层
 *
 * 将 system_interface 的 si_* 函数映射到 emd_hal。
 * eMD 驱动通过 si_* 调用，本文件将其转为 HAL 层调用。
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#ifndef SYSTEM_INTERFACE_H
#define SYSTEM_INTERFACE_H

#include "emd_hal.h"
#include "imu/inv_imu_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 板级 */
static inline int si_board_init(void) { return 0; }

/* IMU I/O 映射到 HAL */
static inline int si_io_imu_init(inv_imu_serif_type_t serif_type) {
    (void)serif_type;
    return 0;
}

static inline int si_io_imu_read_reg(uint8_t reg, uint8_t *buf, uint32_t len) {
    return emd_hal_read_reg(reg, buf, len);
}

static inline int si_io_imu_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len) {
    return emd_hal_write_reg(reg, buf, len);
}

/* GPIO 中断 */
#define SI_GPIO_INT1 1

static inline int si_init_gpio_int(unsigned int int_num,
                                   void (*int_cb)(void *context, unsigned int int_num)) {
    return emd_hal_gpio_init(int_num, (emd_gpio_cb_t)int_cb);
}

/* 定时器 */
static inline int  si_init_timers(void) { return 0; }
static inline void si_sleep_us(uint32_t us) { emd_hal_sleep_us(us); }

static inline uint64_t si_get_time_us(void) {
    return emd_hal_get_time_us();
}

/* 临界区 */
static inline void si_disable_irq(void) { emd_hal_disable_irq(); }
static inline void si_enable_irq(void)  { emd_hal_enable_irq(); }

/* 偏置存储 */
static inline int si_flash_storage_read(uint8_t *data) {
    return emd_hal_storage_read(data, 84);
}

static inline int si_flash_storage_write(const uint8_t *data) {
    return emd_hal_storage_write(data, 84);
}

/* 未使用的桩函数 */
static inline int  si_init_clkin(void)   { return 0; }
static inline int  si_uninit_clkin(void) { return 0; }
static inline int  si_config_uart_for_print(int id, int level) { (void)id; (void)level; return 0; }
static inline int  si_config_uart_for_bin(int id)   { (void)id; return 0; }
static inline int  si_get_uart_command(int id, char *cmd) { (void)id; *cmd=0; return 0; }
static inline int  si_io_akm_init(void *s)    { (void)s; return -1; }
static inline int  si_io_akm_read_reg(void *s, uint8_t r, uint8_t *b, uint32_t l) { (void)s;(void)r;(void)b;(void)l; return -1; }
static inline int  si_io_akm_write_reg(void *s, uint8_t r, const uint8_t *b, uint32_t l) { (void)s;(void)r;(void)b;(void)l; return -1; }
static inline int  si_start_gpio_fsync(uint32_t f, void (*cb)(void*)) { (void)f;(void)cb; return 0; }
static inline int  si_stop_gpio_fsync(void) { return 0; }
static inline void si_toggle_gpio_fsync(void) {}

#define SI_CHECK_RC(rc) do { \
    if (rc) { fprintf(stderr, "SI error %d at %s:%d\n", rc, __FILE__, __LINE__); return rc; } \
} while(0)

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INTERFACE_H */
