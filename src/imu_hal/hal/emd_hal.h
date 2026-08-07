/**
 * @file emd_hal.h
 * @brief 硬件抽象层 (HAL) 接口
 *
 * 将 system_interface 映射到 Linux userspace:
 *   - I2C:    /dev/i2c-N (i2c-dev)
 *   - GPIO:   /dev/gpiochipN (libgpiod)
 *   - Timer:  clock_gettime / usleep
 *   - 存储:   偏置文件持久化
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#ifndef EMD_HAL_H
#define EMD_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 板级初始化 */
int emd_hal_init(const char *i2c_dev, uint8_t imu_addr,
                 const char *gpio_chip, unsigned int gpio_line);

/* IMU I2C 读写 */
int emd_hal_read_reg(uint8_t reg, uint8_t *buf, uint32_t len);
int emd_hal_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len);

/* GPIO 中断 */
typedef void (*emd_gpio_cb_t)(void *context, unsigned int int_num);
int emd_hal_gpio_init(unsigned int int_num, emd_gpio_cb_t cb);
int emd_hal_gpio_wait(int timeout_ms);

/* 定时器 */
void     emd_hal_sleep_us(uint32_t us);
uint64_t emd_hal_get_time_us(void);

/* 临界区保护 (mutex) */
void emd_hal_disable_irq(void);
void emd_hal_enable_irq(void);

/* 偏置持久化 */
int emd_hal_storage_read(uint8_t *data, uint32_t size);
int emd_hal_storage_write(const uint8_t *data, uint32_t size);

/* 资源释放 */
void emd_hal_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EMD_HAL_H */
