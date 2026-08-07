/**
 * @file emd_hal.c
 * @brief HAL Linux userspace 实现
 *
 * 硬件: I2C3, 400kHz, ICM45608 addr 0x68
 *        INT1: GPIO4_PA2
 *
 * 依赖: linux/i2c-dev.h, libgpiod, pthread
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "emd_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>

/* Linux I2C dev */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* libgpiod */
#include <gpiod.h>

/* 内部状态 */
static int              g_i2c_fd       = -1;
static uint8_t          g_imu_addr     = 0x68;

static struct gpiod_chip      *g_gpio_chip   = NULL;
static struct gpiod_line      *g_gpio_line   = NULL;
static emd_gpio_cb_t           g_gpio_cb     = NULL;
static void                   *g_gpio_context = NULL;
static unsigned int             g_gpio_num    = 0;

static pthread_mutex_t  g_irq_mutex    = PTHREAD_MUTEX_INITIALIZER;
static int              g_initialized  = 0;

static const char      *g_bias_file    = "./imu_bias.bin";

/*
 * I2C 操作
 */

int emd_hal_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data ioctl_data;

    msgs[0].addr  = g_imu_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = g_imu_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = buf;

    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(g_i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        fprintf(stderr, "EMD_HAL: I2C read reg=0x%02x len=%u failed: %s\n",
                reg, len, strerror(errno));
        return -1;
    }
    return 0;
}

int emd_hal_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data ioctl_data;
    uint8_t tx[1 + len];

    tx[0] = reg;
    if (len > 0 && buf)
        memcpy(tx + 1, buf, len);

    msgs[0].addr  = g_imu_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1 + len;
    msgs[0].buf   = tx;

    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 1;

    if (ioctl(g_i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        fprintf(stderr, "EMD_HAL: I2C write reg=0x%02x len=%u failed: %s\n",
                reg, len, strerror(errno));
        return -1;
    }
    return 0;
}

/*
 * GPIO 中断 (libgpiod)
 */

int emd_hal_gpio_init(unsigned int int_num, emd_gpio_cb_t cb)
{
    g_gpio_num = int_num;
    g_gpio_cb  = cb;
    return 0;
}

int emd_hal_gpio_wait(int timeout_ms)
{
    struct gpiod_line_event event;
    int ret;

    if (!g_gpio_line)
        return -1;

    ret = gpiod_line_event_wait(g_gpio_line,
            &(struct timespec){ .tv_sec = timeout_ms / 1000,
                                .tv_nsec = (timeout_ms % 1000) * 1000000L });
    if (ret <= 0)
        return ret;

    ret = gpiod_line_event_read(g_gpio_line, &event);
    if (ret < 0)
        return ret;

    if (g_gpio_cb)
        g_gpio_cb(g_gpio_context, g_gpio_num);

    return 1;
}

/*
 * 定时器
 */

void emd_hal_sleep_us(uint32_t us)
{
    usleep(us);
}

uint64_t emd_hal_get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

/*
 * 临界区保护
 */

void emd_hal_disable_irq(void)
{
    pthread_mutex_lock(&g_irq_mutex);
}

void emd_hal_enable_irq(void)
{
    pthread_mutex_unlock(&g_irq_mutex);
}

/*
 * 偏置文件持久化
 */

int emd_hal_storage_read(uint8_t *data, uint32_t size)
{
    FILE *f = fopen(g_bias_file, "rb");
    if (!f)
        return -1;
    size_t n = fread(data, 1, size, f);
    fclose(f);
    return (n == size) ? 0 : -1;
}

int emd_hal_storage_write(const uint8_t *data, uint32_t size)
{
    FILE *f = fopen(g_bias_file, "wb");
    if (!f)
        return -1;
    size_t n = fwrite(data, 1, size, f);
    fclose(f);
    return (n == size) ? 0 : -1;
}

/*
 * 初始化 / 反初始化
 */

int emd_hal_init(const char *i2c_dev, uint8_t imu_addr,
                 const char *gpio_chip, unsigned int gpio_line)
{
    if (g_initialized)
        return 0;

    /* I2C */
    g_i2c_fd = open(i2c_dev, O_RDWR);
    if (g_i2c_fd < 0) {
        fprintf(stderr, "EMD_HAL: open %s failed: %s\n",
                i2c_dev, strerror(errno));
        return -1;
    }
    g_imu_addr = imu_addr;
    printf("EMD_HAL: I2C %s ready (addr=0x%02x)\n", i2c_dev, imu_addr);

    /* GPIO */
    /* 接受 "gpiochip4" 或 "/dev/gpiochip4" 两种格式 */
    char path[64];
    if (gpio_chip[0] == '/')
        snprintf(path, sizeof(path), "%s", gpio_chip);
    else
        snprintf(path, sizeof(path), "/dev/%s", gpio_chip);

    g_gpio_chip = gpiod_chip_open(path);
    if (!g_gpio_chip) {
        fprintf(stderr, "EMD_HAL: gpiod_chip_open(%s) failed: %s\n",
                path, strerror(errno));
        close(g_i2c_fd);
        return -1;
    }

    g_gpio_line = gpiod_chip_get_line(g_gpio_chip, gpio_line);
    if (!g_gpio_line) {
        fprintf(stderr, "EMD_HAL: gpiod get line %u failed\n", gpio_line);
        gpiod_chip_close(g_gpio_chip);
        close(g_i2c_fd);
        return -1;
    }

    /* INT1: DTS 配置为 IRQ_TYPE_LEVEL_LOW，ICM45608 INT 引脚脉冲低有效。
     * 硬件将 INT 引脚配置为 push-pull, active high。
     * 这里用上升沿触发。 */
    if (gpiod_line_request_rising_edge_events(g_gpio_line, "emd-gaf") < 0) {
        fprintf(stderr, "EMD_HAL: gpiod request event failed: %s\n",
                strerror(errno));
        gpiod_chip_close(g_gpio_chip);
        close(g_i2c_fd);
        return -1;
    }
    printf("EMD_HAL: GPIO %s line %u ready\n", path, gpio_line);

    g_initialized = 1;
    return 0;
}

void emd_hal_deinit(void)
{
    if (g_gpio_line) {
        gpiod_line_release(g_gpio_line);
        g_gpio_line = NULL;
    }
    if (g_gpio_chip) {
        gpiod_chip_close(g_gpio_chip);
        g_gpio_chip = NULL;
    }
    if (g_i2c_fd >= 0) {
        close(g_i2c_fd);
        g_i2c_fd = -1;
    }
    g_initialized = 0;
}
