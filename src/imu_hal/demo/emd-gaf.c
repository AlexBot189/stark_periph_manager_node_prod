/**
 * @file emd-gaf.c
 * @brief IMU HAL 独立可执行程序 — 链接 libimu_hal.so
 *
 * 用法: ./emd-gaf -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5
 *
 * 流程: create ,  init ,  start ,  循环读取输出 ,  stop ,  destroy
 *
 * Copyright (c) 2026 zhiqiang.yang
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

#include "emd_gaf.h"

static volatile int g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

static void usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("  -i <dev>     I2C device   (default: /dev/i2c-3)\n");
    printf("  -g <chip>    GPIO chip    (default: gpiochip4)\n");
    printf("  -l <line>    GPIO line    (default: 2)\n");
    printf("  -m <mode>    opmode 0-9   (default: 5)\n");
    printf("  -h           help\n\n");
    printf("Operation modes:\n");
    printf("  0: HRC 200Hz, MAG 100Hz (no fusion)\n");
    printf("  1: HRC 100Hz, MAG 50Hz  (no fusion)\n");
    printf("  2: HRC 100Hz, GYRO OFF  (no fusion)\n");
    printf("  3: GAF 200Hz, MAG 50Hz  (fusion)\n");
    printf("  4: GAF 50Hz,  MAG 50Hz  (fusion)\n");
    printf("  5: GAF 50Hz,  MAG 50Hz  (fusion, default)\n");
    printf("  6: GAF 50Hz,  MAG 50Hz  (fusion, 400Hz sensor)\n");
    printf("  7: GAF 50Hz,  MAG 50Hz  (fusion, 800Hz sensor)\n");
    printf("  8: GAF 50Hz,  MAG OFF   (fusion)\n");
    printf("  9: GAF 50Hz,  MAG 50Hz, GYRO OFF (fusion)\n");
}

int main(int argc, char *argv[])
{
    const char *i2c_dev    = "/dev/i2c-3";
    const char *gpio_chip  = "gpiochip4";
    unsigned int gpio_line = 2;
    int op_mode            = 5;
    int opt;

    while ((opt = getopt(argc, argv, "i:g:l:m:h")) != -1) {
        switch (opt) {
        case 'i': i2c_dev   = optarg; break;
        case 'g': gpio_chip = optarg; break;
        case 'l': gpio_line = (unsigned int)atoi(optarg); break;
        case 'm': op_mode   = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (op_mode < 0 || op_mode > 9) {
        fprintf(stderr, "Invalid opmode %d (0-9)\n", op_mode);
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    fprintf(stderr, "=== IMU HAL Demo ===\n");
    fprintf(stderr, "[I] I2C: %s, GPIO: %s line %u, Mode: %d\n",
            i2c_dev, gpio_chip, gpio_line, op_mode);

    /* 1. 创建实例 */
    emd_gaf_t *gaf = emd_gaf_create();
    if (!gaf) {
        fprintf(stderr, "[E] emd_gaf_create() failed\n");
        return 1;
    }

    /* 2. 初始化 */
    if (emd_gaf_init(gaf, i2c_dev, gpio_chip, gpio_line, op_mode) != 0) {
        fprintf(stderr, "[E] emd_gaf_init() failed\n");
        emd_gaf_destroy(gaf);
        return 1;
    }
    fprintf(stderr, "[I] IMU HAL initialized OK\n");

    /* 3. 启动后台采集 */
    if (emd_gaf_start(gaf) != 0) {
        fprintf(stderr, "[E] emd_gaf_start() failed\n");
        emd_gaf_destroy(gaf);
        return 1;
    }
    fprintf(stderr, "[I] Background thread started, waiting for data...\n");

    /* 4. 等待融合收敛 */
    usleep(2000000);

    /* 5. 主循环 */
    fprintf(stderr, "[I] Entering read loop (Ctrl+C to stop)...\n\n");

    int is_first  = 1;
    int line_cnt  = 0;

    while (g_running) {
        emd_output_t out;

        int ret = emd_gaf_get_output(gaf, &out);
        if (ret == 0) {
            if (is_first || line_cnt % 20 == 0) {
                printf("%-12s %8s %12s %12s %12s %12s %12s %12s %12s %12s %12s %6s %4s %4s %4s %4s\n",
                       "Time(s)", "Heading°", "QuatW", "QuatX", "QuatY", "QuatZ",
                       "AccX(g)", "AccY(g)", "AccZ(g)",
                       "GyrX", "GyrY", "GyrZ",
                       "Temp°C", "Sta", "GA", "MA");
                is_first = 0;
            }
            line_cnt++;

            printf("%-12.3f %8.1f %+11.3f %+11.3f %+11.3f %+11.3f %+11.3f %+11.3f %+11.3f %+11.2f %+11.2f %+11.2f %5.1f  %d  %d/%d\n",
                   out.timestamp_us / 1000000.0f,
                   out.heading_deg,
                   out.quat_w, out.quat_x, out.quat_y, out.quat_z,
                   out.accel_x, out.accel_y, out.accel_z,
                   out.gyro_x, out.gyro_y, out.gyro_z,
                   out.temp_c,
                   out.stationary,
                   out.gyr_accuracy, out.mag_accuracy);
        }

        usleep(10000);
    }

    /* 6. 清理 */
    fprintf(stderr, "\n[I] Stopping...\n");
    emd_gaf_stop(gaf);
    emd_gaf_destroy(gaf);

    fprintf(stderr, "[I] Done.\n");
    return 0;
}
