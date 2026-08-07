#define _DEFAULT_SOURCE

/**
 * @file read_sensor.c
 * @brief IMU HAL 使用示例 — 读取 ICM45608 9 轴融合数据
 *
 * 链接 libimu_hal.so。
 * 用法: ./read_sensor [-i /dev/i2c-3] [-g gpiochip4] [-l 2] [-m 5]
 *
 * Copyright (c) 2026 zhiqiang.yang
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <math.h>

#include "emd_gaf.h"

static volatile int g_running = 1;
static emd_raw_sensor_t g_latest_raw;

static void raw_data_cb(const emd_raw_sensor_t *data, void *user_data)
{
    (void)user_data;
    g_latest_raw = *data;
}

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
    printf("  0: HRC 200Hz, MAG 100Hz\n");
    printf("  1: HRC 100Hz, MAG 50Hz\n");
    printf("  2: HRC 100Hz, GYRO OFF, MAG 50Hz\n");
    printf("  3: GAF 200Hz, MAG 50Hz (fusion)\n");
    printf("  4: GAF 50Hz,  MAG 50Hz (fusion)\n");
    printf("  5: GAF 50Hz,  MAG 50Hz (fusion, default)\n");
    printf("  6: GAF 50Hz,  MAG 50Hz (fusion, 400Hz sensor)\n");
    printf("  7: GAF 50Hz,  MAG 50Hz (fusion, 800Hz sensor)\n");
    printf("  8: GAF 50Hz,  MAG OFF  (fusion)\n");
    printf("  9: GAF 50Hz,  MAG 50Hz, GYRO OFF (fusion)\n");
}

int main(int argc, char *argv[])
{
    const char *i2c_dev    = "/dev/i2c-3";
    const char *gpio_chip  = "gpiochip4";
    unsigned int gpio_line = 2;
    int op_mode = 5;
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

    printf("=== IMU HAL Read Sensor Example ===\n");
    printf("I2C: %s, GPIO: %s line %u, Mode: %d\n",
           i2c_dev, gpio_chip, gpio_line, op_mode);

    /* 1. 创建实例 */
    emd_gaf_t *gaf = emd_gaf_create();
    if (!gaf) {
        fprintf(stderr, "Failed to create IMU HAL instance\n");
        return 1;
    }

    /* 2. 初始化 */
    int rc = emd_gaf_init(gaf, i2c_dev, gpio_chip, gpio_line, op_mode);
    if (rc != 0) {
        fprintf(stderr, "IMU HAL init failed: rc=%d\n", rc);
        emd_gaf_destroy(gaf);
        return 1;
    }
    printf("IMU HAL initialized OK\n");

    /* 3. 启动后台线程 */
    emd_gaf_set_raw_data_callback(gaf, raw_data_cb, NULL);
    rc = emd_gaf_start(gaf);
    if (rc != 0) {
        fprintf(stderr, "IMU HAL start failed: rc=%d\n", rc);
        emd_gaf_destroy(gaf);
        return 1;
    }
    printf("Background thread started\n");

    /* 4. 等待融合收敛 */
    printf("Waiting 2s for fusion to converge...\n");
    usleep(2000000);

    /* 5. 读取循环 */
    printf("\nReading sensor data (Ctrl+C to stop)...\n\n");

    while (g_running) {
        emd_output_t out;
        emd_imu_data_t accel, gyro;

        /* 融合输出 (quat/mag/heading, GAF ODR, 保留最近有效值) */
        if (emd_gaf_get_output(gaf, &out) == 0) {

            /* 四元数 -> 欧拉角 (ZYX: Yaw-Pitch-Roll), rad -> deg */
            float qw = out.quat_w, qx = out.quat_x, qy = out.quat_y, qz = out.quat_z;
            float yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * 57.29578f;
            float sp    = 2.0f*(qw*qy - qz*qx);
            if (sp > 1.0f) sp = 1.0f; else if (sp < -1.0f) sp = -1.0f;
            float pitch = asinf(sp) * 57.29578f;
            float roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 57.29578f;

            printf("ts=%llu euler=(yaw=%.1f pitch=%.1f roll=%.1f) heading=%.1f quat=(%.3f,%.3f,%.3f,%.3f) acc=(%.3f,%.3f,%.3f) gyr=(%.1f,%.1f,%.1f) mag=(%.1f,%.1f,%.1f) temp=%.1fC sta=%d ga=%d ma=%d\n",
                   (unsigned long long)out.timestamp_us,
                   yaw, pitch, roll,
                   out.heading_deg,
                   out.quat_w, out.quat_x, out.quat_y, out.quat_z,
                   g_latest_raw.accel_x, g_latest_raw.accel_y, g_latest_raw.accel_z,
                   g_latest_raw.gyro_x, g_latest_raw.gyro_y, g_latest_raw.gyro_z,
                   out.mag_x, out.mag_y, out.mag_z,
                   g_latest_raw.temp_c,
                   out.stationary,
                   out.gyr_accuracy, out.mag_accuracy);
        }

        /* 原始 IMU 数据 (通过 emd_gaf_get_imu, sensor ODR) */
        emd_gaf_get_imu(gaf, &accel, &gyro);
    }

    /* 6. 清理 */
    printf("\nStopping...\n");
    emd_gaf_stop(gaf);
    emd_gaf_destroy(gaf);

    printf("Done.\n");
    return 0;
}
