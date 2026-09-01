#define _DEFAULT_SOURCE

/**
 * @file read_sensor.c
 * @brief IMU + 气压计 测试程序 (可单独测试)
 *
 * 用法:
 *   只测 IMU (I2C):  ./imu_read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5
 *   只测 IMU (SPI):  ./imu_read_sensor -b spi -s /dev/spidev0.0 -g gpiochip4 -l 2 -m 5
 *   只测气压计:      ./imu_read_sensor -B bmp5xy -o 8 -p 2 -f 3
 *   两者都测:        ./imu_read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5 -B bmp5xy -o 8 -p 2 -f 3
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
#include <dirent.h>
#include <time.h>

#include "emd_gaf.h"

static volatile int g_running = 1;
static emd_raw_sensor_t g_latest_raw;

/* ── 气压计配置 ── */
static int  g_baro_enabled = 0;
static char g_baro_sysfs[256] = {0};
static int  g_baro_odr   = 8;     /* ODR: 8=100Hz, 15=50Hz, 0=240Hz */
static int  g_baro_osr_p = 2;     /* 气压过采样: 0=1x ... 7=128x */
static int  g_baro_iir   = 3;     /* IIR: 0=bypass 1~7=coeff(1,3,7,15,31,63,127) */

#define BARO_OSR_T_FIXED  6        /* 温度过采样固定 64x, 不暴露参数 */

/* ── 气压计 ODR 统计 ── */
static unsigned long long g_baro_prev_pa = 0;
static int  g_baro_changes = 0;
static struct timespec g_baro_t1 = {0}, g_baro_t2 = {0};

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
    printf("\n");
    printf("  IMU:\n");
    printf("    -b <type>   bus type      i2c (default) | spi\n");
    printf("    -i <dev>     I2C device   (default: /dev/i2c-3)\n");
    printf("    -s <dev>     SPI device   (default: /dev/spidev0.0)\n");
    printf("    -p <hz>      SPI speed Hz (default: 8000000)\n");
    printf("    -o <mode>    SPI mode 0-3 (default: 0)\n");
    printf("    -g <chip>    GPIO chip    (default: gpiochip4)\n");
    printf("    -l <line>    GPIO line    (default: 2)\n");
    printf("    -m <mode>    opmode 0-9   (default: 5)\n");
    printf("    -x <map>     mount map    (default: Z,-1,X,-1,Y,1)\n");
    printf("\n");
    printf("  Barometer (BMP581):\n");
    printf("    -B <name>    barometer input name (e.g. bmp5xy)\n");
    printf("    -O <odr>     BMP581 ODR enum (default: 8=100Hz, 0=240Hz, 15=50Hz)\n");
    printf("    -P <osr_p>   BMP581 pressure OSR (default: 2=4x)\n");
    printf("    -F <iir>     BMP581 IIR filter (default: 3=coeff7, 0=bypass)\n");
    printf("\n");
    printf("    -h           help\n\n");
    printf("Examples:\n");
    printf("  IMU (I2C):    %s -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5\n", prog);
    printf("  IMU (SPI):    %s -b spi -s /dev/spidev0.0 -g gpiochip4 -l 2 -m 5\n", prog);
    printf("  Baro only:    %s -B bmp5xy -O 8 -P 2 -F 3\n", prog);
    printf("  Both:         %s -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5 -B bmp5xy -O 8 -P 2 -F 3\n", prog);
    printf("\n");
    printf("mount map: robot_x,robot_x_sign,robot_y,robot_y_sign,robot_z,robot_z_sign\n");
    printf("  axis in {X,Y,Z}, sign in {1,-1}\n");
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
    printf("BMP581 ODR: 0=240Hz 8=100Hz 15=50Hz 31=0.125Hz (32 levels)\n");
    printf("BMP581 OSR_P: 0=1x 1=2x 2=4x 3=8x 4=16x 5=32x 6=64x 7=128x\n");
    printf("BMP581 IIR: 0=bypass 1=coeff1 2=coeff3 3=coeff7 4=coeff15 7=coeff127\n");
}

/* ── 解析 mount 映射 ── */
static int parse_mount(const char *s, int8_t mount_axis[3], int8_t mount_sign[3])
{
    char buf[64];
    char *tok, *save = NULL;
    int i;
    if (!s || !s[0]) return -1;
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (i = 0; i < 3; i++) {
        tok = strtok_r(i == 0 ? buf : NULL, ",", &save);
        if (!tok) return -1;
        if (tok[0] == 'x' || tok[0] == 'X')      mount_axis[i] = 0;
        else if (tok[0] == 'y' || tok[0] == 'Y') mount_axis[i] = 1;
        else if (tok[0] == 'z' || tok[0] == 'Z') mount_axis[i] = 2;
        else return -1;
        tok = strtok_r(NULL, ",", &save);
        if (!tok) return -1;
        int sv = atoi(tok);
        if (sv != 1 && sv != -1) return -1;
        mount_sign[i] = sv;
    }
    return 0;
}

/* ── 气压计 sysfs 辅助 ── */
static int find_baro_sysfs(const char *name, char *out_dir, int out_len)
{
    DIR *d = opendir("/sys/class/input");
    if (!d) return -1;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "input", 5) != 0) continue;
        char path[300];
        snprintf(path, sizeof(path), "/sys/class/input/%s/name", e->d_name);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char buf[64] = {0};
        (void)fread(buf, 1, sizeof(buf)-1, f);
        fclose(f);
        int n = strlen(buf);
        while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == ' ')) buf[--n] = 0;
        if (strcmp(buf, name) == 0) {
            snprintf(out_dir, out_len, "/sys/class/input/%s", e->d_name);
            closedir(d);
            return 0;
        }
    }
    closedir(d);
    return -1;
}

static void sysfs_write(const char *dir, const char *node, const char *value)
{
    char path[300];
    snprintf(path, sizeof(path), "%s/%s", dir, node);
    FILE *f = fopen(path, "w");
    if (f) { fputs(value, f); fclose(f); }
}

static void baro_init(const char *name)
{
    if (find_baro_sysfs(name, g_baro_sysfs, sizeof(g_baro_sysfs)) != 0) {
        fprintf(stderr, "Barometer '%s' not found\n", name);
        return;
    }

    /* 初始化序列 */
    sysfs_write(g_baro_sysfs, "sensor_init", "1");
    usleep(50000);

    char osr_buf[32];
    snprintf(osr_buf, sizeof(osr_buf), "%d %d 1 %d", BARO_OSR_T_FIXED, g_baro_osr_p, g_baro_odr);
    sysfs_write(g_baro_sysfs, "osr_odr_press_config", osr_buf);

    /* IIR 滤波 */
    if (g_baro_iir > 0) {
        char iir_buf[32];
        snprintf(iir_buf, sizeof(iir_buf), "1 1 0 %d %d", g_baro_iir, g_baro_iir);
        sysfs_write(g_baro_sysfs, "iir_config", iir_buf);
        usleep(20000);
    }

    /* 连续模式 */
    sysfs_write(g_baro_sysfs, "power_mode", "3");
    usleep(50000);

    g_baro_enabled = 1;
    printf("Barometer '%s' init: osr_t=%d(fixed) osr_p=%d odr=%d iir=%d power=continuous\n",
           name, BARO_OSR_T_FIXED, g_baro_osr_p, g_baro_odr, g_baro_iir);
}

static void baro_read(void)
{
    if (!g_baro_enabled) return;
    char path[300];
    snprintf(path, sizeof(path), "%s/sensor_data", g_baro_sysfs);
    FILE *f = fopen(path, "r");
    if (!f) return;
    char buf[128] = {0};
    (void)fread(buf, 1, sizeof(buf)-1, f);
    fclose(f);

    unsigned long long press_pa = 0;
    long temp_c = 0;
    if (sscanf(buf, "Pressure: %llu Pa Temperature: %ld deg C", &press_pa, &temp_c) == 2) {
        if (press_pa != g_baro_prev_pa && g_baro_prev_pa != 0) {
            if (g_baro_changes == 0)
                clock_gettime(CLOCK_MONOTONIC, &g_baro_t1);
            clock_gettime(CLOCK_MONOTONIC, &g_baro_t2);
            g_baro_changes++;
        }
        g_baro_prev_pa = press_pa;

        double actual_odr = 0;
        if (g_baro_changes >= 2) {
            double dur_us = (g_baro_t2.tv_sec - g_baro_t1.tv_sec) * 1000000.0
                          + (g_baro_t2.tv_nsec - g_baro_t1.tv_nsec) / 1000.0;
            actual_odr = g_baro_changes * 1000000.0 / dur_us;
        }

        struct timespec ts_now;
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        unsigned long long ts_us = (unsigned long long)ts_now.tv_sec * 1000000ULL
                                   + (unsigned long long)ts_now.tv_nsec / 1000ULL;
        printf("BARO  ts=%llu P=%lluPa T=%ldC odr_meas=%.1fHz (changes=%d)\n",
               ts_us, press_pa, temp_c, actual_odr, g_baro_changes);
    }
}

static void baro_print_summary(void)
{
    if (!g_baro_enabled || g_baro_changes < 2) return;
    double dur_us = (g_baro_t2.tv_sec - g_baro_t1.tv_sec) * 1000000.0
                  + (g_baro_t2.tv_nsec - g_baro_t1.tv_nsec) / 1000.0;
    double odr = g_baro_changes * 1000000.0 / dur_us;
    printf("\n=== Barometer ODR Summary ===\n");
    printf("Config ODR: %d, Measured ODR: %.1f Hz (changes=%d, duration=%.0fus)\n",
           g_baro_odr, odr, g_baro_changes, dur_us);
}

static void run_imu(const char *bus_type, const char *i2c_dev, const char *spi_dev,
                    uint32_t spi_speed, uint8_t spi_mode,
                    const char *gpio_chip, unsigned int gpio_line, int op_mode,
                    const int8_t mount_axis[3], const int8_t mount_sign[3])
{
    int use_spi = (strcmp(bus_type, "spi") == 0);

    printf("=== IMU Read Sensor ===\n");
    if (use_spi) {
        printf("Bus: SPI %s (mode=%u speed=%uHz), GPIO: %s line %u, Mode: %d, Mount: robot=(%c%c,%c%c,%c%c)\n",
               spi_dev, spi_mode, spi_speed, gpio_chip, gpio_line, op_mode,
               mount_sign[0] < 0 ? '-' : '+', "XYZ"[mount_axis[0]],
               mount_sign[1] < 0 ? '-' : '+', "XYZ"[mount_axis[1]],
               mount_sign[2] < 0 ? '-' : '+', "XYZ"[mount_axis[2]]);
    } else {
        printf("Bus: I2C %s, GPIO: %s line %u, Mode: %d, Mount: robot=(%c%c,%c%c,%c%c)\n",
               i2c_dev, gpio_chip, gpio_line, op_mode,
               mount_sign[0] < 0 ? '-' : '+', "XYZ"[mount_axis[0]],
               mount_sign[1] < 0 ? '-' : '+', "XYZ"[mount_axis[1]],
               mount_sign[2] < 0 ? '-' : '+', "XYZ"[mount_axis[2]]);
    }

    emd_gaf_t *gaf = emd_gaf_create();
    if (!gaf) {
        fprintf(stderr, "Failed to create IMU HAL instance\n");
        return;
    }

    emd_gaf_cfg_t cfg = {0};
    cfg.gpio_chip = gpio_chip;
    cfg.gpio_line = gpio_line;
    cfg.op_mode   = op_mode;

    if (use_spi) {
        cfg.if_type      = EMD_GAF_IF_SPI;
        cfg.spi_dev      = spi_dev;
        cfg.spi_speed_hz = spi_speed;
        cfg.spi_mode     = spi_mode;
    } else {
        cfg.if_type      = EMD_GAF_IF_I2C;
        cfg.i2c_dev      = i2c_dev;
    }

    cfg.mount_axis[0] = mount_axis[0];
    cfg.mount_axis[1] = mount_axis[1];
    cfg.mount_axis[2] = mount_axis[2];
    cfg.mount_sign[0] = mount_sign[0];
    cfg.mount_sign[1] = mount_sign[1];
    cfg.mount_sign[2] = mount_sign[2];

    int rc = emd_gaf_init(gaf, &cfg);
    if (rc != 0) {
        fprintf(stderr, "IMU HAL init failed: rc=%d\n", rc);
        emd_gaf_destroy(gaf);
        return;
    }
    printf("IMU HAL initialized OK\n");

    emd_gaf_set_raw_data_callback(gaf, raw_data_cb, NULL);
    rc = emd_gaf_start(gaf);
    if (rc != 0) {
        fprintf(stderr, "IMU HAL start failed: rc=%d\n", rc);
        emd_gaf_destroy(gaf);
        return;
    }
    printf("Background thread started\n");

    printf("Waiting 2s for fusion to converge...\n");
    usleep(2000000);

    printf("\nReading sensor data (Ctrl+C to stop)...\n\n");

    while (g_running) {
        emd_output_t out;
        emd_imu_data_t accel, gyro;

        if (emd_gaf_get_output(gaf, &out) == 0) {
            float qw = out.quat_w, qx = out.quat_x, qy = out.quat_y, qz = out.quat_z;
            float yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * 57.29578f;
            float sp    = 2.0f*(qw*qy - qz*qx);
            if (sp > 1.0f) sp = 1.0f; else if (sp < -1.0f) sp = -1.0f;
            float pitch = asinf(sp) * 57.29578f;
            float roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 57.29578f;

            printf("IMU  ts=%llu euler=(yaw=%.1f pitch=%.1f roll=%.1f) heading=%.1f quat=(%.3f,%.3f,%.3f,%.3f) acc=(%.3f,%.3f,%.3f) gyr=(%.1f,%.1f,%.1f) mag=(%.1f,%.1f,%.1f) temp=%.1fC sta=%d ga=%d ma=%d\n",
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

        baro_read();

        emd_gaf_get_imu(gaf, &accel, &gyro);
        usleep(5000);
    }

    printf("\nStopping IMU...\n");
    emd_gaf_stop(gaf);
    emd_gaf_destroy(gaf);
}

static void run_baro_only(void)
{
    printf("=== Barometer Only Mode ===\n");
    printf("osr_t=%d(fixed) osr_p=%d odr=%d iir=%d\n\n",
           BARO_OSR_T_FIXED, g_baro_osr_p, g_baro_odr, g_baro_iir);
    printf("Reading barometer data (Ctrl+C to stop)...\n\n");

    while (g_running) {
        baro_read();
        usleep(2000);
    }

    baro_print_summary();
}

int main(int argc, char *argv[])
{
    /* IMU 参数 */
    const char *bus_type   = "i2c";
    const char *i2c_dev    = "/dev/i2c-3";
    const char *spi_dev    = "/dev/spidev0.0";
    uint32_t   spi_speed   = 8000000;
    uint8_t    spi_mode    = 0;
    const char *gpio_chip  = "gpiochip4";
    unsigned int gpio_line = 2;
    int op_mode = 5;
    int8_t mount_axis[3] = {2, 0, 1};
    int8_t mount_sign[3] = {-1, -1, 1};

    /* 气压计参数 */
    const char *baro_name = NULL;

    int opt;
    while ((opt = getopt(argc, argv, "b:i:s:p:o:g:l:m:x:B:O:P:F:h")) != -1) {
        switch (opt) {
        case 'b': bus_type  = optarg; break;
        case 'i': i2c_dev   = optarg; break;
        case 's': spi_dev   = optarg; break;
        case 'p': spi_speed = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 'o': spi_mode  = (uint8_t)atoi(optarg); break;
        case 'g': gpio_chip = optarg; break;
        case 'l': gpio_line = (unsigned int)atoi(optarg); break;
        case 'm': op_mode   = atoi(optarg); break;
        case 'x':
            if (parse_mount(optarg, mount_axis, mount_sign) != 0) {
                fprintf(stderr, "Invalid mount spec '%s'\n", optarg);
                usage(argv[0]);
                return 1;
            }
            break;
        case 'B': baro_name    = optarg; break;
        case 'O': g_baro_odr   = atoi(optarg); break;
        case 'P': g_baro_osr_p = atoi(optarg); break;
        case 'F': g_baro_iir   = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 初始化气压计 (如果指定了 -B) */
    if (baro_name) {
        baro_init(baro_name);
    }

    /* 判断是否只测气压计: 有 -B 但没有 IMU 参数 */
    int has_imu_args = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "-g") == 0 ||
            strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "-m") == 0 ||
            strcmp(argv[i], "-x") == 0 || strcmp(argv[i], "-b") == 0 ||
            strcmp(argv[i], "-s") == 0) {
            has_imu_args = 1;
            break;
        }
    }

    if (baro_name && !has_imu_args) {
        run_baro_only();
        return 0;
    }

    /* 跑 IMU (有 -B 时同时跑气压计) */
    if (op_mode < 0 || op_mode > 9) {
        fprintf(stderr, "Invalid opmode %d (0-9)\n", op_mode);
        return 1;
    }

    if (spi_mode > 3) {
        fprintf(stderr, "Invalid SPI mode %d (0-3)\n", spi_mode);
        return 1;
    }

    if (strcmp(bus_type, "spi") != 0 && strcmp(bus_type, "i2c") != 0) {
        fprintf(stderr, "Invalid bus type '%s' (expected: i2c or spi)\n", bus_type);
        return 1;
    }

    run_imu(bus_type, i2c_dev, spi_dev, spi_speed, spi_mode,
            gpio_chip, gpio_line, op_mode, mount_axis, mount_sign);

    baro_print_summary();

    printf("Done.\n");
    return 0;
}
