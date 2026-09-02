#define _DEFAULT_SOURCE

/**
 * @file read_sensor.c
 * @brief IMU (ICM45608 / BHI360) + 气压计 测试程序 (可单独测试)
 *
 * 用法:
 *   ICM45608 (I2C):  ./imu_read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5
 *   ICM45608 (SPI):  ./imu_read_sensor -b spi -s /dev/spidev0.0 -g gpiochip4 -l 2 -m 5
 *   BHI360 (IIO):    ./imu_read_sensor -I bhi360
 *   只测气压计:      ./imu_read_sensor -B bmp5xy -O 8 -P 2 -F 3
 *   ICM + 气压计:    ./imu_read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5 -B bmp5xy -O 8 -P 2 -F 3
 *   BHI360 + 气压计: ./imu_read_sensor -I bhi360 -B bmp5xy -O 8 -P 2 -F 3
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
#include "imu_mount.h"

static volatile int g_running = 1;
static emd_raw_sensor_t g_latest_raw;

/* ── 气压计配置 ── */
static int  g_baro_enabled = 0;
static char g_baro_sysfs[256] = {0};
static int  g_baro_odr   = 8;
static int  g_baro_osr_p = 2;
static int  g_baro_iir   = 3;
#define BARO_OSR_T_FIXED  6

/* ── 气压计 ODR 统计 ── */
static unsigned long long g_baro_prev_pa = 0;
static int  g_baro_changes = 0;
static struct timespec g_baro_t1 = {0}, g_baro_t2 = {0};

/* ── BHI360 IIO 模式 ── */
static int  g_bhi360_enabled = 0;
static char g_bhi360_iio_path[256] = {0};
static int  g_bhi360_sample_ms = 2;  /* 轮询周期, 默认 2ms (500Hz) */
static int8_t g_bhi360_mount_axis[3] = {0, 1, 2};  /* 默认单位矩阵 */
static int8_t g_bhi360_mount_sign[3] = {1, 1, 1};
/* ── BHI360 频率统计 ── */
static int  g_bhi360_changes = 0;
static struct timespec g_bhi360_t1 = {0}, g_bhi360_t2 = {0};

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
    printf("  ICM45608 IMU:\n");
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
    printf("  BHI360 IMU (IIO sysfs):\n");
    printf("    -I <name>    IIO device name (e.g. bhi360)\n");
    printf("    -M <ms>      poll period ms (default: 2 = 500Hz)\n");
    printf("    -x <map>     mount map (default: X,1,Y,1,Z,1 = identity)\n");
    printf("\n");
    printf("  Barometer (BMP581):\n");
    printf("    -B <name>    barometer input name (e.g. bmp5xy)\n");
    printf("    -O <odr>     BMP581 ODR enum (default: 8=100Hz)\n");
    printf("    -P <osr_p>   BMP581 pressure OSR (default: 2=4x)\n");
    printf("    -F <iir>     BMP581 IIR filter (default: 3=coeff7)\n");
    printf("\n");
    printf("    -h           help\n\n");
    printf("Examples:\n");
    printf("  ICM (I2C):    %s -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5\n", prog);
    printf("  ICM (SPI):    %s -b spi -s /dev/spidev0.0 -g gpiochip4 -l 2 -m 5\n", prog);
    printf("  BHI360:        %s -I bhi360\n", prog);
    printf("  BHI360 + Baro: %s -I bhi360 -B bmp5xy -O 8 -P 2 -F 3\n", prog);
    printf("  Baro only:    %s -B bmp5xy -O 8 -P 2 -F 3\n", prog);
}

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

/* ── 通用 sysfs 辅助 ── */
static void sysfs_write(const char *dir, const char *node, const char *value)
{
    char path[300];
    snprintf(path, sizeof(path), "%s/%s", dir, node);
    FILE *f = fopen(path, "w");
    if (f) { fputs(value, f); fclose(f); }
}

static float sysfs_read_float(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0.0f;
    float v = 0.0f;
    (void)fscanf(f, "%f", &v);
    fclose(f);
    return v;
}

static int find_input_by_name(const char *base_dir, const char *name, char *out_dir, int out_len)
{
    DIR *d = opendir(base_dir);
    if (!d) return -1;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') continue;
        char path[300];
        snprintf(path, sizeof(path), "%s/%s/name", base_dir, e->d_name);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char buf[64] = {0};
        (void)fread(buf, 1, sizeof(buf)-1, f);
        fclose(f);
        int n = strlen(buf);
        while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == ' ')) buf[--n] = 0;
        if (strcmp(buf, name) == 0) {
            snprintf(out_dir, out_len, "%s/%s", base_dir, e->d_name);
            closedir(d);
            return 0;
        }
    }
    closedir(d);
    return -1;
}

/* ── 气压计 ── */
static void baro_init(const char *name)
{
    if (find_input_by_name("/sys/class/input", name, g_baro_sysfs, sizeof(g_baro_sysfs)) != 0) {
        fprintf(stderr, "Barometer '%s' not found\n", name);
        return;
    }
    sysfs_write(g_baro_sysfs, "sensor_init", "1");
    usleep(50000);
    char osr_buf[32];
    snprintf(osr_buf, sizeof(osr_buf), "%d %d 1 %d", BARO_OSR_T_FIXED, g_baro_osr_p, g_baro_odr);
    sysfs_write(g_baro_sysfs, "osr_odr_press_config", osr_buf);
    if (g_baro_iir > 0) {
        char iir_buf[32];
        snprintf(iir_buf, sizeof(iir_buf), "1 1 0 %d %d", g_baro_iir, g_baro_iir);
        sysfs_write(g_baro_sysfs, "iir_config", iir_buf);
        usleep(20000);
    }
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
            if (g_baro_changes == 0) clock_gettime(CLOCK_MONOTONIC, &g_baro_t1);
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
    printf("Config ODR: %d, Measured: %.1f Hz (changes=%d)\n", g_baro_odr, odr, g_baro_changes);
}

/* ── BHI360 IIO ── */
static void bhi360_init(const char *name)
{
    if (find_input_by_name("/sys/bus/iio/devices", name, g_bhi360_iio_path, sizeof(g_bhi360_iio_path)) != 0) {
        fprintf(stderr, "BHI360 IIO device '%s' not found\n", name);
        return;
    }
    printf("BHI360 found at %s\n", g_bhi360_iio_path);

    /* 1. 上传固件 */
    sysfs_write(g_bhi360_iio_path, "firmware_upload", "1");
    printf("Uploading firmware (wait 3s)...\n");
    usleep(3000000);

    /* 2. 使能传感器 (加速度+陀螺仪+磁力计+四元数) */
    sysfs_write(g_bhi360_iio_path, "config_function", "2 1");   /* 加速度校准 */
    usleep(100000);
    sysfs_write(g_bhi360_iio_path, "config_function", "6 1");   /* 陀螺仪校准 */
    usleep(100000);
    sysfs_write(g_bhi360_iio_path, "config_function", "10 1");  /* 磁力计校准 */
    usleep(100000);
    sysfs_write(g_bhi360_iio_path, "config_function", "16 1");  /* 9轴四元数 */
    usleep(100000);

    g_bhi360_enabled = 1;
    printf("BHI360 init done: accel+gyro+quat enabled, poll=%dms\n", g_bhi360_sample_ms);
}

/* 读自定义 sysfs 属性 (acc_corrected / gyro_corrected / rotationVector)
 * 这些属性直接读 FIFO 回调缓存, 不依赖 IIO buffer, 有数据
 * IIO 标准 channel (in_accel_*_raw) 需要启 buffer 才更新, 这里不用 */
static int bhi360_read_attr(const char *base, const char *attr, float *v1, float *v2, float *v3)
{
    char path[300];
    snprintf(path, sizeof(path), "%s/%s", base, attr);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char buf[256] = {0};
    (void)fread(buf, 1, sizeof(buf)-1, f);
    fclose(f);
    /* 格式: "Accel Corrected Value:, X: -182, Y: -125, Z: -4089" */
    char *p = buf;
    /* 跳过到第一个数字 */
    while (*p && (*p < '0' || *p > '9') && *p != '-') p++;
    *v1 = atof(p);
    while (*p && (*p == '-' || *p == '.' || (*p >= '0' && *p <= '9'))) p++;
    while (*p && (*p < '0' || *p > '9') && *p != '-') p++;
    *v2 = atof(p);
    while (*p && (*p == '-' || *p == '.' || (*p >= '0' && *p <= '9'))) p++;
    while (*p && (*p < '0' || *p > '9') && *p != '-') p++;
    *v3 = atof(p);
    return 0;
}

static void bhi360_read(void)
{
    if (!g_bhi360_enabled) return;
    char base[256];
    strncpy(base, g_bhi360_iio_path, sizeof(base)-1);
    base[sizeof(base)-1] = 0;

    /* 加速度 (自定义 sysfs: acc_corrected, 单位 LSB, ±8g 量程 1g=4096) */
    float ax, ay, az;
    bhi360_read_attr(base, "acc_corrected", &ax, &ay, &az);
    ax /= 4096.0f; ay /= 4096.0f; az /= 4096.0f;  /* LSB -> g */

    /* 陀螺仪 (自定义 sysfs: gyro_corrected) */
    float gx, gy, gz;
    bhi360_read_attr(base, "gyro_corrected", &gx, &gy, &gz);

    /* 磁力计 (自定义 sysfs: mag_corrected) */
    float mx, my, mz;
    bhi360_read_attr(base, "mag_corrected", &mx, &my, &mz);

    /* 四元数 (自定义 sysfs: rotationVector)
     * 格式: "RotationVector_Value: X: -14907, Y: 6787, Z: 6, w: 396, accuracy: 51471"
     * Q14 定点数, /16384 转浮点, accuracy 0-3 表示校准精度 */
    float qw = 0, qx = 0, qy = 0, qz = 0;
    int rv_accuracy = 0;
    {
        char path[300];
        snprintf(path, sizeof(path), "%s/rotationVector", base);
        FILE *f = fopen(path, "r");
        if (f) {
            char buf[256] = {0};
            (void)fread(buf, 1, sizeof(buf)-1, f);
            fclose(f);
            char *p;
            p = strstr(buf, "X:");
            if (p) qx = atof(p + 2) / 16384.0f;
            p = strstr(buf, "Y:");
            if (p) qy = atof(p + 2) / 16384.0f;
            p = strstr(buf, "Z:");
            if (p) qz = atof(p + 2) / 16384.0f;
            p = strstr(buf, "w:");
            if (p) qw = atof(p + 2) / 16384.0f;
            p = strstr(buf, "accuracy:");
            if (p) rv_accuracy = atoi(p + 9);
        }
    }

    /* accuracy 含义: 0=未校准 1=低精度 2=中精度 3=高精度(校准完成) */
    int calib_acc = (rv_accuracy == 3) ? 3 : (rv_accuracy >= 1 ? 1 : 0);
    int calib_gyro = calib_acc;  /* 陀螺仪精度跟随9轴融合精度 */
    int calib_mag = calib_acc;

    /* 坐标变换: 芯片坐标 -> 机器人坐标 (和 ICM 一致) */
    imu_remap_vec(g_bhi360_mount_axis, g_bhi360_mount_sign, &ax, &ay, &az);
    imu_remap_vec(g_bhi360_mount_axis, g_bhi360_mount_sign, &gx, &gy, &gz);
    imu_remap_vec(g_bhi360_mount_axis, g_bhi360_mount_sign, &mx, &my, &mz);
    imu_remap_quat(g_bhi360_mount_axis, g_bhi360_mount_sign, &qw, &qx, &qy, &qz);

    /* 从四元数计算欧拉角 (ZYX: Yaw-Pitch-Roll, rad -> deg) */
    float yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * 57.29578f;
    float sp    = 2.0f*(qw*qy - qz*qx);
    if (sp > 1.0f) sp = 1.0f; else if (sp < -1.0f) sp = -1.0f;
    float pitch = asinf(sp) * 57.29578f;
    float roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 57.29578f;

    /* 统计频率 */
    static float prev_ax = 0;
    if (ax != prev_ax) {
        if (g_bhi360_changes == 0) clock_gettime(CLOCK_MONOTONIC, &g_bhi360_t1);
        clock_gettime(CLOCK_MONOTONIC, &g_bhi360_t2);
        g_bhi360_changes++;
    }
    prev_ax = ax;

    double actual_hz = 0;
    if (g_bhi360_changes >= 2) {
        double dur_us = (g_bhi360_t2.tv_sec - g_bhi360_t1.tv_sec) * 1000000.0
                      + (g_bhi360_t2.tv_nsec - g_bhi360_t1.tv_nsec) / 1000.0;
        actual_hz = g_bhi360_changes * 1000000.0 / dur_us;
    }

    struct timespec ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    unsigned long long ts_us = (unsigned long long)ts_now.tv_sec * 1000000ULL
                               + (unsigned long long)ts_now.tv_nsec / 1000ULL;

    /* 输出格式和 ICM 一致: IMU ts euler heading quat acc gyr mag sta ga ma */
    printf("IMU  ts=%llu euler=(yaw=%.1f pitch=%.1f roll=%.1f) heading=%.1f quat=(%.3f,%.3f,%.3f,%.3f) acc=(%.1f,%.1f,%.1f) gyr=(%.1f,%.1f,%.1f) mag=(%.1f,%.1f,%.1f) temp=0.0C sta=0 ga=%d ma=%d freq=%.1fHz\n",
           ts_us,
           yaw, pitch, roll,
           yaw,
           qw, qx, qy, qz,
           ax, ay, az,
           gx, gy, gz,
           mx, my, mz,
           calib_gyro, calib_mag,
           actual_hz);
}

static void bhi360_print_summary(void)
{
    if (!g_bhi360_enabled || g_bhi360_changes < 2) return;
    double dur_us = (g_bhi360_t2.tv_sec - g_bhi360_t1.tv_sec) * 1000000.0
                  + (g_bhi360_t2.tv_nsec - g_bhi360_t1.tv_nsec) / 1000.0;
    double hz = g_bhi360_changes * 1000000.0 / dur_us;
    printf("\n=== BHI360 Frequency Summary ===\n");
    printf("Poll period: %dms, Data changes: %d, Measured freq: %.1f Hz\n",
           g_bhi360_sample_ms, g_bhi360_changes, hz);
}

static void run_bhi360(void)
{
    printf("=== BHI360 IIO Mode ===\n");
    printf("Poll period: %dms\n\n", g_bhi360_sample_ms);
    printf("Reading BHI360 data (Ctrl+C to stop)...\n\n");

    while (g_running) {
        bhi360_read();
        baro_read();
        usleep(g_bhi360_sample_ms * 1000);
    }

    bhi360_print_summary();
    baro_print_summary();
}

/* ── ICM45608 ── */
static void run_icm(const char *bus_type, const char *i2c_dev, const char *spi_dev,
                    uint32_t spi_speed, uint8_t spi_mode,
                    const char *gpio_chip, unsigned int gpio_line, int op_mode,
                    const int8_t mount_axis[3], const int8_t mount_sign[3])
{
    int use_spi = (strcmp(bus_type, "spi") == 0);
    printf("=== ICM45608 IMU ===\n");
    if (use_spi)
        printf("Bus: SPI %s (mode=%u speed=%uHz), GPIO: %s line %u, Mode: %d\n",
               spi_dev, spi_mode, spi_speed, gpio_chip, gpio_line, op_mode);
    else
        printf("Bus: I2C %s, GPIO: %s line %u, Mode: %d\n",
               i2c_dev, gpio_chip, gpio_line, op_mode);

    emd_gaf_t *gaf = emd_gaf_create();
    if (!gaf) { fprintf(stderr, "Failed to create IMU HAL\n"); return; }

    emd_gaf_cfg_t cfg = {0};
    cfg.gpio_chip = gpio_chip;
    cfg.gpio_line = gpio_line;
    cfg.op_mode   = op_mode;
    if (use_spi) {
        cfg.if_type = EMD_GAF_IF_SPI;
        cfg.spi_dev = spi_dev;
        cfg.spi_speed_hz = spi_speed;
        cfg.spi_mode = spi_mode;
    } else {
        cfg.if_type = EMD_GAF_IF_I2C;
        cfg.i2c_dev = i2c_dev;
    }
    cfg.mount_axis[0] = mount_axis[0]; cfg.mount_axis[1] = mount_axis[1]; cfg.mount_axis[2] = mount_axis[2];
    cfg.mount_sign[0] = mount_sign[0]; cfg.mount_sign[1] = mount_sign[1]; cfg.mount_sign[2] = mount_sign[2];

    int rc = emd_gaf_init(gaf, &cfg);
    if (rc != 0) { fprintf(stderr, "IMU init failed: rc=%d\n", rc); emd_gaf_destroy(gaf); return; }
    printf("IMU HAL OK\n");

    emd_gaf_set_raw_data_callback(gaf, raw_data_cb, NULL);
    rc = emd_gaf_start(gaf);
    if (rc != 0) { fprintf(stderr, "IMU start failed: rc=%d\n", rc); emd_gaf_destroy(gaf); return; }
    printf("Background thread started\nWaiting 2s...\n");
    usleep(2000000);
    printf("\nReading (Ctrl+C to stop)...\n\n");

    while (g_running) {
        emd_output_t out;
        emd_imu_data_t accel, gyro;
        if (emd_gaf_get_output(gaf, &out) == 0) {
            float qw = out.quat_w, qx = out.quat_x, qy = out.quat_y, qz = out.quat_z;
            float yaw = atan2f(2.0f*(qw*qz+qx*qy), 1.0f-2.0f*(qy*qy+qz*qz)) * 57.29578f;
            float sp = 2.0f*(qw*qy-qz*qx); if (sp>1)sp=1; if(sp<-1)sp=-1;
            float pitch = asinf(sp) * 57.29578f;
            float roll = atan2f(2.0f*(qw*qx+qy*qz), 1.0f-2.0f*(qx*qx+qy*qy)) * 57.29578f;
            printf("IMU  ts=%llu euler=(yaw=%.1f pitch=%.1f roll=%.1f) heading=%.1f quat=(%.3f,%.3f,%.3f,%.3f) acc=(%.3f,%.3f,%.3f) gyr=(%.1f,%.1f,%.1f) mag=(%.1f,%.1f,%.1f) temp=%.1fC sta=%d ga=%d ma=%d\n",
                   (unsigned long long)out.timestamp_us,
                   yaw, pitch, roll,
                   out.heading_deg,
                   qw, qx, qy, qz,
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
    printf("\nStopping...\n");
    emd_gaf_stop(gaf);
    emd_gaf_destroy(gaf);
}

static void run_baro_only(void)
{
    printf("=== Barometer Only ===\n");
    printf("Reading (Ctrl+C to stop)...\n\n");
    while (g_running) { baro_read(); usleep(2000); }
    baro_print_summary();
}

int main(int argc, char *argv[])
{
    /* ICM 参数 */
    const char *bus_type = "i2c", *i2c_dev = "/dev/i2c-3", *spi_dev = "/dev/spidev0.0";
    uint32_t spi_speed = 8000000; uint8_t spi_mode = 0;
    const char *gpio_chip = "gpiochip4"; unsigned int gpio_line = 2;
    int op_mode = 5;
    int8_t mount_axis[3] = {2,0,1}, mount_sign[3] = {-1,-1,1};

    /* BHI360 参数 */
    const char *bhi360_name = NULL;

    /* 气压计参数 */
    const char *baro_name = NULL;

    int opt;
    while ((opt = getopt(argc, argv, "b:i:s:p:o:g:l:m:x:I:M:B:O:P:F:h")) != -1) {
        switch (opt) {
        case 'b': bus_type = optarg; break;
        case 'i': i2c_dev = optarg; break;
        case 's': spi_dev = optarg; break;
        case 'p': spi_speed = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 'o': spi_mode = (uint8_t)atoi(optarg); break;
        case 'g': gpio_chip = optarg; break;
        case 'l': gpio_line = (unsigned int)atoi(optarg); break;
        case 'm': op_mode = atoi(optarg); break;
        case 'x':
            if (parse_mount(optarg, mount_axis, mount_sign) != 0 ||
                parse_mount(optarg, g_bhi360_mount_axis, g_bhi360_mount_sign) != 0) {
                fprintf(stderr, "Invalid mount spec '%s'\n", optarg);
                usage(argv[0]); return 1;
            }
            break;
        case 'I': bhi360_name = optarg; break;
        case 'M': g_bhi360_sample_ms = atoi(optarg); break;
        case 'B': baro_name = optarg; break;
        case 'O': g_baro_odr = atoi(optarg); break;
        case 'P': g_baro_osr_p = atoi(optarg); break;
        case 'F': g_baro_iir = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default: usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* 初始化气压计 */
    if (baro_name) baro_init(baro_name);

    /* BHI360 模式 */
    if (bhi360_name) {
        bhi360_init(bhi360_name);
        if (!g_bhi360_enabled) { fprintf(stderr, "BHI360 init failed\n"); return 1; }
        run_bhi360();
        return 0;
    }

    /* 只测气压计 */
    if (baro_name && !bhi360_name) {
        int has_icm = 0;
        for (int i = 1; i < argc; i++)
            if (strcmp(argv[i],"-i")==0||strcmp(argv[i],"-g")==0||strcmp(argv[i],"-l")==0||
                strcmp(argv[i],"-m")==0||strcmp(argv[i],"-x")==0||strcmp(argv[i],"-b")==0||
                strcmp(argv[i],"-s")==0) { has_icm = 1; break; }
        if (!has_icm) { run_baro_only(); return 0; }
    }

    /* ICM 模式 */
    if (op_mode < 0 || op_mode > 9) { fprintf(stderr, "Invalid opmode\n"); return 1; }
    if (spi_mode > 3) { fprintf(stderr, "Invalid SPI mode\n"); return 1; }
    if (strcmp(bus_type,"spi")!=0 && strcmp(bus_type,"i2c")!=0) { fprintf(stderr,"Invalid bus\n"); return 1; }

    run_icm(bus_type, i2c_dev, spi_dev, spi_speed, spi_mode, gpio_chip, gpio_line, op_mode, mount_axis, mount_sign);
    baro_print_summary();
    printf("Done.\n");
    return 0;
}
