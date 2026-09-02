/*
 * bhi360_imu_sensor.cpp — BHI360 IIO 传感器实现
 *
 * 通过 IIO sysfs 读取 BHI360 数据:
 *   1. firmware_upload → 固件上传
 *   2. config_function → 使能加速度/陀螺仪/磁力计/欧拉角/四元数
 *   3. 后台线程轮询 sysfs 读取数据
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "sensor/imu/bhi360_imu_sensor.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <ctime>
#include <dirent.h>
#include <unistd.h>

namespace stark_periph_manager_node {

Bhi360IioSensor::Bhi360IioSensor() {}

Bhi360IioSensor::~Bhi360IioSensor() { Deinit(); }

/* 遍历 /sys/bus/iio/devices/ 找 name="bhi360" 的设备 */
bool Bhi360IioSensor::_FindIioDevice(const std::string& name, std::string& out_path)
{
    DIR* d = opendir("/sys/bus/iio/devices");
    if (!d) return false;

    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        std::string base = e->d_name;
        if (base[0] == '.') continue;

        std::string name_path = "/sys/bus/iio/devices/" + base + "/name";
        FILE* f = fopen(name_path.c_str(), "r");
        if (!f) continue;

        char buf[64] = {0};
        (void)fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);

        int n = strlen(buf);
        while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == ' ')) buf[--n] = 0;

        if (name == buf) {
            out_path = "/sys/bus/iio/devices/" + base;
            closedir(d);
            return true;
        }
    }
    closedir(d);
    return false;
}

bool Bhi360IioSensor::_SysfsWrite(const std::string& path, const std::string& value)
{
    FILE* f = fopen(path.c_str(), "w");
    if (!f) return false;
    size_t n = fwrite(value.c_str(), 1, value.size(), f);
    fclose(f);
    return n == value.size();
}

float Bhi360IioSensor::_SysfsReadFloat(const std::string& path)
{
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return 0.0f;
    float v = 0.0f;
    (void)fscanf(f, "%f", &v);
    fclose(f);
    return v;
}

int Bhi360IioSensor::_SysfsReadInt(const std::string& path)
{
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return 0;
    int v = 0;
    (void)fscanf(f, "%d", &v);
    fclose(f);
    return v;
}

bool Bhi360IioSensor::Init(const ImuConfig& cfg)
{
    /* 保存配置 */
    m_sample_period_ms = cfg.sample_period_ms;
    memcpy(m_mount_axis, cfg.mount_axis, sizeof(m_mount_axis));
    memcpy(m_mount_sign, cfg.mount_sign, sizeof(m_mount_sign));

    /* 定位 IIO 设备 */
    if (!_FindIioDevice("bhi360", m_iio_path)) {
        fprintf(stderr, "[Bhi360IioSensor] IIO device 'bhi360' not found\n");
        return false;
    }

    fprintf(stderr, "[Bhi360IioSensor] found at %s\n", m_iio_path.c_str());

    /* 1. 上传固件 */
    _SysfsWrite(m_iio_path + "/firmware_upload", "1");
    usleep(2000000);  /* 固件上传需要约 2 秒 */

    /* 2. 使能传感器 (config_function: feature_id enable) */
    _SysfsWrite(m_iio_path + "/config_function", "2 1");   /* 加速度校准 */
    usleep(100000);
    _SysfsWrite(m_iio_path + "/config_function", "6 1");   /* 陀螺仪校准 */
    usleep(100000);
    _SysfsWrite(m_iio_path + "/config_function", "10 1");  /* 磁力计校准 */
    usleep(100000);
    _SysfsWrite(m_iio_path + "/config_function", "16 1");  /* 9轴四元数 */
    usleep(100000);

    /* 3. 设置采样频率 (如果可配) */
    _SysfsWrite(m_iio_path + "/in_sampling_frequency", "100");

    fprintf(stderr, "[Bhi360IioSensor] firmware uploaded, sensors enabled\n");

    /* 4. 启动后台读取线程 */
    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&Bhi360IioSensor::_ReaderThread, this);
    return true;
}

void Bhi360IioSensor::Deinit()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_ready.store(false, std::memory_order_release);
}

void Bhi360IioSensor::_ReaderThread()
{
    std::string p = m_iio_path;

    while (m_running.load(std::memory_order_acquire)) {
        imu_data_t d = {};

        /* 加速度 (自定义 sysfs: acc_corrected, ±8g 量程 1g=4096 LSB) */
        {
            FILE* f = fopen((p + "/acc_corrected").c_str(), "r");
            if (f) {
                char buf[256] = {0};
                (void)fread(buf, 1, sizeof(buf)-1, f);
                fclose(f);
                const char* s = buf;
                auto skip_to_num = [&s]() {
                    while (*s && (*s < '0' || *s > '9') && *s != '-') s++;
                };
                auto read_num = [&s]() -> float {
                    float v = strtof(s, (char**)&s);
                    return v;
                };
                skip_to_num(); d.acc_x = read_num() / 4096.0f;
                while (*s && (*s == '-' || *s == '.' || (*s >= '0' && *s <= '9'))) s++;
                skip_to_num(); d.acc_y = read_num() / 4096.0f;
                while (*s && (*s == '-' || *s == '.' || (*s >= '0' && *s <= '9'))) s++;
                skip_to_num(); d.acc_z = read_num() / 4096.0f;
            }
        }

        /* 陀螺仪 (自定义 sysfs: gyro_corrected) */
        {
            FILE* f = fopen((p + "/gyro_corrected").c_str(), "r");
            if (f) {
                char buf[256] = {0};
                (void)fread(buf, 1, sizeof(buf)-1, f);
                fclose(f);
                const char* s = buf;
                auto skip_to_num = [&s]() {
                    while (*s && (*s < '0' || *s > '9') && *s != '-') s++;
                };
                auto read_num = [&s]() -> float {
                    float v = strtof(s, (char**)&s);
                    return v;
                };
                skip_to_num(); d.gyro_x = read_num() / 16.384f;
                while (*s && (*s == '-' || *s == '.' || (*s >= '0' && *s <= '9'))) s++;
                skip_to_num(); d.gyro_y = read_num() / 16.384f;
                while (*s && (*s == '-' || *s == '.' || (*s >= '0' && *s <= '9'))) s++;
                skip_to_num(); d.gyro_z = read_num() / 16.384f;
            }
        }

        /* 四元数 (自定义 sysfs: rotationVector)
         * 格式: "RotationVector_Value: X: -14907, Y: 6787, Z: 6, w: 396, accuracy: 51471"
         * 值为 Q14 定点数, 需除以 16384 转浮点 */
        {
            FILE* f = fopen((p + "/rotationVector").c_str(), "r");
            if (f) {
                char buf[256] = {0};
                (void)fread(buf, 1, sizeof(buf)-1, f);
                fclose(f);
                const char* pX = strstr(buf, "X:");
                const char* pY = strstr(buf, "Y:");
                const char* pZ = strstr(buf, "Z:");
                const char* pW = strstr(buf, "w:");
                if (pX) d.quat_x = strtof(pX + 2, nullptr) / 16384.0f;
                if (pY) d.quat_y = strtof(pY + 2, nullptr) / 16384.0f;
                if (pZ) d.quat_z = strtof(pZ + 2, nullptr) / 16384.0f;
                if (pW) d.quat_w = strtof(pW + 2, nullptr) / 16384.0f;
            }
        }

        /* 从四元数计算欧拉角 (ZYX: Yaw-Pitch-Roll, rad -> deg) */
        /* 先做坐标变换: 芯片坐标 -> 机器人坐标 */
        imu_remap_vec(m_mount_axis, m_mount_sign, &d.acc_x, &d.acc_y, &d.acc_z);
        imu_remap_vec(m_mount_axis, m_mount_sign, &d.gyro_x, &d.gyro_y, &d.gyro_z);
        imu_remap_vec(m_mount_axis, m_mount_sign, &d.mag_x, &d.mag_y, &d.mag_z);
        imu_remap_quat(m_mount_axis, m_mount_sign, &d.quat_w, &d.quat_x, &d.quat_y, &d.quat_z);

        float qw = d.quat_w, qx = d.quat_x, qy = d.quat_y, qz = d.quat_z;
        d.yaw = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * 57.29578f;
        float sp = 2.0f*(qw*qy - qz*qx);
        if (sp > 1.0f) sp = 1.0f; else if (sp < -1.0f) sp = -1.0f;
        d.pitch = asinf(sp) * 57.29578f;
        d.roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 57.29578f;
        d.heading_deg = d.yaw;

        /* 磁力计暂不读 (9轴融合在芯片内部完成) */
        d.mag_x = 0; d.mag_y = 0; d.mag_z = 0;

        /* 时间戳 */
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        d.timestamp_us = (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;

        m_data.store(d);
        m_ready.store(true, std::memory_order_release);

        usleep(m_sample_period_ms * 1000);
    }
}

void Bhi360IioSensor::Read(imu_data_t* out) const
{
    if (!out) return;
    m_data.load(*out);
}

}  /* namespace stark_periph_manager_node */
