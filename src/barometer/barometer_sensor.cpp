/*
 * barometer_sensor.cpp — BMP581 气压计传感器实现 (L0)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 数据来源: 内核 input 驱动的 sysfs sensor_data (非 input 事件、非 IIO)。
 * 后台线程轮询 sysfs, 解析气压(Pa)/温度(°C), 换算 hPa + 计算海拔, 缓存。
 */

#include "barometer/barometer_sensor.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <ctime>
#include <dirent.h>
#include <unistd.h>

namespace stark_periph_manager_node {

/* 按 input 设备名 (如 "bmp5xy") 定位 sysfs 目录 /sys/class/input/inputN */
static bool _find_input_by_name(const std::string& name, std::string& out_dir)
{
    DIR* d = opendir("/sys/class/input");
    if (!d) return false;

    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        std::string base = e->d_name;
        if (base.rfind("input", 0) != 0) continue;  /* 只要 inputN */

        std::string name_path = "/sys/class/input/" + base + "/name";
        FILE* f = fopen(name_path.c_str(), "r");
        if (!f) continue;

        char buf[64] = {0};
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);

        /* 去掉尾部换行/空白 */
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == ' ' ||
                         buf[n - 1] == '\t' || buf[n - 1] == '\r'))
            buf[--n] = '\0';

        if (name == buf) {
            out_dir = "/sys/class/input/" + base;
            closedir(d);
            return true;
        }
    }

    closedir(d);
    return false;
}

BarometerSensor::BarometerSensor()
{
}

BarometerSensor::~BarometerSensor()
{
    Deinit();
}

bool BarometerSensor::_WriteSysfs(const std::string& path, const std::string& value)
{
    FILE* f = fopen(path.c_str(), "w");
    if (!f) return false;
    size_t n = fwrite(value.c_str(), 1, value.size(), f);
    fclose(f);
    return n == value.size();
}

bool BarometerSensor::Init(const BarometerConfig& cfg)
{
    m_cfg = cfg;

    /* 定位 sysfs 目录 (设备未接入时返回 false, 上层跳过) */
    if (!_find_input_by_name(cfg.input_name, m_sysfs_dir)) {
        fprintf(stderr, "[baro] input device '%s' not found\n",
                cfg.input_name.c_str());
        return false;
    }

    /* 初始化序列: sensor_init → osr_odr_press_config → power_mode
     * (对齐驱动 README 实测, 开机必须执行) */
    char osr_buf[64];
    snprintf(osr_buf, sizeof(osr_buf), "%d %d 1 %d",
             cfg.osr_t, cfg.osr_p, cfg.odr);

    _WriteSysfs(m_sysfs_dir + "/sensor_init", "1");
    usleep(50000);
    _WriteSysfs(m_sysfs_dir + "/osr_odr_press_config", osr_buf);
    _WriteSysfs(m_sysfs_dir + "/power_mode", std::to_string(cfg.power_mode));
    usleep(50000);

    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&BarometerSensor::_ReaderThread, this);
    return true;
}

void BarometerSensor::Deinit()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_ready.store(false, std::memory_order_release);
}

void BarometerSensor::_ReaderThread()
{
    std::string path = m_sysfs_dir + "/sensor_data";

    while (m_running.load(std::memory_order_acquire)) {
        FILE* f = fopen(path.c_str(), "r");
        if (f) {
            char buf[128] = {0};
            (void)fread(buf, 1, sizeof(buf) - 1, f);
            fclose(f);

            unsigned long long press_pa = 0;
            long temp_c = 0;
            if (sscanf(buf, "Pressure: %llu Pa Temperature: %ld deg C",
                       &press_pa, &temp_c) == 2) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t ts_us = (uint64_t)ts.tv_sec * 1000000UL +
                                 (uint64_t)ts.tv_nsec / 1000UL;

                float pressure_hpa = (float)press_pa / 100.0f;
                float temperature_c = (float)temp_c;
                /* 标准大气压测高公式 */
                float altitude_m = 44330.0f * (1.0f -
                    powf(pressure_hpa / m_cfg.sea_level_hpa, 1.0f / 5.255f));

                {
                    std::lock_guard<std::mutex> lk(m_lock);
                    m_data.pressure_hpa  = pressure_hpa;
                    m_data.temperature_c = temperature_c;
                    m_data.altitude_m    = altitude_m;
                    m_data.timestamp_us  = ts_us;
                }
                m_ready.store(true, std::memory_order_release);
            }
        }

        usleep(m_cfg.sample_period_ms * 1000);
    }
}

void BarometerSensor::Read(barometer_data_t* out) const
{
    if (!out) return;
    std::lock_guard<std::mutex> lk(m_lock);
    *out = m_data;
}

}  /* namespace stark_periph_manager_node */
