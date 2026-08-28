/*
 * barometer_source.h — 气压计数据源抽象接口
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 统一不同气压计驱动的数据输出为 barometer_data_t:
 *   - Bosch BMP581 (内核 input 驱动, 读 sysfs sensor_data, 本实现)
 *   - 后续换芯片/换接口 = 换 L0 实现, 上层零改动
 */
#pragma once

#include <string>
#include <cstdint>

#include "stark_shm.h"   /* barometer_data_t */

namespace stark_periph_manager_node {

/* 气压计配置 (来自 config.json sensor.barometer 段) */
struct BarometerConfig {
    std::string input_name       = "bmp5xy";    /* sysfs input 设备名 (定位目录用) */
    uint32_t    sample_period_ms = 50;          /* 采样/上报周期 ms (默认 20Hz) */
    int         osr_t            = 6;           /* 温度过采样 */
    int         osr_p            = 2;           /* 气压过采样 */
    int         odr              = 15;          /* ODR 枚举 (0=240Hz ... 31=0.125Hz) */
    int         power_mode       = 1;           /* 0=standby 1=normal 2=forced 3=continuous */
    float       sea_level_hpa    = 1013.25f;    /* 海平面气压基准 (算海拔用) */
};

/* 气压计数据源接口 */
class IBarometerSource {
public:
    virtual ~IBarometerSource() = default;

    virtual bool Init(const BarometerConfig& cfg) = 0;
    virtual void Deinit() = 0;
    virtual void Read(barometer_data_t* out) const = 0;
    virtual bool IsReady() const = 0;
};

}  /* namespace stark_periph_manager_node */
