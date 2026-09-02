/*
 * imu_source.h — IMU 数据源抽象接口
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 统一不同 IMU 驱动的数据输出为 imu_data_t:
 *   - InvenSense ICM45608 (libimu_hal, 本实现)
 *   - Bosch (Linux IIO, 后续扩展)
 */
#pragma once

#include <string>
#include <cstdint>

#include "stark_shm.h"   /* imu_data_t */

namespace stark_periph_manager_node {

/* IMU 配置 (来自 config.json imu 段) */
struct ImuConfig {
    std::string driver      = "invensense";  /* invensense | bosch */
    std::string interface   = "i2c";          /* i2c | spi */
    std::string i2c_dev     = "/dev/i2c-3";
    std::string spi_dev     = "/dev/spidev0.0";
    uint32_t    spi_speed_hz = 8000000;
    uint8_t     spi_mode     = 0;
    std::string gpio_chip   = "gpiochip4";
    uint32_t    gpio_line   = 6;
    int         op_mode     = 5;
    /* BHI360 上报频率: 轮询周期 ms (仅 BHI360 使用) */
    uint32_t    sample_period_ms = 2;  /* 默认 2ms = 500Hz */
    /* 坐标轴映射: robot[x,y,z] 取 chip 的轴(0=X,1=Y,2=Z) + 符号 */
    int8_t      mount_axis[3] = {2, 0, 1};
    int8_t      mount_sign[3] = {-1, -1, 1};
};

/* IMU 数据源接口 */
class IImuSource {
public:
    virtual ~IImuSource() = default;

    virtual bool Init(const ImuConfig& cfg) = 0;
    virtual void Deinit() = 0;
    virtual void Read(imu_data_t* out) const = 0;
    virtual bool IsReady() const = 0;
};

}  /* namespace stark_periph_manager_node */
