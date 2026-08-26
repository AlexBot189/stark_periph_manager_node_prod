/*
 * stark_node_context.h — 全局上下文 (供状态机 enter/exit 钩子访问)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * enter/exit 钩子为 void(void) 无参数函数,
 * 通过此全局指针访问 HAL / SHM / 配置.
 */
#pragma once

#include <string>

extern "C" {
#include "motor_hal.h"
#include "stark_shm.h"
}

namespace stark_periph_manager_node {

struct StarkNodeContext {
    motor_hal_t* hal          = nullptr;
    stark_shm_t*   shm          = nullptr;
    int          motor_count  = 2;

    /* 传感器透传 */
    uint16_t     sensor_period_ms = 1;        /* ms (旧字段, 兼容显示用) */
    uint16_t     sensor_period_div = 1;       /* 0.5ms 基准分频, 默认 1=2000Hz */
    uint8_t      sensor_bus_format = 3;       /* 3=CANFD BRS, 0=Classic CAN */
    uint8_t      sensor_mode = 2;             /* 0=关 1=仅传感器帧 2=全部帧 */
    uint8_t      sensor_force_module = 1;     /* 0=CAN力矩 1=SPI力矩 */

    /* 周期上报 */
    bool         report_auto_enable = true;   /* 校准后自动开启上报 */
    uint32_t     report_period_ms   = 5;      /* 上报周期 ms */
    std::string  report_data_source = "mixed"; /* 数据来源: mixed | unified_6c0 */

    /* LED 灯显 */
    int          led_motor_id = 0;            /* 0=禁用, 1=右电机, 2=左电机 */

    /* 按键 */
    std::string  btn_calib_chip;
    int          btn_calib_line  = -1;
    int          btn_calib_long_press_ms = 5000;
    std::string  btn_report_chip;
    int          btn_report_line = -1;
};

}  /* namespace stark_periph_manager_node */

extern stark_periph_manager_node::StarkNodeContext* g_ctx;
