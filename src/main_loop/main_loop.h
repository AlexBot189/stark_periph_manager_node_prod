/*
 * main_loop.h — 主循环逻辑
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 从 src/main.cpp 提取主循环和所有轮询函数.
 * 全局变量 (g_running, g_log_running, g_dispatcher, g_rt_worker, g_rt_log, g_ctx)
 * 留在 src/main.cpp 中, 通过 extern 引用.
 */
#pragma once

extern "C" {
#include "motor_hal.h"
#include "stark_shm.h"
}

namespace stark_periph_manager_node {
class CanDispatcher;
class StarkRtWorker;
}

void main_loop_run(motor_hal_t* hal, stark_shm_t* shm,
                   int motor_count, stark_periph_manager_node::CanDispatcher* dispatcher,
                   stark_periph_manager_node::StarkRtWorker* rt_worker, bool enable_rt);
