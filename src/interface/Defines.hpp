/*
 * @file Defines.hpp
 * @brief 类型定义
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

namespace stark_periph_manager_node
{

enum class ListenerType
{
    ROS = 0,
    WEB = 1,
    MAX_SIZE
};

enum class MsgType
{
    IMU     = 0,
    MOTOR   = 1,
    SENSOR  = 2,
    STATE   = 3,
    MAX_SIZE
};

}  /* namespace stark_periph_manager_node */
