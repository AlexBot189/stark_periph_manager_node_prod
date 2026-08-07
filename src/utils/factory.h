/*
 * Factory.h — 对象工厂
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <memory>
#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"

#ifdef ENABLE_ROS
#include <ros/ros.h>
#endif

namespace stark_periph_manager_node {

class Factory {
public:
    static std::shared_ptr<IMsgInternalDispatcher>
    CreateSingletonDispatcher();

#ifdef ENABLE_ROS
    static std::shared_ptr<IListener>
    CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<IMsgInternalDispatcher> dispatcher);
#endif
};

}  /* namespace stark_periph_manager_node */
