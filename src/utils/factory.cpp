/*
 * Factory.cpp — 对象工厂实现
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "utils/factory.h"
#include "motor/motor_init.h"
#include "stark_shm.h"

#ifdef ENABLE_ROS
#include "ros/RosAdapter.h"
#endif

#include <cstdio>
#include <log_helper/LogHelper.h>

namespace stark_periph_manager_node {

std::shared_ptr<IMsgInternalDispatcher>
Factory::CreateSingletonDispatcher()
{
    auto dispatcher = std::make_shared<CanDispatcher>();
    if (!dispatcher->InitDispatcher()) {
        ECO_ERROR_NEW("[Factory] CanDispatcher::InitDispatcher() failed");
        return nullptr;
    }
    ECO_INFO_NEW("[Factory] CanDispatcher created");
    return dispatcher;
}

#ifdef ENABLE_ROS
std::shared_ptr<IListener>
Factory::CreateRosListener(std::shared_ptr<ros::NodeHandle> nh,
                           std::shared_ptr<IMsgInternalDispatcher> dispatcher)
{
    if (!nh || !dispatcher) {
        ECO_ERROR_NEW("[Factory] CreateRosListener: invalid args");
        return nullptr;
    }

    auto can_disp = std::dynamic_pointer_cast<CanDispatcher>(dispatcher);
    if (!can_disp) {
        ECO_ERROR_NEW("[Factory] dispatcher is not CanDispatcher");
        return nullptr;
    }

    stark_shm_t* shm = can_disp->GetShm();
    if (!shm) {
        ECO_ERROR_NEW("[Factory] SHM not available");
        return nullptr;
    }

    auto adapter = std::make_shared<RosAdapter>(nh, shm);
    adapter->SetDispatcher(dispatcher);
    adapter->Start();

    ECO_INFO_NEW("[Factory] RosAdapter created (pull from SHM)");
    return adapter;
}
#endif

}  /* namespace stark_periph_manager_node */
