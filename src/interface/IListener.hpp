/*
 * @file IListener.hpp
 * @brief 观察者基类 — 接收 CanDispatcher 推送的数据
 * Copyright (c) 2026 zhiqiang.yang
 *
 * RosAdapter / WebServer 通过 Update() 接收 boost::any 数据包.
 */
#pragma once

#include <memory>
#include <boost/any.hpp>
#include "interface/Defines.hpp"

namespace stark_periph_manager_node
{

class IMsgInternalDispatcher;

class IListener
{
public:
    virtual ~IListener() = default;

    virtual void
    Update(const boost::any& data) = 0;
};

}  /* namespace stark_periph_manager_node */
