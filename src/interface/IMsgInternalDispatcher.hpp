/*
 * @file IMsgInternalDispatcher.hpp
 * @brief 被观察者基类 — 电机数据调度中心
 * Copyright (c) 2026 zhiqiang.yang
 *
 * CanDispatcher 实现此接口，负责:
 *   管理 IListener 注册/注销
 *   接收上层控制命令 (Send)
 *   向所有 Listener 广播反馈数据 (NotifyObserver)
 */
#pragma once

#include <string>
#include <boost/any.hpp>
#include "interface/Defines.hpp"
#include "interface/IListener.hpp"

namespace stark_periph_manager_node
{

class IMsgInternalDispatcher
{
public:
    IMsgInternalDispatcher() = default;
    virtual ~IMsgInternalDispatcher() = default;

    virtual bool
    InitDispatcher() = 0;

    virtual bool
    DestroyDispatcher() = 0;

    virtual void
    Send(const std::string& data) = 0;

    virtual void
    RegisterObserver(ListenerType type, std::shared_ptr<IListener> listener) = 0;

    virtual void
    RemoveObserver(ListenerType type, std::shared_ptr<IListener> listener) = 0;

    virtual void
    NotifyObserver(const boost::any& data) = 0;
};

}  /* namespace stark_periph_manager_node */
