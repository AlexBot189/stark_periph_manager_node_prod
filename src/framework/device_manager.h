/*
 * device_manager.h — 设备管理器 (L2)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 中心注册表: 编译期静态注册驱动, 配置驱动实例化设备, 统一生命周期。
 */
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "framework/device.h"

namespace stark {

using DeviceCreator = Device* (*)();

/*
 * 设备管理器 (单例)
 */
class DeviceManager {
public:
    static DeviceManager& instance();

    /* 注册驱动 (由 REGISTER_DEVICE 宏调用) */
    void registerDriver(const std::string& driver, DeviceCreator creator);

    /* 解析 devices 配置数组, 创建并初始化所有设备 (fail-safe, 不崩溃) */
    bool loadDevices(const nlohmann::json& devices);

    /* 统一生命周期: 顺序启动, 逆序停止 */
    bool startAll();
    void stopAll();

    /* 按名查找 */
    IMotorDevice*  motor(const std::string& name);
    ISensorDevice* sensor(const std::string& name);
    const std::vector<std::unique_ptr<Device>>& all() const { return m_devices; }

private:
    DeviceManager() = default;

    std::unordered_map<std::string, DeviceCreator> m_drivers;
    std::vector<std::unique_ptr<Device>>           m_devices;
};

/*
 * 静态注册器: 构造时自动注册到 DeviceManager
 */
struct DeviceRegistrar {
    DeviceRegistrar(const std::string& driver, DeviceCreator creator);
};

}  // namespace stark

/*
 * 注册宏: 在驱动 .cpp 里调用一次, 编译期注册, 不用 dlopen
 *
 *   REGISTER_DEVICE(motor_canfd, MotorCanfd::create)
 */
#define REGISTER_DEVICE(driver, creator) \
    static const stark::DeviceRegistrar _registrar_##driver(#driver, creator)
