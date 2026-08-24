/*
 * device_manager.cpp — 设备管理器实现 (L2)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/device_manager.h"

#include <cstdio>

namespace stark {

DeviceManager& DeviceManager::instance()
{
    static DeviceManager mgr;
    return mgr;
}

void DeviceManager::registerDriver(const std::string& driver, DeviceCreator creator)
{
    m_drivers[driver] = creator;
}

bool DeviceManager::loadDevices(const nlohmann::json& devices)
{
    if (!devices.is_array()) {
        return true;  /* 无设备配置, 允许空启动 */
    }

    for (const auto& entry : devices) {
        const std::string name   = entry.value("name", "");
        const std::string driver = entry.value("driver", "");
        if (name.empty() || driver.empty()) {
            fprintf(stderr, "[DeviceManager] device entry missing name/driver, skip\n");
            continue;
        }

        auto it = m_drivers.find(driver);
        if (it == m_drivers.end()) {
            fprintf(stderr, "[DeviceManager] unknown driver '%s' for '%s', skip\n",
                    driver.c_str(), name.c_str());
            continue;
        }

        Device* dev = it->second();
        if (!dev) {
            fprintf(stderr, "[DeviceManager] create '%s' (driver=%s) failed\n",
                    name.c_str(), driver.c_str());
            continue;
        }

        /* 注入实例名到 config, 供设备 initialize 读取 */
        nlohmann::json cfg = entry.contains("config") ? entry["config"]
                                                      : nlohmann::json::object();
        cfg["name"] = name;

        std::unique_ptr<Device> holder(dev);
        if (!holder->initialize(cfg)) {
            fprintf(stderr, "[DeviceManager] initialize '%s' failed, skip\n",
                    name.c_str());
            continue;  /* holder 析构释放 */
        }

        m_devices.push_back(std::move(holder));
    }
    return true;
}

bool DeviceManager::startAll()
{
    bool ok = true;
    for (auto& d : m_devices) {
        if (!d->start()) {
            fprintf(stderr, "[DeviceManager] start '%s' failed\n", d->name());
            ok = false;
        }
    }
    return ok;
}

void DeviceManager::stopAll()
{
    for (auto it = m_devices.rbegin(); it != m_devices.rend(); ++it) {
        (*it)->stop();
    }
}

IMotorDevice* DeviceManager::motor(const std::string& name)
{
    for (auto& d : m_devices) {
        if (name == d->name()) {
            return dynamic_cast<IMotorDevice*>(d.get());
        }
    }
    return nullptr;
}

ISensorDevice* DeviceManager::sensor(const std::string& name)
{
    for (auto& d : m_devices) {
        if (name == d->name()) {
            return dynamic_cast<ISensorDevice*>(d.get());
        }
    }
    return nullptr;
}

DeviceRegistrar::DeviceRegistrar(const std::string& driver, DeviceCreator creator)
{
    DeviceManager::instance().registerDriver(driver, creator);
}

}  // namespace stark
