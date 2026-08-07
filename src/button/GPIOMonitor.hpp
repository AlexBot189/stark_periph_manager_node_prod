#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <map>

struct gpiod_chip;

class GPIOMonitor {
public:
    using EventCallback = std::function<void(int line_num, int event_type)>;

    explicit GPIOMonitor(const std::string& chip_path);
    ~GPIOMonitor();

    bool watch_line(int line_num, EventCallback callback);
    void stop();

private:
    struct gpiod_chip* chip_;
    std::atomic<bool> running_;
    std::map<int, std::thread> monitor_threads_;
};
