#include "GPIOMonitor.hpp"
#include <gpiod.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <log_helper/LogHelper.h>

GPIOMonitor::GPIOMonitor(const std::string& chip_path)
    : chip_(nullptr), running_(false)
{
    /* 兼容 "gpiochipN" 和 "/dev/gpiochipN" 两种格式 */
    std::string full_path = chip_path;
    if (!chip_path.empty() && chip_path[0] != '/') {
        full_path = "/dev/" + chip_path;
    }

    chip_ = gpiod_chip_open(full_path.c_str());
    if (!chip_) {
        ECO_ERROR_NEW("[GPIOMonitor] open({}) failed: {}", full_path, strerror(errno));
    } else {
        ECO_INFO_NEW("[GPIOMonitor] opened: {}", full_path);
    }
}

GPIOMonitor::~GPIOMonitor()
{
    stop();
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

bool GPIOMonitor::watch_line(int line_num, EventCallback callback)
{
    if (!chip_) return false;

    struct gpiod_line* line = gpiod_chip_get_line(chip_, line_num);
    if (!line) {
        ECO_ERROR_NEW("[GPIOMonitor] get line {} failed", line_num);
        return false;
    }

    if (gpiod_line_request_both_edges_events(line, "btn_monitor") < 0) {
        ECO_ERROR_NEW("[GPIOMonitor] request events line {} failed", line_num);
        return false;
    }

    if (!running_) running_ = true;

    monitor_threads_[line_num] = std::thread([this, line, line_num, callback]() {
        struct gpiod_line_event event;
        struct timespec timeout = {0, 100000000}; /* 100ms, stop() 后可退出 */
        while (running_) {
            int ret = gpiod_line_event_wait(line, &timeout);
            if (ret < 0) break;
            if (ret == 0) continue;

            if (gpiod_line_event_read(line, &event) < 0) continue;

            if (callback) {
                callback(line_num, event.event_type);
            }
        }
        gpiod_line_release(line);
    });

    return true;
}

void GPIOMonitor::stop()
{
    running_ = false;
    for (auto& pair : monitor_threads_) {
        if (pair.second.joinable()) {
            pair.second.join();
        }
    }
    monitor_threads_.clear();
}
