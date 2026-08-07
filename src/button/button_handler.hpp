#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <cstdint>

extern "C" {
#include "stark_shm.h"
}

class GPIOMonitor;

class ButtonHandler {
public:
    struct Config {
        std::string gpio_chip;
        int         line;
    };

    ButtonHandler(const Config& calib, const Config& report,
                  stark_shm_t* shm, int motor_count);
    ~ButtonHandler();

    ButtonHandler(const ButtonHandler&) = delete;
    ButtonHandler& operator=(const ButtonHandler&) = delete;

    void poll();

private:
    void onCalibEdge(int line, int event_type);
    void onReportEdge(int line, int event_type);
    static uint64_t nowUs();

    std::unique_ptr<GPIOMonitor> m_calib_mon;
    std::unique_ptr<GPIOMonitor> m_report_mon;
    stark_shm_t* m_shm;
    int m_motor_count;

    /* calib button state (GPIO thread write, poll() read) */
    std::atomic<int>      m_calib_cnt{0};
    std::atomic<uint64_t> m_calib_first_us{0};
    std::atomic<bool>     m_calib_pressed{false};
    std::atomic<bool>     m_calib_trig{false};
    std::atomic<uint64_t> m_calib_last_edge_us{0};

    /* report button state */
    std::atomic<uint64_t> m_report_fall_us{0};  /* 下降沿去抖 */
    std::atomic<uint64_t> m_report_rise_us{0};  /* 上升沿去抖 */
};
