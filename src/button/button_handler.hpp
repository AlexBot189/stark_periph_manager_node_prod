#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <cstdint>
#include <functional>

extern "C" {
#include "stark_shm.h"
}

class GPIOMonitor;

class ButtonHandler {
public:
    struct Config {
        std::string gpio_chip;
        int         line;
        int         long_press_ms = 5000;   /* 长按阈值, 按住满该时长触发长按动作 */
    };

    /* 长按动作回调 (可插拔: 关机/其他动作由调用方决定) */
    using LongPressCallback = std::function<void()>;

    ButtonHandler(const Config& calib, const Config& report,
                  stark_shm_t* shm, int motor_count,
                  LongPressCallback long_press_cb = nullptr);
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
    int m_long_press_ms;
    LongPressCallback m_long_press_cb;

    /* calib button state (GPIO thread write, poll() read) */
    std::atomic<int>      m_calib_cnt{0};
    std::atomic<uint64_t> m_calib_first_us{0};
    std::atomic<bool>     m_calib_pressed{false};
    std::atomic<bool>     m_calib_trig{false};
    std::atomic<uint64_t> m_calib_last_edge_us{0};
    std::atomic<uint64_t> m_calib_press_us{0};   /* 本次按下时刻, 用于长按计时 */

    /* report button state */
    std::atomic<uint64_t> m_report_fall_us{0};  /* 下降沿去抖 */
    std::atomic<uint64_t> m_report_rise_us{0};  /* 上升沿去抖 */
};
