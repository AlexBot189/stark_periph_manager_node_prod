#include "button_handler.hpp"
#include "GPIOMonitor.hpp"
#include "stark_shm.h"
#include <gpiod.h>
#include <time.h>
#include <log_helper/LogHelper.h>

static constexpr int64_t  DEBOUNCE_US  = 100000;
static constexpr int64_t  CLICK_WIN_US = 2000000;
static constexpr int      CLICK_COUNT  = 3;

uint64_t ButtonHandler::nowUs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
}

ButtonHandler::ButtonHandler(const Config& calib, const Config& report,
                             stark_shm_t* shm, int motor_count)
    : m_shm(shm), m_motor_count(motor_count)
{
    if (!calib.gpio_chip.empty() && calib.line >= 0) {
        m_calib_mon.reset(new GPIOMonitor(calib.gpio_chip));
        m_calib_mon->watch_line(calib.line,
            [this](int line, int ev) { onCalibEdge(line, ev); });
    }

    if (!report.gpio_chip.empty() && report.line >= 0) {
        m_report_mon.reset(new GPIOMonitor(report.gpio_chip));
        m_report_mon->watch_line(report.line,
            [this](int line, int ev) { onReportEdge(line, ev); });
    }
}

ButtonHandler::~ButtonHandler()
{
    if (m_calib_mon)  m_calib_mon->stop();
    if (m_report_mon) m_report_mon->stop();
}

/* GPIO thread callback */
void ButtonHandler::onCalibEdge(int line, int event_type)
{
    (void)line;
    uint64_t now = nowUs();

    /* debounce */
    uint64_t last = m_calib_last_edge_us.load(std::memory_order_relaxed);
    if (last && now - last < (uint64_t)DEBOUNCE_US) return;
    m_calib_last_edge_us.store(now, std::memory_order_relaxed);

    bool is_press = (event_type == GPIOD_LINE_EVENT_FALLING_EDGE);

    if (is_press) {
        m_calib_pressed.store(true, std::memory_order_release);

        int cnt = m_calib_cnt.load(std::memory_order_relaxed);
        uint64_t first = m_calib_first_us.load(std::memory_order_relaxed);

        if (cnt == 0 || (first && now - first > (uint64_t)CLICK_WIN_US)) {
            m_calib_cnt.store(1, std::memory_order_relaxed);
            m_calib_first_us.store(now, std::memory_order_relaxed);
        } else {
            cnt++;
            m_calib_cnt.store(cnt, std::memory_order_relaxed);
            if (cnt >= CLICK_COUNT) {
                m_calib_trig.store(true, std::memory_order_release);
                m_calib_cnt.store(0, std::memory_order_relaxed);
                m_calib_first_us.store(0, std::memory_order_relaxed);
            }
        }
    } else {
        m_calib_pressed.store(false, std::memory_order_release);
    }
}

/* GPIO thread callback */
void ButtonHandler::onReportEdge(int line, int event_type)
{
    (void)line;
    uint64_t now = nowUs();
    bool is_press = (event_type == GPIOD_LINE_EVENT_FALLING_EDGE);

    /* 上升沿/下降沿独立去抖, 避免释放反弹的下降沿过滤掉真正的上升沿 */
    std::atomic<uint64_t>& last_ref = is_press ? m_report_fall_us : m_report_rise_us;
    uint64_t last = last_ref.load(std::memory_order_relaxed);
    if (last && now - last < (uint64_t)DEBOUNCE_US) return;
    last_ref.store(now, std::memory_order_relaxed);

    uint8_t state = is_press ? 1 : 0;

    __atomic_store_n(&m_shm->btn_report_state, state, __ATOMIC_RELEASE);
    if (is_press) {
        __atomic_add_fetch(&m_shm->btn_report_seq, 1, __ATOMIC_RELEASE);
    }
}

/* main_loop 50ms poll */
void ButtonHandler::poll()
{
    uint64_t now = nowUs();

    /* 3-click toggle calib */
    if (m_calib_trig.load(std::memory_order_acquire)) {
        m_calib_trig.store(false, std::memory_order_relaxed);
        __atomic_store_n(&m_shm->calib_requested, 1, __ATOMIC_RELEASE);
        ECO_INFO_NEW("[BTN] calib triggered");
    }
}
