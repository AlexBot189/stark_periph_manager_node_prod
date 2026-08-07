/*
 * @file WebServer.h
 * @brief WebSocket 调试服务器 — 从 SHM 读取反馈帧, 推送到 WebSocket 客户端
 *
 * ## 定位
 *
 *   WebServer 是一个调试/监控组件, 提供机器人关节数据的实时可视化。
 *   不参与控制闭环, 仅消费 SHM fb_buffer 数据。
 *
 * ## 数据流
 *
 *   motor_node ,  SHM fb_buffer ,  PullLoop (200Hz) ,  JSON ,  WebSocket ,  浏览器
 *
 * ## 依赖
 *
 *   - stark_shm_t* (共享内存)
 *   - crow WebSocket 库 (crow_all.h)
 *   - ENABLE_WEBSERVER 编译开关
 *
 * ## 线程模型
 *
 *   PullLoop 在独立线程中运行, 以 200Hz 周期从 SHM 双 Buffer 读数据,
 *   序列化为 JSON 后推送到所有连接的 WebSocket 客户端。
 */
#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <cstdint>
#include "interface/IListener.hpp"
#include "stark_shm.h"

extern "C" {
#include "motor_hal.h"
}

namespace stark_periph_manager_node {

class WebServer : public IListener {
public:
    /**
     * @param shm  共享内存指针 (stark_shm_mgr_t->ptr)
     * @param port HTTP/WebSocket 端口, 默认 8080
     */
    WebServer(stark_shm_t* shm, uint16_t port = 8080, uint32_t push_period_ms = 5);
    ~WebServer();

    /* ── IListener 接口 ── */
    void Update(const boost::any& data) override;

    /* ── 生命周期 ── */
    void Start();
    void Stop();

    /* ── candump RX ── */
    void StartCandump(const char* iface);
    void StopCandump();
    void CandumpLoop();  /* thread func */
    bool IsRunning() const { return m_running.load(std::memory_order_acquire); }

    /* ── 设置 motor_hal (用于 SDO 控制命令) ── */
    void SetMotorHal(motor_hal_t* hal) { m_motor_hal = hal; }

private:
    /**
     * @brief 主循环 — 200Hz 从 SHM 读反馈帧, JSON 序列化, push 到 WebSocket
     *
     * 流程:
     *   1. atomic_load(active_idx, acquire)  ,   读活跃 Buffer 索引
     *   2. memcpy fb_buffer[active]  ,   防撕裂快照
     *   3. 序列化 feedback_frame_t ,  JSON 字符串
     *   4. 遍历 websocket_clients, push(json)
     *   5. usleep(5000)  ,   200Hz
     */
    void PullLoop();

    /* ── 外部依赖 ── */
    stark_shm_t*      m_shm;
    motor_hal_t*    m_motor_hal = nullptr;
    uint16_t        m_port;
    uint32_t        m_push_period_us;  /* push interval in microseconds */

    /* ── 网络 ── */
    int              m_listen_fd = -1;
    std::vector<int> m_clients;         /* WebSocket 客户端 fd 列表 */
    std::mutex       m_clients_mutex;

    /* ── 线程控制 ── */
    std::atomic<bool> m_running;
    std::atomic<bool> m_push_enabled{true};  /* 数据推送开关 */
    std::thread     m_thread;

    /* ── 统计 ── */
    uint64_t m_frame_count;         /* 总推送帧数 */
    uint64_t m_fail_count;          /* 推送失败次数 */

    /* ── 指令追踪 (用于图表显示下发的目标值) ── */
public:
    struct CmdTrack {
        int32_t cur_m1 = 0;   /* mA */
        int32_t cur_m2 = 0;
        int32_t pos_m1 = 0;   /* °×100 */
        int32_t pos_m2 = 0;
        int32_t vel_m1 = 0;   /* RPM */
        int32_t vel_m2 = 0;
        int32_t tq_m1  = 0;   /* 0.05N.m */
        int32_t tq_m2  = 0;
        bool    cur_valid_m1 = false;
        bool    cur_valid_m2 = false;
        bool    pos_valid_m1 = false;
        bool    pos_valid_m2 = false;
        bool    vel_valid_m1 = false;
        bool    vel_valid_m2 = false;
        bool    tq_valid_m1  = false;
        bool    tq_valid_m2  = false;
    } m_last_cmd;

    /* ── candump RX ── */
    FILE*            m_candump_fp = nullptr;
    std::thread      m_candump_thread;
    std::atomic<bool> m_candump_running{false};
    static constexpr int CANDUMP_BUF_SIZE = 32;
    std::vector<std::string> m_can_rx_buf;    /* ring buffer of raw CAN lines */
    std::mutex        m_can_rx_mutex;
    uint32_t          m_can_rx_seq = 0;       /* 用于前端去重 */
};

}  /* namespace stark_periph_manager_node */
