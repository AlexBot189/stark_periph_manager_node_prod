/*
 * RosAdapter.cpp — ROS 适配器实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 简化: 2 电机, snprintf JSON, ECO_INFO_NEW 日志.
 */
#include "RosAdapter.h"
#include "interface/IMsgInternalDispatcher.hpp"
#include "motor/motor_state.h"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <log_helper/LogHelper.h>

namespace stark_periph_manager_node
{

RosAdapter::RosAdapter(std::shared_ptr<ros::NodeHandle> nh, stark_shm_t* shm)
    : m_nh(nh), m_shm(shm), m_running(false)
{
    m_feedbackPub = nh->advertise<std_msgs::String>("/stark/motor/feedback", 10);
    m_statePub    = nh->advertise<std_msgs::String>("/stark/motor/state", 10);
    m_motorCtrlSub = nh->subscribe<std_msgs::String>(
        "/stark/motor/ctrl", 10, &RosAdapter::OnMotorCtrl, this);

    ECO_INFO_NEW("[RosAdapter] pubs/subs created");
}

RosAdapter::~RosAdapter()
{
    Stop();
}

void RosAdapter::Update(const boost::any& data)
{
    (void)data;
}

void RosAdapter::SetDispatcher(std::shared_ptr<IMsgInternalDispatcher> dispatcher)
{
    m_dispatcher = dispatcher;
}

void RosAdapter::Start()
{
    if (m_running) return;
    m_running = true;
    m_pullThread = std::thread(&RosAdapter::PullLoop, this);
    ECO_INFO_NEW("[RosAdapter] pull thread started");
}

void RosAdapter::Stop()
{
    m_running = false;
    if (m_pullThread.joinable()) {
        m_pullThread.join();
    }
}

/* PullLoop: 200Hz 从 SHM 读反馈 ,  publish */

void RosAdapter::PullLoop()
{
    while (m_running) {
        if (!m_shm) {
            usleep(5000);
            continue;
        }

        uint32_t active = __atomic_load_n(&m_shm->active_idx, __ATOMIC_ACQUIRE);
        const feedback_frame_t& fb = m_shm->fb_buffer[active];

        PubFeedback(fb);

        static int state_cnt = 0;
        if (++state_cnt >= 20) {
            state_cnt = 0;
            PubState((stark_state_t)m_shm->node_state);
        }

        usleep(5000);
    }
}

/* PubFeedback: feedback_frame_t ,  JSON ,  /stark/motor/feedback */

void RosAdapter::PubFeedback(const feedback_frame_t& fb)
{
    if (!m_shm) return;

    char buf[2048];
    int off = 0;

    off += snprintf(buf + off, sizeof(buf) - off,
        "{\"type\":\"feedback\",\"ts\":%lu,\"motor\":[",
        (unsigned long)fb.timestamp_us);

    for (int i = 0; i < STARK_MAX_MOTORS; ++i) {
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        uint8_t online = m_shm->motor_online;
        if (!(online & (1 << i))) {
            off += snprintf(buf + off, sizeof(buf) - off, "null");
            continue;
        }
        off += snprintf(buf + off, sizeof(buf) - off,
            "{\"id\":%d,\"pos\":%d,\"vel\":%d,\"cur\":%d,"
            "\"temp\":%d,\"status\":%d}",
            i + 1,
            (int)fb.motor[i].position,
            (int)fb.motor[i].velocity,
            (int)fb.motor[i].current_iq,
            (int)fb.motor[i].temperature,
            (int)fb.motor[i].status_byte);
    }

    off += snprintf(buf + off, sizeof(buf) - off,
        "],\"imu\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"latency\":{\"fb_avg\":%u,\"fb_max\":%u,\"ctrl_avg\":%u},"
        "\"severity\":%d,\"fault\":%d}",
        (double)fb.imu.roll, (double)fb.imu.pitch, (double)fb.imu.yaw,
        (double)fb.baro.pressure_hpa, (double)fb.baro.temperature_c,
        m_shm->fb_total_avg_us, m_shm->fb_total_max_us, m_shm->ctrl_total_avg_us,
        (int)m_shm->motor_severity, (int)m_shm->fault_reason);

    std_msgs::String msg;
    msg.data = buf;
    m_feedbackPub.publish(msg);
}

/* PubState */

void RosAdapter::PubState(stark_state_t state)
{
    if (!m_shm) return;

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"state\",\"state\":\"%s\",\"severity\":%d,"
        "\"reason\":%d,\"online\":%d,\"overruns\":%u}",
        state_name(state),
        (int)m_shm->motor_severity,
        (int)m_shm->fault_reason,
        (int)m_shm->motor_online,
        m_shm->cycle_overrun_count);

    std_msgs::String msg;
    msg.data = buf;
    m_statePub.publish(msg);
}

/* OnMotorCtrl: ROS ,  dispatcher->Send */

void RosAdapter::OnMotorCtrl(const std_msgs::String::ConstPtr& msg)
{
    if (!m_dispatcher) {
        ECO_WARN_NEW("[RosAdapter] OnMotorCtrl: no dispatcher");
        return;
    }
    m_dispatcher->Send(msg->data);
}

}  /* namespace stark_periph_manager_node */
