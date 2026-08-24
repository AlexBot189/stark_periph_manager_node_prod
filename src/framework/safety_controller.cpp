/*
 * safety_controller.cpp — 具体安全控制器实现 (L4)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "framework/safety_controller.h"

namespace stark {

bool SafetyController::check(const SafetyContext& ctx)
{
    if (!m_motor) return true;

    const int n = m_motor->motorCount();
    if (n > (int)m_states.size()) {
        m_states.resize(n);
    }

    const uint64_t now_us = _safety_now_us();

    for (int i = 0; i < n; i++) {
        MotorFeedback fb;
        if (!m_motor->readFeedback(i, fb)) {
            continue;  /* 无反馈 (电机未在线) */
        }

        /* 过流 (mA) */
        if (m_limits.overcurrent_ma > 0 && fb.current > m_limits.overcurrent_ma) {
            fprintf(stderr, "[Safety] motor[%d] overcurrent %d mA\n", i, fb.current);
            emergencyStop();
            return false;
        }

        /* 过温 (反馈单位 0.1°C) */
        if (m_limits.overtemp_celsius > 0 &&
            fb.temperature > m_limits.overtemp_celsius * 10) {
            fprintf(stderr, "[Safety] motor[%d] overtemp %d (0.1°C)\n", i, fb.temperature);
            emergencyStop();
            return false;
        }

        /* 编码器停滞 */
        if (m_limits.encoder_stall_s > 0) {
            MotorSafetyState& st = m_states[i];
            if (fb.position == st.last_position) {
                if (st.stall_since_us == 0) {
                    st.stall_since_us = now_us;
                } else if (now_us - st.stall_since_us >
                           (uint64_t)m_limits.encoder_stall_s * 1000000ULL) {
                    fprintf(stderr, "[Safety] motor[%d] encoder stall\n", i);
                    emergencyStop();
                    return false;
                }
            } else {
                st.last_position  = fb.position;
                st.stall_since_us = 0;
            }
        }
    }

    /* 心跳超时 */
    if (m_limits.heartbeat_timeout_ms > 0 &&
        ctx.algo_heartbeat_ms > m_limits.heartbeat_timeout_ms) {
        fprintf(stderr, "[Safety] heartbeat timeout %u ms\n", ctx.algo_heartbeat_ms);
        emergencyStop();
        return false;
    }

    return true;
}

void SafetyController::emergencyStop()
{
    if (!m_motor) return;
    const int n = m_motor->motorCount();
    for (int i = 0; i < n; i++) {
        /* 急停走 PDO (非阻塞, RT 安全): 电流环 + 失能 + 电流 0 */
        MotorCommand cmd = {};
        cmd.mode    = MOTOR_MODE_CURRENT;
        cmd.flags   = 0;  /* enable=0 失能 */
        cmd.target1 = 0;
        m_motor->writeCommand(i, cmd);
    }
}

}  // namespace stark
