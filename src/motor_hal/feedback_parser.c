/**
 * @file feedback_parser.c
 * @brief 反馈帧解析 - 从 CAN 帧提取电机状态
 */

#include "canopen_frames.h"

void feedback_parse_raw(const canfd_frame_t *f, motor_feedback_t *out)
{
    canopen_parse_feedback(f, out);
}

/* 便捷方法: 从 feedback_t 提取位字段 */
bool feedback_is_enabled(const motor_feedback_t *fb) {
    return fb && (fb->status_byte & 0x80);
}

bool feedback_brake_released(const motor_feedback_t *fb) {
    return fb && (fb->status_byte & 0x40);
}

bool feedback_has_error(const motor_feedback_t *fb) {
    return fb && (fb->status_byte & 0x20);
}

bool feedback_target_reached(const motor_feedback_t *fb) {
    return fb && (fb->status_byte & 0x10);
}

const char* feedback_error_string(uint16_t err) {
    switch (err) {
        case 0x0000: return "None";
        case ERR_OVER_VOLTAGE:    return "OverVoltage";
        case ERR_UNDER_VOLTAGE:   return "UnderVoltage";
        case ERR_OVER_TEMP:       return "OverTemperature";
        case ERR_STALL:           return "Stall";
        case ERR_OVERLOAD:        return "Overload";
        case ERR_CURRENT_SAMPLE:  return "CurrentSampleError";
        case ERR_POS_LIMIT:       return "PositiveLimit";
        case ERR_NEG_LIMIT:       return "NegativeLimit";
        case ERR_ENCODER_TIMEOUT: return "EncoderTimeout";
        case ERR_OVER_MAX_SPEED:  return "OverMaxSpeed";
        case ERR_ELEC_ANGLE_FAIL: return "ElecAngleInitFail";
        case ERR_POS_ERROR_LARGE: return "PositionErrorLarge";
        case ERR_ENCODER_FAULT:   return "EncoderFault";
        default: return "Unknown";
    }
}
