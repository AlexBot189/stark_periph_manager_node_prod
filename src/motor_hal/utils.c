/**
 * @file utils.c
 * @brief 工具函数
 */

#include "motor_hal_types.h"
#include <sys/time.h>
#include <time.h>
#include <math.h>

/* ---------- 时间 ---------- */

uint64_t motor_utils_now_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000UL + (uint64_t)tv.tv_usec;
}

void motor_utils_sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* ---------- 编码器换算 ---------- */

float motor_utils_counts_to_deg(int16_t counts)
{
    return (float)counts * 360.0f / (float)ENCODER_LOAD_RES;
}

int16_t motor_utils_deg_to_counts(float degrees)
{
    /* 钳位 */
    if (degrees < LOAD_ANGLE_MIN) degrees = LOAD_ANGLE_MIN;
    if (degrees > LOAD_ANGLE_MAX) degrees = LOAD_ANGLE_MAX;
    return (int16_t)(degrees * ENCODER_LOAD_RES / 360.0f);
}

/* ---------- 错误码字符串 ---------- */

const char* motor_utils_error_str(uint16_t code)
{
    switch (code) {
        case 0x0000: return "OK";
        case ERR_OVER_VOLTAGE:    return "OVER_VOLTAGE";
        case ERR_UNDER_VOLTAGE:   return "UNDER_VOLTAGE";
        case ERR_OVER_TEMP:       return "OVER_TEMP";
        case ERR_STALL:           return "STALL";
        case ERR_OVERLOAD:        return "OVERLOAD";
        case ERR_CURRENT_SAMPLE:  return "CURRENT_SAMPLE_ERR";
        case ERR_POS_LIMIT:       return "POS_LIMIT";
        case ERR_NEG_LIMIT:       return "NEG_LIMIT";
        case ERR_ENCODER_TIMEOUT: return "ENCODER_TIMEOUT";
        case ERR_OVER_MAX_SPEED:  return "OVER_MAX_SPEED";
        case ERR_ELEC_ANGLE_FAIL: return "ELEC_ANGLE_INIT_FAIL";
        case ERR_POS_ERROR_LARGE: return "POS_ERROR_LARGE";
        case ERR_ENCODER_FAULT:   return "ENCODER_FAULT";
        default: return "UNKNOWN";
    }
}

/* ---------- SDO Abort Code 查表 (CiA 301, Table 52) ---------- */

const char* motor_utils_sdo_abort_str(uint32_t code)
{
    switch (code) {
        case SDO_ABORT_TOGGLE_BIT:     return "Toggle bit not alternated";
        case SDO_ABORT_TIMEOUT:        return "SDO protocol timed out";
        case SDO_ABORT_CMD_UNKNOWN:    return "Command specifier not valid";
        case SDO_ABORT_BLK_SIZE:       return "Invalid block size";
        case SDO_ABORT_UNSUPPORTED:    return "Unsupported access to object";
        case SDO_ABORT_WRITE_ONLY:     return "Attempt to read write-only object";
        case SDO_ABORT_READ_ONLY:      return "Attempt to write read-only object";
        case SDO_ABORT_NO_OBJECT:      return "Object does not exist in OD";
        case SDO_ABORT_NO_PDO_MAP:     return "Object cannot be mapped to PDO";
        case SDO_ABORT_PDO_LEN:        return "PDO length exceeded";
        case SDO_ABORT_PARAM_INCOMPAT: return "General parameter incompatibility";
        case SDO_ABORT_HW_ERR:         return "Hardware error";
        case SDO_ABORT_TYPE_MISMATCH:  return "Data type mismatch";
        case SDO_ABORT_DATA_LEN:       return "Data length too large";
        case SDO_ABORT_DATA_LEN_SHORT: return "Data length too short";
        case SDO_ABORT_NO_SUBINDEX:    return "Sub-index does not exist";
        case SDO_ABORT_VALUE_RANGE:    return "Value out of range";
        case SDO_ABORT_VALUE_TOO_HIGH: return "Value too high";
        case SDO_ABORT_VALUE_TOO_LOW:  return "Value too low";
        case SDO_ABORT_NO_STORE:       return "Cannot store to EEPROM";
        case SDO_ABORT_LOCAL_ONLY:     return "Local control by application";
        default:                       return NULL;
    }
}

/* ---------- NMT 状态字符串 ---------- */

const char* motor_utils_nmt_state_str(uint8_t state)
{
    switch (state) {
        case NMT_STATE_INITIALISING: return "Initialising";
        case NMT_STATE_STOPPED:      return "Stopped";
        case NMT_STATE_OPERATIONAL:  return "Operational";
        case NMT_STATE_PRE_OP:       return "Pre-Operational";
        default:                     return NULL;
    }
}

/* ---------- EMCY 错误码查表 ---------- */

const char* motor_utils_emcy_str(uint16_t code)
{
    switch (code) {
        case EMCY_GENERIC:         return "Generic error";
        case EMCY_CURRENT:         return "Current error";
        case EMCY_VOLTAGE:         return "Voltage error";
        case EMCY_TEMPERATURE:     return "Temperature error";
        case EMCY_HARDWARE:        return "Hardware error";
        case EMCY_SOFTWARE:        return "Software internal error";
        case EMCY_COMMUNICATION:   return "Communication error";
        case EMCY_POSITION_ERROR:  return "Position error";
        case EMCY_CAN_OVERRUN:     return "CAN overrun";
        case EMCY_CAN_PASSIVE:     return "CAN passive mode";
        case EMCY_HEARTBEAT:       return "Heartbeat lost";
        case EMCY_SYNC_TIMEOUT:    return "SYNC timeout";
        default:                   return NULL;
    }
}
