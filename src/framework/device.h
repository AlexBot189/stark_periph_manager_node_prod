/*
 * device.h — 设备抽象接口 (L1)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 借鉴内核 device model + Android HAL: 接口稳定, 实现可换。
 * 加设备 = 实现接口 + 注册; 换设备 = 换 L0 驱动实现, 上层零改动。
 */
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>

namespace stark {

/* 电机反馈 (通用, 与厂商/协议无关) */
struct MotorFeedback {
    int32_t  position;        /* 位置 */
    int32_t  velocity;        /* 速度 */
    int32_t  current;         /* 电流 Iq */
    int32_t  torque;          /* 力矩 */
    int16_t  temperature;     /* 温度 */
    uint16_t error_code;      /* 错误码 */
    uint8_t  mode;            /* 当前控制模式 */
    uint8_t  status;          /* 状态字 (使能/抱闸/故障/到位) */
    uint64_t timestamp_us;    /* 时间戳 */
};

/* 电机命令 (通用) */
struct MotorCommand {
    int32_t  target1;         /* 目标值1 (含义由 mode 决定) */
    int32_t  target2;         /* 目标值2 */
    int32_t  feedforward;     /* 前馈 */
    uint8_t  mode;            /* 控制模式 */
    uint8_t  flags;           /* 见 MOTOR_FLAG_* */
};
#define MOTOR_FLAG_ENABLE        (1u << 0)  /* 使能 */
#define MOTOR_FLAG_RELEASE_BRAKE (1u << 1)  /* 释放抱闸 */
#define MOTOR_FLAG_CLEAR_ERROR   (1u << 2)  /* 清故障 */

/* 传感器类型 (与 SHM 槽位一一对应) */
enum SensorType : uint8_t {
    SENSOR_IMU           = 1,
    SENSOR_FOOT_PRESSURE = 2,
    /* 新增类型追加 */
};

/* 传感器数据 (类型标签 + 数据指针, 指针指向设备内部缓冲, read 后短时有效) */
struct SensorData {
    uint8_t     type;         /* SensorType */
    uint32_t    size;         /* 数据字节数 */
    const void* data;         /* 指向设备内部缓冲 */
    uint64_t    timestamp_us;
};

/*
 * 设备基类 — 通用标识 + 生命周期
 *
 * 生命周期: initialize -> start -> [运行] -> stop
 *   initialize/start/stop 由非 RT 线程调用 (可阻塞、可分配)
 */
class Device {
public:
    virtual ~Device() = default;

    virtual const char* name() const = 0;    /* 实例名, 如 "motor_hip_right" */
    virtual const char* type() const = 0;    /* 类别, 如 "motor"/"imu" */

    virtual bool initialize(const nlohmann::json& config) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
};

/*
 * 电机设备接口 — RT 路径接口禁止分配内存/拿锁/阻塞
 */
class IMotorDevice : public Device {
public:
    /* 本实例驱动的电机数量 */
    virtual int motorCount() const = 0;

    /* RT: 读反馈 */
    virtual bool readFeedback(int index, MotorFeedback& fb) = 0;
    /* RT: 写命令 */
    virtual bool writeCommand(int index, const MotorCommand& cmd) = 0;

    /* 非RT: 使能/失能/清故障 */
    virtual bool enable(int index) = 0;
    virtual bool disable(int index) = 0;
    virtual bool clearFault(int index) = 0;
};

/*
 * 传感器设备接口
 */
class ISensorDevice : public Device {
public:
    /* RT: 读最新数据 */
    virtual bool read(SensorData& out) = 0;
};

}  // namespace stark
