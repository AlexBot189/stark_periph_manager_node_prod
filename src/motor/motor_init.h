/*
 * CanDispatcher.h — 电机数据调度中心
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 初始化流程:
 *   1. motor_hal_create + init (CANFD)
 *   2. 注册电机 (ID 1,2)
 *   3. recv_start
 *   4. 创建 SHM
 *   5. 主循环 (事件驱动)
 *
 * 所有 SDO/PDO/OD 控制通过 StarkMotorCtrl 封装.
 * RT 控制走 StarkRtWorker ,  SHM mailbox.
 */
#pragma once

#include "interface/IMsgInternalDispatcher.hpp"
#include "interface/IListener.hpp"
#include "interface/Defines.hpp"

#include <unordered_map>
#include <memory>
#include <string>
#include <mutex>
#include <nlohmann/json.hpp>

extern "C" {
#include "stark_shm.h"
#include "shm/shm_mgr.h"
#include "motor_hal.h"
}

#include "motor/motor_ctrl.h"
#include "imu/imu_source.h"
#include "foot_pressure/FootPressureSensor.h"
#include "motor/motor_rt_worker.h"

namespace stark_periph_manager_node {

class CanDispatcher : public IMsgInternalDispatcher {
public:
    CanDispatcher();
    ~CanDispatcher();

    /* IMsgInternalDispatcher */
    bool InitDispatcher() override;
    bool DestroyDispatcher() override;
    void Send(const std::string& data) override;
    void RegisterObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void RemoveObserver(ListenerType type, std::shared_ptr<IListener> listener) override;
    void NotifyObserver(const boost::any& data) override;

    /* 获取内部实例 */
    motor_hal_t*   GetHal()       { return m_hal; }
    stark_shm_t*     GetShm()       { return m_shm; }
    StarkMotorCtrl*  GetCtrl()      { return m_ctrl.get(); }
    IImuSource*   GetImuSensor() { return m_imu_sensor; }
    FootPressureSensor* GetFootPressureSensor() { return m_foot_sensor; }

    /* 配置 (从 config.json 读取, 或默认值) */
    uint32_t          GetHeartbeatTimeoutMs() const { return m_heartbeat_timeout_ms; }
    const RtConfig&     GetRtConfig()      { return m_rt_cfg; }
    const std::string&  GetShmName()       { return m_shm_name; }
    const std::string&  GetCanIface()      { return m_can_iface; }

    /* 配置获取 (主线程读取, 设置 g_ctx) */
    int  GetMotorCount()     const { return m_motor_count; }
    int  GetCalibTimeoutMs() const { return m_calib_timeout_ms; }
    uint16_t GetSensorPeriodMs()   const { return m_sensor_period_ms; }
    uint16_t GetSensorPeriodDiv()  const { return m_sensor_period_div; }
    uint8_t  GetSensorBusFormat()  const { return m_sensor_bus_format; }
    uint8_t  GetSensorMode()        const { return m_sensor_mode; }
    uint8_t  GetSensorForceModule() const { return m_sensor_force_module; }
    bool     GetReportAutoEnable()   const { return m_report_auto_enable; }
    uint32_t GetReportPeriodMs()     const { return m_report_period_ms; }
    const std::string& GetReportDataSource() const { return m_report_data_source; }
    bool     GetMotorAutoEnable()  const { return m_motor_auto_enable; }
    int      GetLedMotorId()       const { return m_led_motor_id; }
    const std::string& GetBtnCalibChip()  const { return m_btn_calib_chip; }
    int      GetBtnCalibLine()  const { return m_btn_calib_line; }
    int      GetBtnCalibLongPressMs() const { return m_btn_calib_long_press_ms; }
    const std::string& GetBtnReportChip() const { return m_btn_report_chip; }
    int      GetBtnReportLine() const { return m_btn_report_line; }
    bool     GetWebEnabled()   const { return m_web_enabled; }
    uint16_t GetWebPort()      const { return m_web_port; }
    uint32_t GetWebPushPeriodMs() const { return m_web_push_period_ms; }

    bool IsRunning() const { return m_running; }
    void SetConfigPath(const std::string& path) { m_config_path = path; }

private:
    bool LoadMotorConfig();
    void _dispatch_command(const std::string& cmd, uint8_t id, int value);

    motor_hal_t*    m_hal;
    stark_shm_mgr_t*  m_shm_mgr;
    stark_shm_t*      m_shm;
    bool            m_running;

    std::unique_ptr<StarkMotorCtrl> m_ctrl;

    std::mutex m_listener_mutex;
    std::unordered_map<ListenerType, std::shared_ptr<IListener>> m_listeners;
    std::string m_config_path;

    /* 配置 (优先 config.json, 读失败则默认值) */
    uint32_t     m_heartbeat_timeout_ms = 1000;  /* 算法心跳超时 (写 SHM) */
    RtConfig     m_rt_cfg;
    std::string  m_shm_name  = STARK_SHM_NAME;
    size_t       m_shm_size_bytes = STARK_SHM_SIZE;
    std::string  m_can_iface = "can0";
    int          m_can_arb_rate  = 1000000;
    int          m_can_data_rate = 5000000;

    /* IMU (底层对象由 DeviceManager 持有, 这里只存指针) */
    IImuSource*   m_imu_sensor = nullptr;
    ImuConfig     m_imu_cfg;

    /* 足底压力传感器 (底层对象由 DeviceManager 持有, 这里只存指针) */
    FootPressureSensor* m_foot_sensor = nullptr;
    bool        m_foot_enabled   = true;
    std::string m_foot_uart_dev  = "/dev/ttyS7";
    int         m_foot_baud_rate = 460800;
    int         m_foot_timeout_ms = 10;

    /* 电机数量 (从 config.json 读取, ≤ STARK_MAX_MOTORS) */
    int          m_motor_count    = 2;
    nlohmann::json m_motors_json;   /* 电机配置数组 (传给 MotorCanfd) */

    /* 校准/透传配置 (来自 config.json) */
    int          m_calib_timeout_ms = 10000;
    uint16_t     m_sensor_period_ms = 1;
    uint16_t     m_sensor_period_div = 1;   /* 0.5ms 基准分频, 默认 1 */
    uint8_t      m_sensor_bus_format = 3;  /* CANFD BRS */
    uint8_t      m_sensor_mode = 2;         /* 0=关 1=仅传感器帧 2=全部帧 */
    uint8_t      m_sensor_force_module = 1; /* 0=CAN力矩 1=SPI力矩 */
    bool         m_report_auto_enable = true;  /* 校准后自动开启周期上报 */
    uint32_t     m_report_period_ms   = 5;    /* 上报周期 ms */
    std::string  m_report_data_source = "mixed"; /* 数据来源: mixed(混合/默认) | unified_6c0(统一6C0) */
    bool         m_log_onoff           = false; /* 调试日志开关: 默认关闭, config中 log_onoff=true 开启 */
    bool         m_motor_auto_enable  = false; /* 任意电机 auto_enable=true 则置 true */
    int          m_led_motor_id = 0;          /* 0=禁用, 1=右, 2=左 */

    /* 按键 */
    std::string  m_btn_calib_chip;
    int          m_btn_calib_line  = -1;
    int          m_btn_calib_long_press_ms = 5000;
    std::string  m_btn_report_chip;
    int          m_btn_report_line = -1;

    /* WebSocket */
    bool         m_web_enabled = false;
    uint16_t     m_web_port    = 8080;
    uint32_t     m_web_push_period_ms = 5;  /* push rate, default 5ms = 200Hz */
};

}  /* namespace stark_periph_manager_node */
