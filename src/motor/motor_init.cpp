/*
 * CanDispatcher.cpp — 电机数据调度中心实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 所有 SDO/PDO/OD 通过 StarkMotorCtrl 封装.
 */
#include "motor/motor_init.h"
#include "imu/imu_sensor.h"
#include "foot_pressure/FootPressureSensor.h"
#include "framework/device_manager.h"
#include "framework/motor_canfd.h"
#include "nlohmann/json.hpp"

#include <cstring>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <log_helper/LogHelper.h>

namespace stark_periph_manager_node {

#include "shm/shm_mgr.h"
extern "C" {
stark_shm_mgr_t* stark_shm_mgr_open(const char* name, bool create, size_t size);
void            stark_shm_mgr_close(stark_shm_mgr_t* mgr);
}

CanDispatcher::CanDispatcher()
    : m_hal(nullptr), m_shm(nullptr), m_running(false)
    , m_config_path("stark_periph_node/config/stark_config.json")
{
}

CanDispatcher::~CanDispatcher()
{
    if (m_running) DestroyDispatcher();
}

/* InitDispatcher() — CANFD + 电机注册 + recv + SHM */

bool CanDispatcher::InitDispatcher()
{
    if (m_running) return false;

    /* 1. 读取配置 (can/motors/其他) */
    if (!LoadMotorConfig()) {
        ECO_ERROR_NEW("[CanDispatcher] LoadMotorConfig() failed");
        return false;
    }

    /* 2. 创建设备 (DeviceManager + MotorCanfd, 电机数由配置决定) */
    nlohmann::json devices = nlohmann::json::array();
    {
        nlohmann::json motor_entry;
        motor_entry["name"]                = "motor";
        motor_entry["driver"]              = "motor_canfd";
        motor_entry["config"]["can_iface"] = m_can_iface;
        motor_entry["config"]["arb_rate"]  = m_can_arb_rate;
        motor_entry["config"]["data_rate"] = m_can_data_rate;
        motor_entry["config"]["motors"]    = m_motors_json;
        devices.push_back(motor_entry);
    }
    stark::DeviceManager::instance().loadDevices(devices);

    auto* motor_dev = dynamic_cast<stark::MotorCanfd*>(
        stark::DeviceManager::instance().motor("motor"));
    if (!motor_dev) {
        ECO_ERROR_NEW("[CanDispatcher] motor device not created (check can/motors config)");
        return false;
    }
    m_hal = motor_dev->hal();
    ECO_INFO_NEW("[CanDispatcher] CANFD {}: arb={}bps data={}bps ({} motor(s))",
                 m_can_iface, m_can_arb_rate, m_can_data_rate, m_motor_count);

    /* 3.5 调试日志桥 (根据 config: log_onoff) */
    if (m_log_onoff) {
        motor_hal_set_log_callback([](const char *msg) {
            ECO_INFO_NEW("[PROTO] {}", msg);
        });
        ECO_INFO_NEW("[CanDispatcher] log_onoff ENABLED");
    }

    /* 4. 设置接收线程实时参数 */
    motor_hal_recv_set_rt(m_hal, m_rt_cfg.enable_rt, m_rt_cfg.recv_priority);

    /* 4.5 接收线程绑核 (与 RT 线程物理隔离, 避免同核 CPU/锁竞争) */
    motor_hal_recv_set_affinity(m_hal, m_rt_cfg.recv_cpu);

    /* 5. 启动接收线程 */
    int ret = motor_hal_recv_start(m_hal);
    if (ret < 0) {
        ECO_ERROR_NEW("[CanDispatcher] motor_hal_recv_start() failed: {}", ret);
        stark::DeviceManager::instance().stopAll(); m_hal = nullptr;
        return false;
    }

    /* 6. 创建 StarkMotorCtrl 封装 */
    m_ctrl = std::make_unique<StarkMotorCtrl>(m_hal);

    /* 7. 初始化 IMU (driver 决定具体实现, 配置已在 LoadMotorConfig 中读取) */
    if (m_imu_cfg.driver == "bosch") {
        ECO_WARN_NEW("[CanDispatcher] IMU driver 'bosch' not implemented, running without IMU");
    } else {
        m_imu_sensor = std::make_unique<ImuHALSensor>();
        if (!m_imu_sensor->Init(m_imu_cfg)) {
            ECO_WARN_NEW("[CanDispatcher] IMU init failed, running without IMU");
        }
    }

    /* 7.5 初始化足底压力传感器 */
    if (m_foot_enabled) {
        m_foot_sensor = std::make_unique<FootPressureSensor>();
        if (!m_foot_sensor->Init(m_foot_uart_dev.c_str(), m_foot_baud_rate,
                                  m_foot_timeout_ms)) {
            ECO_WARN_NEW("[CanDispatcher] FootPressure init failed, running without it");
        }
    } else {
        ECO_INFO_NEW("[CanDispatcher] FootPressure disabled (foot_pressure.enabled=false)");
    }

    /* 8. 打开共享内存 */
    m_shm_mgr = stark_shm_mgr_open(m_shm_name.c_str(), true, m_shm_size_bytes);
    if (!m_shm_mgr) {
        ECO_ERROR_NEW("[CanDispatcher] stark_shm_mgr_open() failed");
        stark::DeviceManager::instance().stopAll(); m_hal = nullptr;
        return false;
    }
    m_shm = (stark_shm_t*)m_shm_mgr->ptr;

    /* 全量清零 SHM — 防止跨进程残留触发假 FAULT */
    memset(m_shm, 0, STARK_SHM_SIZE);
    m_shm->magic      = STARK_SHM_MAGIC;
    m_shm->version    = STARK_SHM_VERSION;
    m_shm->node_state = STATE_BOOTING;

    m_running = true;
    ECO_INFO_NEW("[CanDispatcher] ready");
    return true;
}

/* DestroyDispatcher() */

bool CanDispatcher::DestroyDispatcher()
{
    if (!m_running) return true;
    m_running = false;

    /* PDO 安全停机: estop + torque=0 */
    if (m_hal) {
        for (uint8_t id = 1; id <= (uint8_t)m_motor_count; id++) {
            motor_hal_pdo_estop(m_hal, id);        /* pdo_byte0 enable=0, bus=0 */
            motor_hal_set_torque(m_hal, id, 0);     /* 发 PDO 帧: enable=0, torque=0 */
        }
    }

    /* 停止 IMU HAL */
    if (m_imu_sensor) {
        m_imu_sensor->Deinit();
        m_imu_sensor.reset();
    }

    /* 停止足底压力传感器 */
    if (m_foot_sensor) {
        m_foot_sensor->Deinit();
        m_foot_sensor.reset();
    }

    /* 停止设备 (MotorCanfd::stop → recv_stop + destroy) */
    stark::DeviceManager::instance().stopAll();
    m_hal = nullptr;
    m_ctrl.reset();

    /* 关闭 SHM */
    if (m_shm) {
        stark_shm_mgr_close(m_shm_mgr);
        m_shm_mgr = nullptr;
        m_shm = nullptr;
    }

    ECO_INFO_NEW("[CanDispatcher] stopped");
    return true;
}

/*
 * Send() — JSON 控制命令 ,  StarkMotorCtrl (SDO 路径, 非实时)
 */

void CanDispatcher::Send(const std::string& data)
{
    if (!m_hal || !m_running || !m_ctrl) return;

    try {
        auto j = nlohmann::json::parse(data);

        std::string cmd = j.value("cmd", std::string{});
        int motor_id    = j.value("motor_id", 0);
        int value       = j.value("value", 0);

        if (cmd.empty()) {
            ECO_WARN_NEW("[CanDispatcher] Send: missing 'cmd'");
            return;
        }

        _dispatch_command(cmd, (uint8_t)motor_id, value);

    } catch (const nlohmann::json::parse_error& e) {
        ECO_ERROR_NEW("[CanDispatcher] JSON parse: {}", e.what());
    } catch (const std::exception& e) {
        ECO_ERROR_NEW("[CanDispatcher] Send exception: {}", e.what());
    }
}

void CanDispatcher::_dispatch_command(const std::string& cmd, uint8_t id, int val)
{
    if (cmd == "torque") {
        m_ctrl->Torque(id, val);
    }
    else if (cmd == "speed") {
        m_ctrl->Speed(id, val);
    }
    else if (cmd == "position") {
        m_ctrl->AbsPosition(id, val);
    }
    else if (cmd == "stop") {
        if (id == 0) {
            for (uint8_t i = 1; i <= (uint8_t)m_motor_count; i++) m_ctrl->AbsStop(i);
        } else {
            m_ctrl->AbsStop(id);
        }
    }
    else if (cmd == "enable") {
        m_ctrl->Enable(id);
    }
    else if (cmd == "disable") {
        m_ctrl->Disable(id);
    }
    else if (cmd == "fault_reset") {
        m_ctrl->FaultReset(id);
    }
    else if (cmd == "setzero") {
        m_ctrl->SetZero(id);
    }
    else if (cmd == "save") {
        m_ctrl->SaveFlash(id);
    }
    else if (cmd == "reboot") {
        m_ctrl->Reboot(id);
    }
    else if (cmd == "pid") {
        ECO_WARN_NEW("[CanDispatcher] pid command needs full JSON fields, use stark_motor_ctrl API directly");
    }
    else {
        ECO_WARN_NEW("[CanDispatcher] unknown cmd: {}", cmd);
    }
}

/* Observer */

void CanDispatcher::RegisterObserver(ListenerType type,
                                     std::shared_ptr<IListener> listener)
{
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    m_listeners[type] = listener;
}

void CanDispatcher::RemoveObserver(ListenerType type,
                                   std::shared_ptr<IListener> listener)
{
    (void)listener;
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    m_listeners.erase(type);
}

void CanDispatcher::NotifyObserver(const boost::any& data)
{
    std::lock_guard<std::mutex> lock(m_listener_mutex);
    for (auto& kv : m_listeners) {
        if (kv.second) kv.second->Update(data);
    }
}

/* LoadMotorConfig() — 从 config.json 读取, 失败则用硬编码默认值 */

bool CanDispatcher::LoadMotorConfig()
{
    std::ifstream ifs(m_config_path);
    if (ifs.is_open()) {
        nlohmann::json cfg;
        try { ifs >> cfg; }
        catch (const nlohmann::json::parse_error& e) {
            ECO_ERROR_NEW("[CanDispatcher] config parse: {}", e.what());
            return false;
        }
        ifs.close();

        /* 解析 CAN */
        if (cfg.contains("can")) {
            m_can_iface    = cfg["can"].value("interface",        "can0");
            m_can_arb_rate = cfg["can"].value("arbitration_rate", 1000000);
            m_can_data_rate= cfg["can"].value("data_rate",        5000000);
        }

        /* 解析 motors (配置存入 m_motors_json, 由 MotorCanfd 注册) */
        if (cfg.contains("motors") && cfg["motors"].is_array()) {
            m_motor_count = (int)cfg["motors"].size();
            if (m_motor_count > STARK_MAX_MOTORS) {
                ECO_WARN_NEW("[CanDispatcher] config has {} motors, clamping to {}",
                             m_motor_count, STARK_MAX_MOTORS);
                m_motor_count = STARK_MAX_MOTORS;
            }
            ECO_INFO_NEW("[CanDispatcher] motor count from config: {}", m_motor_count);
            m_motors_json = cfg["motors"];
            for (const auto& m : cfg["motors"]) {
                if (m.value("auto_enable", false)) m_motor_auto_enable = true;
            }
        }

        /* 解析 safety */
        if (cfg.contains("safety")) {
            auto& s = cfg["safety"];
            m_safety_cfg.heartbeat_timeout_ms = s.value("heartbeat_timeout_ms", 1000u);
            m_safety_cfg.overtemp_celsius  = s.value("overtemp_celsius",  80);
            m_safety_cfg.can_offline_ms    = s.value("can_offline_ms",    2000u);
            m_safety_cfg.encoder_stall_s   = s.value("encoder_stall_s",   3u);
        }

        /* 解析 rt */
        if (cfg.contains("rt")) {
            auto& r = cfg["rt"];
            m_rt_cfg.priority      = r.value("control_priority",  90);
            m_rt_cfg.recv_priority  = r.value("recv_priority",     85);
            m_rt_cfg.recv_cpu       = r.value("recv_cpu",          -1);
            m_rt_cfg.period_us     = r.value("control_period_us", 1000u);
            m_rt_cfg.report_divider = r.value("report_divider",    5);
            m_rt_cfg.enable_rt     = r.value("enable_rt", true);
            m_rt_cfg.sync_enable   = r.value("sync_enable", true);
            m_rt_cfg.sync_priority = r.value("sync_priority", 98);
            m_rt_cfg.sync_cpu      = r.value("sync_cpu", 1);
            m_rt_cfg.perf_trace    = r.value("perf_trace", true);
            if (r.contains("cpu_affinity") && r["cpu_affinity"].is_array()
                && r["cpu_affinity"].size() > 0) {
                m_rt_cfg.cpu_affinity[0] = r["cpu_affinity"][0].get<int>();
                m_rt_cfg.cpu_affinity[1] = (r["cpu_affinity"].size() > 1)
                    ? r["cpu_affinity"][1].get<int>() : -1;
            }
        }

        /* 解析 shm */
        if (cfg.contains("shm")) {
            m_shm_name = cfg["shm"].value("name", std::string(STARK_SHM_NAME));
            size_t kb  = cfg["shm"].value("size_kb", (size_t)(STARK_SHM_SIZE / 1024));
            m_shm_size_bytes = kb * 1024;
            if (m_shm_size_bytes < sizeof(stark_shm_t)) {
                ECO_WARN_NEW("[CanDispatcher] shm.size_kb={} < struct size={}, clamping",
                             kb, sizeof(stark_shm_t));
                m_shm_size_bytes = sizeof(stark_shm_t);
            }
        }

        /* 解析 calib */
        if (cfg.contains("calib")) {
            auto& c = cfg["calib"];
            m_calib_timeout_ms = c.value("timeout_ms", 10000);
        }

        /* 解析 sensor */
        if (cfg.contains("sensor")) {
            auto& s = cfg["sensor"];
            m_sensor_period_ms = s.value("period_ms", 1u);
            m_sensor_bus_format = s.value("bus_format", 3u);  /* CANFD BRS */
            m_sensor_mode = (uint8_t)s.value("mode", 3u);
            m_sensor_force_module = (uint8_t)s.value("force_module", 1u);
            /* 优先直接配 period_div (0.5ms 基准, 默认1=2000Hz);
             * 兼容旧 period_ms (×2 换算到 0.5ms 基准) */
            if (s.contains("period_div"))
                m_sensor_period_div = s.value("period_div", 1u);
            else if (s.contains("period_ms"))
                m_sensor_period_div = (uint16_t)(m_sensor_period_ms * 2);
            else
                m_sensor_period_div = 1;
        }

        /* 解析 imu */
        if (cfg.contains("imu")) {
            auto& imu_cfg = cfg["imu"];
            m_imu_cfg.driver       = imu_cfg.value("driver",    std::string("invensense"));
            m_imu_cfg.interface    = imu_cfg.value("interface", std::string("i2c"));
            m_imu_cfg.i2c_dev      = imu_cfg.value("i2c_dev",   std::string("/dev/i2c-3"));
            m_imu_cfg.spi_dev      = imu_cfg.value("spi_dev",   std::string("/dev/spidev0.0"));
            m_imu_cfg.spi_speed_hz = imu_cfg.value("spi_speed_hz", 8000000u);
            m_imu_cfg.spi_mode     = imu_cfg.value("spi_mode",  0u);
            m_imu_cfg.gpio_chip    = imu_cfg.value("gpio_chip", std::string("gpiochip4"));
            m_imu_cfg.gpio_line    = imu_cfg.value("gpio_line", 6u);
            m_imu_cfg.op_mode      = imu_cfg.value("op_mode",   5);
        }

        /* 解析 foot_pressure */
        if (cfg.contains("foot_pressure")) {
            auto& fp = cfg["foot_pressure"];
            m_foot_enabled   = fp.value("enabled",    true);
            m_foot_uart_dev  = fp.value("uart_dev",   std::string("/dev/ttyS7"));
            m_foot_baud_rate = fp.value("baud_rate",  460800);
            m_foot_timeout_ms = fp.value("timeout_ms", 10);
        }

        /* 解析 report */
        if (cfg.contains("report")) {
            auto& rpt = cfg["report"];
            m_report_auto_enable = rpt.value("auto_enable", true);
            m_report_period_ms   = rpt.value("period_ms",   5u);
            m_report_data_source = rpt.value("data_source", std::string("mixed"));
            m_log_onoff          = rpt.value("log_onoff",  false);
        }

        /* 解析 led */
        if (cfg.contains("led")) {
            m_led_motor_id = cfg["led"].value("motor_id", 0);
            ECO_INFO_NEW("[CanDispatcher] led motor_id={} (0=disabled)", m_led_motor_id);
        }

        /* 解析 buttons */
        if (cfg.contains("buttons")) {
            auto& btn = cfg["buttons"];
            if (btn.contains("calib")) {
                m_btn_calib_chip = btn["calib"].value("gpio_chip", std::string{});
                m_btn_calib_line = btn["calib"].value("line", -1);
                m_btn_calib_long_press_ms = btn["calib"].value("long_press_ms", 5000);
            }
            if (btn.contains("report")) {
                m_btn_report_chip = btn["report"].value("gpio_chip", std::string{});
                m_btn_report_line = btn["report"].value("line", -1);
            }
            ECO_INFO_NEW("[CanDispatcher] buttons calib={}:{} report={}:{}",
                         m_btn_calib_chip, m_btn_calib_line,
                         m_btn_report_chip, m_btn_report_line);
        }

        /* 解析 web (WebSocket 调试) */
        if (cfg.contains("web")) {
            m_web_enabled = cfg["web"].value("enabled", false);
            m_web_port    = cfg["web"].value("port", 8080);
            m_web_push_period_ms = cfg["web"].value("push_period_ms", 5);
            ECO_INFO_NEW("[CanDispatcher] web enabled={} port={} push_period={}ms", m_web_enabled, m_web_port, m_web_push_period_ms);
        }

        return true;  /* 文件解析完成, 缺失字段用默认值 */
    }

    /* 配置文件不存在 ,  全部默认值 */
    ECO_INFO_NEW("[CanDispatcher] config not found, using hardcoded defaults");

    /* 校准/透传默认值 (对齐 motor_tool daemon) */
    m_calib_timeout_ms = 10000;
    m_sensor_period_ms = 1;
    m_sensor_period_div = 1;
    m_sensor_bus_format = 3;  /* CANFD BRS */
    m_sensor_mode = 3;
    m_sensor_force_module = 1;

    m_motors_json = nlohmann::json::array();
    for (uint8_t id = 1; id <= (uint8_t)m_motor_count; id++) {
        nlohmann::json m;
        m["id"] = id;
        m["heartbeat_ms"]      = 0;
        m["profile_accel"]     = 500;
        m["profile_decel"]     = 500;
        m["profile_velocity"]  = 20;
        m["disable_watchdog"]  = true;
        m["auto_enable"]       = false;
        m["bootup_timeout_ms"] = 5000;
        m["tpdo_sync_count"]   = 1;
        m_motors_json.push_back(m);
    }

    ECO_INFO_NEW("[CanDispatcher] registered {} motors (defaults)", m_motor_count);
    return true;
}

}  /* namespace stark_periph_manager_node */
