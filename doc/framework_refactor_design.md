# stark_periph_manager_node 框架优化设计

> 分支: feature/device-framework | 日期: 2026-08-24 | 维护: zhiqiang.yang
> 基线: prod 版（巨蟹电机控制, 已 1 个月压测稳定）| 参考: stark_periph_manager_node_v2 定稿设计
> 状态: **待确认**（确认后按本文档分步实现, 每步实现后审查再继续）

---

## 0. 背景

prod 版已满足控制功能与实时性要求，但存在三类问题，量产前需要解决：

1. **冗余**：`CanDispatcher` 上帝类、`motor_rt_worker` 大杂烩、左右字段重复展开、SHM 写死 `motor[2]`
2. **扩展性差**：加设备/换设备/换产品都要改结构体 + 重编译 + 算法同步改
3. **量产缺口**：安全兜底（SafetyCheck）已移除、配置无 fail-safe、SHM 无版本协商

## 1. 设计目标（可验收）

| # | 目标 | 验收标准 |
|---|------|---------|
| G1 | 加电机/加传感器 = 改配置或实现接口 | 加一个设备不改 framework 层代码 |
| G2 | 换电机厂商/换传感器接口 = 只改 L0 驱动 | 上层接口与数据契约零改动 |
| G3 | 实时性不劣化 | RT 路径无锁/无分配/无阻塞, 与 prod 持平 |
| G4 | 硬件级安全兜底 | 算法挂/超时/发异常命令, periph 能脱使能 |
| G5 | 配置 fail-safe | 配错/缺字段/类型错 → 回退默认值 + 告警, 不崩溃 |
| G6 | 框架可复用 | L1/L2/L3 骨架跨产品直接搬, L0/L4 按产品写 |

## 2. 目标架构（分层）

```
L4 控制层    SafetyController(安全兜底, RT域, 独立于算法)
L3 运行时层  RtScheduler(1KHz单线程) + 状态机(table) + LifecycleManager
L2 设备管理层 DeviceManager(静态注册表) + 工厂(配置驱动)
L1 设备抽象层 Device / IMotorDevice / ISensorDevice (稳定接口)
L0 硬件驱动层 MotorCanfd(包装motor_hal) / ImuIcm45608 / FootPressure
```

**分层铁律**：上层只依赖下层的接口，不依赖实现。换硬件=只动 L0；加设备=只动 L0+配置；换控制算法=只动 L4+算法端。

## 3. L1 设备抽象接口（具体签名）

```cpp
namespace stark {

struct MotorFeedback {
    int32_t  position, velocity, current;   // 位置/速度/Iq电流
    int16_t  temperature;
    uint16_t status;                        // 状态/故障码
    uint64_t timestamp_us;
};
struct MotorCommand {
    int32_t target;                         // 目标值(含义由 mode 决定)
    uint8_t  mode;                          // 控制模式
};

class Device {                               // 通用标识 + 生命周期
public:
    virtual ~Device() = default;
    virtual const char* name() const = 0;    // "motor_hip_right"
    virtual const char* type() const = 0;    // "motor"/"imu"
    virtual bool initialize(const void* config) = 0;  // 非RT, 可阻塞/分配
    virtual bool start() = 0;                          // 非RT
    virtual void stop() = 0;                           // 非RT/RT(急停可能RT调)
};

class IMotorDevice : public Device {
public:
    virtual int  motorCount() const = 0;               // 一驱动可管多路
    virtual bool readFeedback(int idx, MotorFeedback& fb) = 0;   // RT, 无锁无阻塞无分配
    virtual bool writeCommand(int idx, const MotorCommand& cmd) = 0; // RT
    virtual bool enable(int idx) = 0;
    virtual bool disable(int idx) = 0;
    virtual bool clearFault(int idx) = 0;
};

class ISensorDevice : public Device {
public:
    virtual bool read(SensorData& out) = 0;            // RT
};

}  // namespace stark
```

**RT 铁律**：`readFeedback/writeCommand/read` 内禁止 malloc/拿锁/阻塞/系统调用；数据契约用固定偏移直读直写。

## 4. L2 设备管理（静态注册 + 配置驱动）

```cpp
#define REGISTER_DEVICE(driver, creator) \
    static const DeviceRegistrar _reg_##driver(#driver, creator)

class DeviceManager {
public:
    static DeviceManager& instance();
    void registerDriver(const char* driver, DeviceCreator creator);
    bool loadDevices(const DeviceConfigList& configs);  // 配置→按driver创建→注册
    bool startAll();                                     // 统一生命周期, 按序
    void stopAll();                                      // 逆序回滚
    IMotorDevice*  motor(const char* name);
    ISensorDevice* sensor(const char* name);
};
```

**关键决策**：
- **静态注册**（编译期），不用 dlopen —— 量产无热插拔，零动态加载失败，行为可预测
- 每个驱动 .cpp 里 `REGISTER_DEVICE("motor_canfd", creator)` 一次
- `loadDevices` 解析配置 → 按 `driver` 字段找到 creator → 实例化 → 注册

## 5. SHM 自描述 + 数据契约

**沿用现有并发机制**（双 buffer + mailbox 环形 + seq/ack），**只改布局组织**：

```
┌─ 固定头 ────────────────────────────────┐
│ magic | version | size | crc           │
│ motor_count | sensor_type_table[]       │  ← 启动读一次, 缓存偏移后直读
├─ 电机反馈区(双buffer, periph RT 写)──────┤
├─ 传感器区(每类型槽位双buffer, RT 写)─────┤
├─ 命令区 mailbox(环形SPSC, 算法写)────────┤
├─ 状态区(node_state/心跳/severity)────────┘
```

**核心改造**：从"写死 `motor[2]`"到"编译期上限 `MAX_MOTORS`(16) + 头部运行时 `motor_count`"。加电机=count+1+配置加设备，SHM 结构本身不改。

**版本协商**：所有进程启动读 version，与自身支持版本比对，不一致拒绝启动 + 明确报错（防新旧进程混跑读错偏移）。

## 6. L3 状态机 + 生命周期

6 状态：`INIT → IDLE → CALIBRATING → RUNNING → FAULT → SHUTDOWN`，table 驱动，非法转换拒绝。

`LifecycleManager`：统一 `initialize → start → [运行] → stop → cleanup`，异常逆序回滚。

**明确不做**：BT.CPP 行为树、流程编排器接口 —— 状态机已够，白背依赖不值。

## 7. L4 安全控制器（量产重点）

独立于算法、RT 域运行，算法挂/超时/发异常命令都要能兜底：

| 项 | 动作 |
|---|---|
| 过流/过温 | 限幅/降功率 |
| 位置偏差 | 偏差超限 → 脱使能 |
| 通信超时 | 算法心跳超时 → 脱使能 |
| 紧急停止 | 立即脱使能 |

> 注：prod 已移除 SafetyCheck（心跳脱使能），量产必须补回，但用"基于反馈帧的硬件级保护"实现，不复用算法心跳机制。

## 8. 迁移路径（分步，每步可独立验证）

| 步骤 | 内容 | 验证 |
|------|------|------|
| S1 | 搭 L1/L2 骨架，接 1 个电机跑通全链路 | 单电机使能/反馈/命令正常 |
| S2 | 逐个迁移设备（电机→IMU→足压）到新接口 | 各设备数据正常 |
| S3 | 迁移安全控制器 + 状态机 + 生命周期 | 故障注入能脱使能 |
| S4 | SHM 自描述改造 + 版本协商，算法层切新契约 | 新旧版本拒绝启动 |
| S5 | 清理 V1 旧代码 | 编译干净, 无残留 |

**原则**：每步结束都"能跑 + 能验证"，不停在不可用的中间态。

## 9. 量产化清单（最后补）

1. 配置 fail-safe 铁律（配错回退默认值 + 告警，不崩溃；量产不提供配置文件自动落默认值）
2. SHM 版本协商（拒绝启动 + 明确报错）
3. 降权运行 + 关调试口（JTAG/串口/SSH/adb 量产全关）+ 只读 rootfs
4. 关键参数硬件信任根加密

## 10. 明确裁剪（不做，避免过度设计）

| 项 | 理由 |
|---|---|
| BT.CPP 行为树 | 状态机已够，收益只在复杂流程编排场景 |
| dlopen 插件 | 量产无热插拔，静态注册更可预测 |
| 泛型 blob 传感器数据 | 用类型化槽位，类型安全 + 可审计 |
| 每设备独立 RT 线程 | 单线程统一调度更稳定可预测 |

## 11. 待你确认的决策点

1. 分支名 `feature/device-framework` 是否 OK
2. L1 接口命名（`IMotorDevice`/`ISensorDevice`/`Device`）是否沿用 v2 命名
3. 电机上限 `MAX_MOTORS=16` 还是其他值
4. S1 先接"左髋/右髋 2 电机"还是先接"1 个电机"跑通
5. 安全控制器是 S3 做还是提前到 S1（量产关键，我建议提前）
