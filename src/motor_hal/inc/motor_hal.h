/**
 * @file motor_hal.h
 * @brief 关节模组 HAL 层 — 面向应用开发者的公共 API
 *
 * ## 这是什么
 *
 *   motor_hal 是一个纯 C 的 CANopen CiA 402 关节电机控制库。
 *   你不需要懂 CANopen 协议细节，也不需要知道 COB-ID 或 SDO/PDO 帧格式。
 *   只要按下面几个步骤调用 API 就能控制电机。
 *
 * ## 最小示例 (复制即用)
 *
 * @code
 *   motor_hal_t *hal = motor_hal_create();
 *   motor_hal_init(hal, "can0", 1000000, 5000000);
 *
 *   motor_config_t cfg = {0};
 *   cfg.node_id = 1;
 *   cfg.disable_watchdog = true;
 *   cfg.auto_enable = true;
 *   cfg.bootup_timeout_ms = 5000;
 *   motor_hal_add_motor(hal, &cfg);
 *
 *   motor_hal_recv_start(hal);          // 先启动接收线程
 *   motor_hal_startup(hal, 1, 5000);    // 再启动电机
 *
 *   while (running) {
 *       motor_feedback_t fb;
 *       motor_hal_get_feedback(hal, 1, &fb);   // 读反馈 (非阻塞)
 *       motor_hal_set_position(hal, 1, 30.0f);  // 发控制 (非阻塞)
 *       usleep(5000);
 *   }
 *
 *   motor_hal_destroy(hal);
 * @endcode
 *
 * ## 线程模型
 *
 *   ┌────────────────────┐     ┌─────────────────────┐
 *   │  接收线程 (框架内部) │     │  控制线程 (你的代码)   │
 *   │                     │     │                     │
 *   │ 阻塞等 CAN 帧       │     │ motor_hal_get_      │
 *   │ 自动更新反馈缓存     │ ──,  │   feedback()  读缓存 │
 *   │ 自动触发回调        │     │ motor_hal_set_      │
 *   │                     │     │   position() 发PDO  │
 *   └────────────────────┘     └─────────────────────┘
 *
 *   控制线程 "不触网卡" — 不调 recv/poll/select，只读缓存 + 发 PDO。
 *   这也是它能在 PREEMPT_RT 内核下安全运行的原因。
 *
 * ## 启动时序 (重要)
 *
 *   motor_hal_init       ,  打开 CAN 接口
 *   motor_hal_add_motor   ,  注册电机节点
 *   motor_hal_recv_start  ,  启动接收线程 (必须在 startup 之前)
 *   motor_hal_startup     ,  上电 + 使能
 *   [控制循环]
 *   motor_hal_destroy     ,  自动脱使能 + 关 CAN
 */

#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include "motor_hal_types.h"
#include "canopen_frames.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * HAL 句柄 (不透明类型 — 只通过 API 操作, 不要直接访问内部字段)
 * ============================================================================ */

typedef struct motor_hal motor_hal_t;

/* ============================================================================
 * 1. 生命周期 — 创建 / 初始化 / 销毁
 * ============================================================================ */

/**
 * @brief 创建 HAL 实例
 *
 * 分配并初始化内部数据结构。返回的指针用于所有后续 API 调用。
 * 一个进程通常只需要一个 HAL 实例。
 *
 * @return 成功返回非 NULL 指针; 内存不足返回 NULL。
 *
 * @code
 * motor_hal_t *hal = motor_hal_create();
 * if (!hal) { perror("create"); exit(1); }
 * @endcode
 */
motor_hal_t* motor_hal_create(void);

/**
 * @brief 销毁 HAL 实例, 释放所有资源
 *
 * 执行以下清理步骤 (按顺序):
 *   1. 停止接收线程 (如果已启动)
 *   2. 对所有已注册电机发送 Disable Voltage 命令
 *   3. 销毁回调、反馈缓存、mutex
 *   4. 关闭 CAN 接口
 *   5. 释放 HAL 内存
 *
 * 调用后 hal 指针失效, 不可再用。
 *
 * @param hal HAL 实例
 */
void motor_hal_destroy(motor_hal_t *hal);

/**
 * @brief 打开并绑定 CANFD 接口
 *
 * 内部创建 SocketCAN RAW socket, 启用 CANFD, 绑定到指定接口。
 * 调用前需要先用 ip 命令配置 CAN 接口:
 *
 * @code
 * sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 * sudo ip link set can0 up
 * @endcode
 *
 * @param hal          HAL 实例
 * @param iface        CAN 接口名, 如 "can0"
 * @param arb_bitrate  仲裁段波特率 (bps), 标准 CANopen 用 1000000 (1M)
 * @param data_bitrate 数据段波特率 (bps), CANFD 高速段, 通常 5000000 (5M)
 * @return 0=成功; <0=失败 (检查 errno, 常见: 接口不存在/权限不足)
 */
int motor_hal_init(motor_hal_t *hal, const char *iface,
                   uint32_t arb_bitrate, uint32_t data_bitrate);

/* ============================================================================
 * 2. 电机管理 — 注册 / 移除
 * ============================================================================ */

/**
 * @brief 注册一个电机节点
 *
 * 每个物理电机对应一个节点, 通过 CAN 节点 ID 区分。
 * cfg 参数会被内部拷贝, 可以分配在栈上。
 * 最多注册 MOTOR_HAL_MAX_MOTORS (16) 个电机。
 *
 * @param hal HAL 实例
 * @param cfg 电机配置 (见 motor_config_t), 拷贝语义
 * @return 0=成功; -EEXIST=ID重复; -ENOSPC=数量超限
 *
 * @code
 * motor_config_t cfg = {0};
 * cfg.node_id           = 1;            // CAN 节点 1
 * cfg.heartbeat_ms      = 2000;         // 2s 心跳
 * cfg.profile_accel     = 5000;         // 加速度 5000 RPM/s
 * cfg.profile_decel     = 5000;
 * cfg.profile_velocity  = 20;           // 最大轨迹速度 20 RPM
 * cfg.disable_watchdog  = true;         // 推荐关闭看门狗
 * cfg.auto_enable       = true;         // startup 时自动使能
 * cfg.bootup_timeout_ms = 5000;         // 5s 启动超时
 * motor_hal_add_motor(hal, &cfg);
 * @endcode
 */
int motor_hal_add_motor(motor_hal_t *hal, const motor_config_t *cfg);

/**
 * @brief 移除已注册的电机
 *
 * 内部紧凑数组, 后续电机索引前移。
 * 不会自动脱使能 — 调用前建议先 motor_hal_disable。
 *
 * @param hal     HAL 实例
 * @param node_id 要移除的 CAN 节点 ID
 */
void motor_hal_remove_motor(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 3. 启动与停用 — 电机上电 / 使能 / 脱使能
 *   motor_hal_recv_start 必须在这类函数之前调用
 * ============================================================================ */

/**
 * @brief 完整启动流程 — 从 Bootup 等待到 Operation Enabled
 *
 * 自动完成以下步骤 (不需要你手动干预):
 *   1. 等待驱动板发送 Bootup 帧 (0x700+ID, data=0)
 *   2. 通过 SDO 配置心跳周期 (0x1017)
 *   3. 通过 SDO 关闭看门狗 (0x2650=1) — 如果 cfg.disable_watchdog=true
 *   4. 通过 SDO 读取固件版本 (0x100A) — 验证通信链路
 *   5. 走 DS402 状态机: Shutdown ,  SwitchOn ,  EnableOp
 *   6. 延时 120ms 等待抱闸释放
 *
 * @param hal        HAL 实例
 * @param node_id    电机 CAN 节点 ID
 * @param timeout_ms 总超时时间 (ms), 推荐 5000
 * @return 0=成功; <0=超时或 SDO 通信失败
 *
 * @note 必须先调用 motor_hal_recv_start(), 否则 SDO 响应无法接收
 */
int motor_hal_startup(motor_hal_t *hal, uint8_t node_id, int timeout_ms);

/**
 * @brief 仅等待 Bootup 帧, 不做使能
 *
 * 用于只想确认驱动板上电正常但暂不使能的场景。
 * 与 motor_hal_startup 不同的是: 跳过心跳配置/SDO验证/DS402使能。
 *
 * @return 0=收到 Bootup; -ETIMEDOUT=超时
 */
int motor_hal_wait_bootup(motor_hal_t *hal, uint8_t node_id, int timeout_ms);

/**
 * @brief 使能电机 — 走 DS402 状态机进入 Operation Enabled
 *
 * 分三步 (每步间隔 20ms):
 *   Shutdown (CW=0x06) ,  ReadyToSwitchOn
 *   SwitchOn  (CW=0x07) ,  SwitchedOn
 *   EnableOp  (CW=0x0F) ,  OperationEnabled ,  等 120ms 抱闸释放
 *
 * 只在电机处于 NOT_READY / SWITCH_ON_DISABLED / READY_TO_SWITCH_ON
 * 状态时才能调用。如果电机在 FAULT 状态, 先调用 motor_hal_fault_reset。
 *
 * @return 0=成功; <0=SDO 通信失败
 */
int motor_hal_enable(motor_hal_t *hal, uint8_t node_id);

/**
 * @brief 脱使能电机 — 发送 Shutdown 命令
 *
 * 电机从 OperationEnabled ,  ReadyToSwitchOn。
 * 运动中的电机会先减速停止再脱使能。
 * 脱使能后电机不再响应位置/速度控制命令。
 *
 * @return 0=成功; <0=SDO 通信失败
 */
int motor_hal_disable(motor_hal_t *hal, uint8_t node_id);

/**
 * @brief 故障复位 — 清除 FAULT 状态
 *
 * 发送控制字 CW_FAULT_RESET (0x80, bit7上升沿)。
 * 成功后电机回到 SWITCH_ON_DISABLED 状态,
 * 需要重新调用 motor_hal_enable 才能再次控制。
 *
 * 常见的可复位故障: 过流/堵转/限位触发
 * 不可复位故障 (需断电重启): 编码器故障/硬件异常
 *
 * @return 0=成功; <0=SDO 通信失败
 */
int motor_hal_fault_reset(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 4. 实时控制 — 通过 PDO 帧直接控制电机 (非阻塞, RT 安全)
 *
 *   这些函数不经过 SDO, 而是直接构造 PDO 帧写入 SocketCAN。
 *   延迟 = 一次 write() 系统调用 + CAN 控制器发送时间。
 *   可在 RT 控制线程中安全调用。
 * ============================================================================ */

/**
 * @brief 位置控制 — 设置电机目标角度
 *
 * 使用 Profile Position (PP) 模式。
 * 内部换算: 角度(°) ,  编码器 count 值 ,  PDO 帧。
 * 电机按 cfg.profile_accel/velocity 规划梯形速度曲线运动到目标。
 *
 * @param hal       HAL 实例
 * @param node_id   电机 CAN 节点 ID
 * @param angle_deg 目标角度 (度), 范围 -180.0 ~ 180.0
 * @return 0=成功; -ENOENT=电机不存在; -EAGAIN=电机未使能
 *
 * @code
 * motor_hal_set_position(hal, 1, 45.0f);   // 转到 45°
 * motor_hal_set_position(hal, 1, -30.0f);  // 转到 -30°
 * @endcode
 */
int motor_hal_set_position(motor_hal_t *hal, uint8_t node_id, float angle_deg);

/**
 * @brief 速度控制 — 设置电机目标转速
 *
 * 使用 Profile Velocity (PV) 模式。
 * 正数 = 正转方向, 负数 = 反转方向。
 * 内部通过 PDO 帧下发, 不经过 SDO。
 *
 * @param rpm_motor 目标转速 (RPM, 电机端), 符号决定方向
 * @return 0=成功; -ENOENT=电机不存在; -EAGAIN=未使能
 *
 * @code
 * motor_hal_set_velocity(hal, 1, 500.0f);   // 500 RPM 正转
 * motor_hal_set_velocity(hal, 2, -300.0f);  // 300 RPM 反转
 * @endcode
 */
int motor_hal_set_velocity(motor_hal_t *hal, uint8_t node_id, float rpm_motor);

/**
 * @brief 电流 (力矩) 控制 — 设置 Iq 轴目标电流
 *
 * 使用电流环模式。电流 ≈ 力矩 (在恒转矩区成正比)。
 * 典型用法: 力矩控制、零力示教、力控底层。
 *
 * @param current_ma 目标 Iq 电流 (mA)。正=正向力矩, 负=反向力矩。
 * @return 0=成功; -ENOENT=电机不存在
 *
 * @code
 * motor_hal_set_torque(hal, 1, 2000);   // 2A 力矩
 * motor_hal_set_torque(hal, 1, -500);   // -0.5A 反向
 * @endcode
 */
int motor_hal_set_torque(motor_hal_t *hal, uint8_t node_id, int16_t current_ma);

/**
 * @brief MIT 阻抗控制 — 力位混合控制
 *
 * 模仿弹簧-阻尼系统的行为:
 *   τ = kp × (θ_target - θ_actual) + kd × (θ'_target - θ'_actual) + τ_ff
 *
 * 适用场景: 协作机器人、柔顺交互、力控示教、步行机器人的摆动相。
 *
 * @param position 目标位置 (°)
 * @param velocity 目标速度
 * @param kp       位置刚度 (值越大越"硬")
 * @param kd       速度阻尼 (值越大阻力越大)
 * @param torque   前馈力矩
 *
 * @code
 * // 柔顺模式: 低刚度, 可以被人推动
 * motor_hal_mit_control(hal, 1, 0, 0, 0.3f, 0.05f, 0);
 *
 * // 刚性位置控制: 高刚度
 * motor_hal_mit_control(hal, 1, 30.0f, 0, 2.0f, 0.3f, 0);
 * @endcode
 */
int motor_hal_mit_control(motor_hal_t *hal, uint8_t node_id,
                          float position, float velocity,
                          float kp, float kd, float torque);

/**
 * @brief 从驱动器读取 MIT 快控缩放参数 (0x2542~0x2546)
 *
 * 必须在电机启动后调用, 建议在 motor_startup 完成后立即调用。
 * 读取成功后覆盖 motor_node_t 中的默认缩放值。
 *
 * @return 0=全部成功, 非0=部分或全部读取失败 (已保留默认值)
 */
int motor_hal_read_mit_scales(motor_hal_t *hal, uint8_t node_id);

/**
 * @brief 获取已读取的 MIT 缩放值
 *
 * @param idx 0=pmax, 1=vmax, 2=kpmax, 3=kdmax, 4=tmax
 * @return 缩放值 (float), 未读取时返回默认值
 */
float motor_hal_get_mit_scale(motor_hal_t *hal, uint8_t node_id, int idx);

/**
 * @brief MIT 编码: 物理量 → raw 值
 *
 * 按 KWS CANFD V2 协议公式:
 *   pos_raw  = round((pos_rad + pmax) / (2*pmax) × 65535)
 *   vel_raw  = round((vel_rads + vmax) / (2*vmax) × 4095)
 *   kp_raw   = round(kp / kpmax × 4095)
 *   kd_raw   = round(kd / kdmax × 4095)
 *   tq_raw   = round((tau_ff_nm + tmax) / (2*tmax) × 4095)
 *
 * 所有 raw 值均钳位到合法范围。
 */
void motor_hal_mit_encode_raw(const mit_scales_t *s,
                              float pos_rad, float vel_rads,
                              float kp, float kd, float tau_ff_nm,
                              uint16_t *pos_raw, uint16_t *vel_raw,
                              uint16_t *kp_raw, uint16_t *kd_raw,
                              uint16_t *tq_raw);

/**
 * @brief MIT 双电机多轴控制 (物理量 → 0x210 一帧同时发出)
 *
 * 从 motor_node 读取各自缩放参数, 分别编码后打包到一个 0x210 帧。
 * 任一电机不存在或未使能则跳过该电机。
 */
void motor_hal_mit_multi_ctrl_phys(motor_hal_t *hal,
    uint8_t id1, float pos1, float vel1, float kp1, float kd1, float tq1,
    uint8_t id2, float pos2, float vel2, float kp2, float kd2, float tq2);

/**
 * @brief 通用 PDO 控制 — 指定模式 + 裸参数
 *
 * 当标准接口 (set_position/set_velocity/set_torque) 不够灵活时使用。
 * 你直接指定控制模式和三个参数, 框架帮你组帧发送。
 *
 * @param mode        控制模式 (见 motor_mode_t)
 * @param target1     参数1 (含义取决于模式: PP=角度counts, PV=速度RPM, 电流=mA)
 * @param target2     参数2 (通常为加减速度或保留)
 * @param feedforward 前馈值
 * @return 0=成功; -ENOENT=不存在; -EAGAIN=未使能
 *
 * @code
 * // CSP 模式: 每周期给目标位置 (上位机插补)
 * int16_t counts = motor_deg_to_counts(45.0f);
 * motor_hal_ctrl_raw(hal, 1, MOTOR_MODE_CSP, counts, 0, 0);
 * @endcode
 */
int motor_hal_ctrl_raw(motor_hal_t *hal, uint8_t node_id,
                       motor_mode_t mode,
                       int16_t target1, uint16_t target2, int16_t feedforward);

/**
 * @brief 停止运动 — 发送目标位置=0 的 PDO 帧
 *
 * 电机立即开始减速到 0 位置。
 * 不脱使能, 之后仍可重新发送位置/速度命令。
 *
 * @param node_id 电机 ID
 * @return 0=成功
 */
int motor_hal_stop(motor_hal_t *hal, uint8_t node_id);

/**
 * @brief 抱闸控制 — 释放/吸合电机抱闸
 *
 * 通过 PDO data[0] bit6 控制。
 * release=true:  松开抱闸 (电机可自由转动)
 * release=false: 吸合抱闸 (电机制动锁死)
 *
 * @param release true=松开, false=吸合
 * @return 0=成功; -ENOENT=电机不存在; -EAGAIN=未使能
 */
int motor_hal_set_brake(motor_hal_t *hal, uint8_t node_id, bool release);

/**
 * @brief 急停 — 发送 Quick Stop 命令 (CW=0x02)
 *
 * 通过 SDO 写 Controlword 0x0002。
 * 电机立即以 profile_decel 减速到 0, 然后保持使能。
 * 与 motor_hal_stop 的区别: stop 改目标位置, quickstop 走 DS402 急停逻辑。
 *
 * @return 0=成功; <0=SDO 失败
 */
int motor_hal_quick_stop(motor_hal_t *hal, uint8_t node_id);

/* ============================================================================
 * 5a. PDO Byte0 — 实时控制字节
 *
 *   Byte0: [7]Enable [6]BUS_ON [5]ClearErr [4:1]Mode [0]Rsvd
 *   SDO 不碰 PDO, PDO 不碰 SDO。
 *
 *   bit5 清错建议流程 (由上层决策):
 *     1. motor_hal_get_feedback ,  fb.error_code
 *     2. 判断错误: 过温, 等降温; 过流, 先失能; 其他, 直接清除
 *     3. motor_hal_pdo_clear_fault ,  bit5 脉冲, 下一帧自动清0
 *
 *   推荐用法:
 *     startup ,  pdo_enable + pdo_set_mode ,  控制循环(不碰Byte0)
 *     急停    ,  pdo_estop
 *     恢复    ,  pdo_recover
 * ============================================================================ */

int motor_hal_pdo_enable(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_disable(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_bus_on(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_bus_off(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_clear_fault(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode);
int motor_hal_pdo_estop(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_recover(motor_hal_t *hal, uint8_t node_id);
int motor_hal_pdo_set_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t byte0);
int motor_hal_pdo_get_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t *byte0);

/**
 * @brief 消费 pdo_byte0 — 读当前值并自动处理 clr_err 脉冲
 *
 * 与 motor_hal_pdo_get_byte0 的区别:
 *   本函数在读的同时自动清除 bit5 (如果 clr_err_pending=true),
 *   保证 clr_err 脉冲的单帧语义。
 *
 *   推荐控制路径使用此函数代替 get_byte0。
 */
int motor_hal_pdo_consume_byte0(motor_hal_t *hal, uint8_t node_id, uint8_t *byte0);

/* ============================================================================
 * 5. 模式与参数配置 — 通过 SDO 读写电机参数 (同步阻塞, 50~200ms)
 *
 *   这类函数通过 SDO 协议与驱动板通信。
 *   调用会阻塞等待 SDO 响应 (内部通过条件变量 + 接收线程)。
 *   不要在实时控制循环中调用 — 只在初始化或低频率调参时使用。
 * ============================================================================ */

/** @brief 切换控制模式 (0x6060)。mode 取值见 motor_mode_t 枚举。 */
int motor_hal_set_mode(motor_hal_t *hal, uint8_t node_id, motor_mode_t mode);

/** @brief 设置加减速度 (0x6083/0x6084), 单位 RPM/s, 电机端。 */
int motor_hal_set_accel_decel(motor_hal_t *hal, uint8_t node_id,
                              uint16_t accel, uint16_t decel);

/** @brief 设置位置模式下的最大轨迹速度 (0x6081), 单位 RPM, 输出端。 */
int motor_hal_set_profile_velocity(motor_hal_t *hal, uint8_t node_id, uint16_t rpm_out);

/** @brief 写入全部 PID 参数 (电流环P/I, 速度环P/I, 位置环P/I)。 */
int motor_hal_set_pid(motor_hal_t *hal, uint8_t node_id, const motor_pid_t *pid);

/** @brief 当前参数保存到 Flash, 掉电不丢失。 */
int motor_hal_save_flash(motor_hal_t *hal, uint8_t node_id);

/** @brief 触发零位标定 — 将当前位置记为编码器零点。 */
int motor_hal_set_zero(motor_hal_t *hal, uint8_t node_id);

/** @brief 设置软限位, 单位 °。 */
int motor_hal_set_limits(motor_hal_t *hal, uint8_t node_id, float pos_deg, float neg_deg);

/** @brief 关闭看门狗 (写 0x2650=1)。推荐在 motor_config_t 中设置, 无需手动调用。 */
int motor_hal_disable_watchdog(motor_hal_t *hal, uint8_t node_id);

/** @brief 位置环控制 — 启动/停止绝对位置运动 (SDO写 0x6040) */
int motor_hal_set_pos_ctrl(motor_hal_t *hal, uint8_t node_id, bool start);

/** @brief 设置位置环目标位置 (SDO写 0x607A), 单位编码器count */
int motor_hal_set_pos_target(motor_hal_t *hal, uint8_t node_id, int32_t target_counts);

/** @brief 设置速度环目标速度 (SDO写 0x60FF), 单位RPM */
int motor_hal_set_speed_target(motor_hal_t *hal, uint8_t node_id, int32_t target_rpm);

/* ============================================================================
 * 6. 状态查询 — 通过 SDO 同步读取 (阻塞 50~200ms)
 * ============================================================================ */

/**
 * @brief 查询 DS402 状态
 * @return motor_state_t 枚举值: NOT_READY / SWITCH_ON_DISABLED /
 *         READY_TO_SWITCH_ON / SWITCHED_ON / OPERATION_ENABLED / FAULT
 */
motor_state_t motor_hal_get_state(motor_hal_t *hal, uint8_t node_id);

/** @brief 读取状态字原始值 (0x6041), uint16。 */
uint16_t motor_hal_get_statusword(motor_hal_t *hal, uint8_t node_id);

/** @brief 读取当前编码器位置 (0x6064), 返回 int32 编码器 count 值。 */
int32_t motor_hal_get_position(motor_hal_t *hal, uint8_t node_id);

/** @brief 读取当前速度 (0x606C), 通过 SDO。用 motor_hal_get_feedback 读缓存更快。 */
int32_t motor_hal_get_velocity(motor_hal_t *hal, uint8_t node_id);

/** @brief 读取当前电流 (0x6078), 通过 SDO。 */
int32_t motor_hal_get_current(motor_hal_t *hal, uint8_t node_id);

/** @brief 读取全部 PID 参数到 motor_pid_t 结构。 */
int motor_hal_read_pid(motor_hal_t *hal, uint8_t node_id, motor_pid_t *pid);

/**
 * @brief 通用 SDO 读 — 读取任意对象字典索引
 *
 * 用于读取没有专门 API 的对象字典条目 (如固件版本 0x100A、厂商名称 0x1008)。
 *
 * @param index   对象字典索引 (如 OD_FIRMWARE_VER = 0x100A)
 * @param subidx  子索引 (通常 0x00)
 * @param value   [out] 返回的 uint32 值
 * @return 0=成功; <0=SDO 超时或错误
 *
 * @code
 * uint32_t fw_ver;
 * motor_hal_sdo_read_u32(hal, 1, OD_FIRMWARE_VER, 0x00, &fw_ver);
 * printf("Firmware: 0x%08X\n", fw_ver);
 * @endcode
 */
int motor_hal_sdo_read_u32(motor_hal_t *hal, uint8_t node_id,
                           uint16_t index, uint8_t subidx, uint32_t *value);

/**
 * @brief 通用 SDO 写 — 写入任意对象字典索引
 *
 * @param index   对象字典索引
 * @param subidx  子索引
 * @param value   写入值 (uint32)
 * @param size    数据字节数 (1/2/4)
 * @return 0=成功; <0=SDO 超时或错误
 */
int motor_hal_sdo_write(motor_hal_t *hal, uint8_t node_id,
                        uint16_t index, uint8_t subidx,
                        uint32_t value, uint8_t size);

/* ============================================================================
 * 7. 反馈缓存 — 非阻塞读取最近一次反馈数据 (PI mutex, RT 安全)
 * ============================================================================ */

/**
 * @brief 获取最近一次反馈 (线程安全拷贝)
 *
 * 不通过 SDO — 直接从接收线程维护的反馈缓存中拷贝。
 * 延迟 = 一次 memcpy (几十 ns) + 一次 PI mutex 加锁。
 * 可在 RT 控制线程中高频调用 (1kHz 以上)。
 *
 * 数据来源: 驱动板周期性上报的反馈帧 (COB=0x300+ID, 12 字节)。
 * 接收线程在后台自动接收和更新。
 *
 * @param fb [out] 拷贝到这里, 由调用者分配
 * @return 0=成功; -EINVAL=参数无效; -ENOENT=节点不存在
 *
 * @code
 * motor_feedback_t fb;
 * motor_hal_get_feedback(hal, 1, &fb);
 * float angle = motor_counts_to_deg(fb.position);
 * if (fb.status_byte & 0x20) printf("Error: 0x%04X\n", fb.error_code);
 * @endcode
 */
int motor_hal_get_feedback(motor_hal_t *hal, uint8_t node_id, motor_feedback_t *fb);

/* ============================================================================
 * 8. 接收线程 — 独立线程阻塞接收 CAN 帧, 自动分发
 * ============================================================================ */

/**
 * @brief 启动接收线程
 *
 * 创建一个独立线程, 内部循环:
 *   while (running) {
 *       can_driver_recv(hal->drv, &f, 100);  // 阻塞等帧
 *       _dispatch_frame(hal, &f);              // 按 COB-ID 分发
 *   }
 *
 * 分发规则:
 *   0x580 ,  SDO 响应 ,  入队列 ,  condvar ,  sdo_client 等待
 *   0x300 ,  反馈帧  ,  写缓存 (PI mutex) + 触发反馈回调
 *   0x700 ,  Bootup/Heartbeat ,  设置 bootup_received 标志
 *   0x080 ,  EMCY 紧急报文 ,  触发错误回调
 *
 * @note 必须在 motor_hal_startup 之前调用 (否则 SDO 无人接收)
 * @note 启动后不要再调 motor_hal_poll (会检测到并直接 return)
 * @return 0=成功; -ENODEV=CAN未初始化; -EBUSY=已启动
 */
int motor_hal_recv_start(motor_hal_t *hal);

/**
 * @brief 设置接收线程实时优先级
 *
 * 必须在 motor_hal_recv_start 之前调用。
 * 接收线程启动后内部调用 pthread_setschedparam 设置 SCHED_FIFO + 指定优先级。
 * enable=false 时线程保持默认 SCHED_OTHER。
 *
 * @param hal      HAL 实例
 * @param enable   是否启用实时调度
 * @param priority SCHED_FIFO 优先级 (1-99), enable=false 时忽略
 */
void motor_hal_recv_set_rt(motor_hal_t *hal, bool enable, int priority);

/**
 * @brief 设置接收线程 CPU 亲和性 (绑核)
 *
 * 必须在 motor_hal_recv_start 之前调用。
 * 建议绑到与 RT 线程相同的核, 共享 cache 降低反馈读取延迟。
 *
 * @param hal HAL 实例
 * @param cpu 绑定的 CPU 核编号 (-1=不绑, 跟随系统调度)
 */
void motor_hal_recv_set_affinity(motor_hal_t *hal, int cpu);

/**
 * @brief 停止接收线程
 *
 * 阻塞等待线程退出 (pthread_join)。
 * 通常在 motor_hal_destroy 中自动调用, 一般不需要手动调用。
 *
 * @return 0=成功; -EINVAL=未启动
 */
int motor_hal_recv_stop(motor_hal_t *hal);

/** @brief 查询接收线程是否在运行 */
bool motor_hal_recv_is_running(motor_hal_t *hal);

/* ============================================================================
 * 8b. 外设传感器透传 — 通过 0x680+ID 周期性上报
 * ============================================================================ */

/**
 * @brief 配置外设传感器透传
 *
 * 写入 SDO 0x5503:04, 电机按配置周期自动发送 8 字节传感器数据。
 * 数据帧 COB-ID = 0x680 + node_id, DLC=8, 小端 bit-packed。
 *
 * @param node_id     电机 CAN 节点 ID
 * @param period_div  周期分频 N (1tick=0.25ms, V1.2基准; 0=关闭)
 *                    例: 4=1ms/1KHz, 16=4ms/250Hz
 * @param bus_format  总线格式: 0=Classic CAN, 3=CANFD BRS (mode=3时忽略)
 * @return 0=成功; <0=SDO 失败
 */
int motor_hal_sensor_config(motor_hal_t *hal, uint8_t node_id,
                            uint16_t period_div, uint8_t bus_format);

/**
 * @brief 完整透传配置 (period_div + bus_format + mode + force_module)
 * @param mode          0=兼容 1=运行反馈 2=全部 3=聚合(V1.2 新增)
 * @param force_module  0=CAN(0x680力矩) 1=SPI(0x6B0力矩, 默认)
 * 配置字: period_div[15:0]|bus_format[17:16]|mode[19:18]|force_module[21:20]
 */
int motor_hal_sensor_config_ex(motor_hal_t *hal, uint8_t node_id,
                               uint16_t period_div, uint8_t bus_format,
                               uint8_t mode, uint8_t force_module);

/** @brief 停止传感器透传 (等价 sensor_config(node, 0, 0)) */
int motor_hal_sensor_stop(motor_hal_t *hal, uint8_t node_id);

/** @brief 获取最近一次传感器数据 (线程安全拷贝) */
int motor_hal_get_sensor(motor_hal_t *hal, uint8_t node_id, motor_sensor_t *s);

/**
 * @brief 注册传感器回调 — 每收到一帧透传数据就触发
 * @note 回调在接收线程上下文中执行, 不要做 SDO 或耗时操作
 */
void motor_hal_set_sensor_cb(motor_hal_t *hal, uint8_t node_id,
                             motor_sensor_cb_t cb, void *ctx);

/* ============================================================================
 * 9. 回调系统 — 事件驱动的异步通知
 * ============================================================================ */

/**
 * @brief 注册反馈回调 — 每收到一帧反馈就触发
 *
 * 回调在 **接收线程上下文** 中执行, 因此:
 *    可以做: 更新全局变量, 填充环形缓冲区, notify 控制线程
 *    不要做: SDO 操作 (会死锁), printf (不在 RT 线程), 长时间计算
 *
 * @param node_id 要监听的电机的 CAN 节点 ID
 * @param cb      回调函数, 传 NULL 取消
 * @param ctx     用户上下文指针, 会传回回调的第三个参数
 */
void motor_hal_set_feedback_cb(motor_hal_t *hal, uint8_t node_id,
                               motor_feedback_cb_t cb, void *ctx);

/**
 * @brief 注册错误回调 — EMCY 帧或反馈帧 bit5=1 时触发
 *
 * 触发源有两种:
 *   1. 驱动板主动发送的 EMCY 紧急报文 (COB=0x080+ID)
 *   2. 反馈帧 status_byte.bit5=1 (电机内部错误)
 *
 * 回调参数: node_id (哪个电机), error_code (错误码, 见 motor_hal_types.h), ctx
 */
void motor_hal_set_error_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_error_cb_t cb, void *ctx);

/**
 * @brief 注册状态变更回调 — enable/disable/fault 时触发
 *
 * 回调参数: node_id, old_state, new_state, ctx
 * 用于上层状态机同步 (如: 检测到 FAULT 后触发安全逻辑)。
 */
void motor_hal_set_state_cb(motor_hal_t *hal, uint8_t node_id,
                            motor_state_cb_t cb, void *ctx);

/* ============================================================================
 * 10. 轮询 — 向前兼容 (接收线程启动后无需使用)
 * ============================================================================ */

/**
 * @brief [向前兼容] 手动轮询一帧 CAN 数据
 *
 * 如果已调用 motor_hal_recv_start(), 此函数直接返回 (不执行任何操作)。
 * 新版推荐用 motor_hal_recv_start 代替手动 poll。
 *
 * 如果接收线程未启动, 此函数行为与 v1 相同:
 *   阻塞等待一帧 (timeout_ms), 然后按 COB-ID 分发。
 *
 * @param timeout_ms 等帧超时: 0=非阻塞, >0=最多等这些毫秒
 */
void motor_hal_poll(motor_hal_t *hal, int timeout_ms);

/**
 * @brief 处理 auto_enable 触发的待启动电机 (主线程定期调用)
 *
 * recv 线程收到 bootup 后只设 pending_startup 标志,
 * 主线程调用此函数执行 motor_startup_full (含 SDO 操作).
 * 必须在 motor_hal_recv_start 之后调用.
 *
 * @param hal HAL 实例
 * @return 本次启动的电机数量
 */
int motor_hal_process_pending_startups(motor_hal_t *hal);

/* ============================================================================
 * 11. 全局控制 — 影响总线上所有节点
 * ============================================================================ */

/**
 * @brief 发送 NMT 广播命令
 * @param cmd NMT 命令码: NMT_CMD_START / NMT_CMD_STOP /
 *            NMT_CMD_PRE_OP / NMT_CMD_RESET_NODE / NMT_CMD_RESET_COMM
 */
void motor_hal_nmt_broadcast(motor_hal_t *hal, uint8_t cmd);

/**
 * @brief 发送 SYNC 同步帧
 *
 * SYNC 帧仅 0 字节, 开销极低。
 * 用途: 触发同步 PDO 传输, 或作为看门狗喂狗信号 (100ms 发一次)。
 */
void motor_hal_sync(motor_hal_t *hal);

/**
 * @brief 启动 SYNC 定时器 — 按 period_us 周期发送 SYNC 帧
 *
 * 创建一个高优先级线程, 以精确周期发送 SYNC (0 字节, 极低开销)。
 * 配合从站 TPDO 的同步传输模式 (trans_type=1~240),
 * 可实现确定性的 5ms/1ms 反馈周期。
 *
 * 典型用法:
 *   motor_hal_sync_start(hal, 5000);  // 5ms = 200Hz
 *
 * @param period_us  SYNC 周期 (微秒), 推荐 1000~100000
 * @return 0=成功; -EBUSY=已在运行; -ENODEV=CAN未初始化
 */
int motor_hal_sync_start(motor_hal_t *hal, uint32_t period_us);

/** @brief 停止 SYNC 定时器 */
int motor_hal_sync_stop(motor_hal_t *hal);

/** @brief 设置 SYNC 线程实时参数: SCHED_FIFO + 优先级 (启动前调用) */
void motor_hal_sync_set_rt(motor_hal_t *hal, bool enable, int priority);

/** @brief 设置 SYNC 线程绑核 (-1=不绑) */
void motor_hal_sync_set_affinity(motor_hal_t *hal, int cpu);

/** @brief 查询 SYNC 定时器是否在运行 */
bool motor_hal_sync_is_running(motor_hal_t *hal);

/**
 * @brief 通用 PDO 映射 — 自定义从站 RPDO/TPDO 映射表
 *
 * 映射时序 (CiA 301 标准):
 *   1. 停用 PDO (写 comm sub0x01 bit31=1)
 *   2. 清空映射 (写 map sub0x00 = 0)
 *   3. 写入映射条目 (写 map sub0x01, sub0x02, ...)
 *   4. 设置映射数量 (写 map sub0x00 = N)
 *   5. 设置传输类型 (写 comm sub0x02)
 *   6. 启用 PDO (写 comm sub0x01 = COB-ID)
 *
 * @param node_id    电机 CAN 节点 ID
 * @param entries    映射条目数组 (用户自由选择 OD 索引/子索引/位宽)
 * @param count      映射条目数量 (1~8)
 * @param pdo_idx    PDO 索引: 0=RPDO1/TDO1, 1=RPDO2/TPDO2
 * @param type       PDO 方向: PDO_TYPE_RPDO 或 PDO_TYPE_TPDO
 * @param cob_id     COB-ID (如 0x200+node 或 0x180+node)
 * @param trans_type 传输类型: 1~240=同步周期, 254/255=异步, 0=同步非周期
 * @return 0=成功; <0=SDO 失败
 *
 * @code
 *   pdo_map_entry_cfg_t tpdo_entries[] = {
 *       {0x6041, 0x00, 16},  // Statusword
 *       {0x6064, 0x00, 32},  // Position
 *       {0x606C, 0x00, 32},  // Velocity
 *   };
 *   motor_hal_pdo_map(hal, 1, tpdo_entries, 3, 0, PDO_TYPE_TPDO,
 *                     0x181, 1);  // 每个 SYNC 上报一次
 * @endcode
 */
int motor_hal_pdo_map(motor_hal_t *hal, uint8_t node_id,
                      const pdo_map_entry_cfg_t *entries, uint8_t count,
                      uint8_t pdo_idx, pdo_type_t type,
                      uint32_t cob_id, uint8_t trans_type);

/**
 * @brief 配置从站 TPDO1 为同步周期上报 (便捷封装)
 *
 * 调用 motor_hal_pdo_map 实现, 映射固定为:
 *   Statusword(16b) + Position(32b) + Velocity(32b) + Current(16b)
 *
 * @param node_id    电机 CAN 节点 ID
 * @param sync_count 每 N 个 SYNC 发一次 (1~240)
 * @return 0=成功; <0=SDO 失败
 */
int motor_hal_tpdo_config(motor_hal_t *hal, uint8_t node_id, uint8_t sync_count);

/**
 * @brief 发送标准 RPDO 控制帧 (用户自定义映射后使用)
 *
 * 标准 RPDO 需要通过 SDO 先配置映射 (motor_hal_pdo_map + PDO_TYPE_RPDO),
 * 然后调用此函数按映射格式发送控制帧。
 *
 * COB-ID 固定为 0x200 + node_id (CiA 预定义 RPDO1)。
 *
 * @param node_id  电机 CAN 节点 ID
 * @param data     数据字节 (小端, 按映射顺序排列)
 * @param dlc      数据长度 (映射条目总字节数)
 * @return 0=成功
 *
 * @code
 *   // 映射: Controlword(16b) + TargetPosition(32b) = 6 bytes
 *   uint8_t data[] = {0x0F, 0x00,  // CW=EnableOp
 *                      0x00, 0x40, 0x00, 0x00};  // Pos=16384cnt(90°)
 *   motor_hal_rpdo_send(hal, 1, data, 6);
 * @endcode
 */
int motor_hal_rpdo_send(motor_hal_t *hal, uint8_t node_id,
                        const uint8_t *data, uint8_t dlc);

/**
 * @brief 注册标准 TPDO 原始帧回调
 *
 * 当自定义 TPDO 映射后, 接收线程收到 TPDO 帧 (COB=0x180+node) 时,
 * 如果注册了此回调, 则调用回调并跳过默认硬编码解析。
 * 回调中用户根据自己的映射配置自行解析数据。
 *
 * @param cb  回调函数, 传 NULL 取消注册
 *
 * @code
 * void my_tpdo(uint8_t id, const canfd_frame_t *f, void *ctx) {
 *     int16_t sw  = f->data[0] | (f->data[1] << 8);
 *     int32_t pos = f->data[2] | (f->data[3] << 8) | ...;
 *     ...
 * }
 * motor_hal_set_tpdo_cb(hal, 1, my_tpdo, NULL);
 * @endcode
 */
void motor_hal_set_tpdo_cb(motor_hal_t *hal, uint8_t node_id,
                           motor_tpdo_raw_cb_t cb, void *ctx);

/**
 * @brief 多轴广播控制 — 一帧 CANFD 控制最多 8 个电机
 *
 * 利用 CANFD 的 64 字节数据段, 一帧同时下发 8 条 PDO 命令。
 * 相比逐个调用 set_position/set_velocity, 延迟一致性好很多。
 *
 * @param cmds  命令数组 (每个电机一个 multi_axis_cmd_t)
 * @param count 命令数量 (1~8)
 *
 * @code
 * multi_axis_cmd_t cmds[2] = {
 *     { .node_id=1, .mode=MOTOR_MODE_CSP, .enable=true, .target1=pos1 },
 *     { .node_id=2, .mode=MOTOR_MODE_CSP, .enable=true, .target1=pos2 },
 * };
 * motor_hal_multi_ctrl(hal, cmds, 2);  // 一帧控制双关节
 * @endcode
 */
void motor_hal_multi_ctrl(motor_hal_t *hal, const multi_axis_cmd_t *cmds, uint8_t count);

/** @brief MIT 多轴广播 (0x210, 64Byte CAN FD), 一帧同时控制最多6个电机 */
void motor_hal_mit_multi_ctrl(motor_hal_t *hal, const multi_mit_cmd_t *cmds, uint8_t count);

/* ============================================================================
 * 11b. 专用 SDO 控制接口 — 对应巨蟹协议 4.3 章每条指令
 *
 *   这些是对 motor_hal_sdo_write / motor_hal_sdo_read_u32 的语义封装。
 *   封装的好处: 类型安全、参数语义明确、不需要记 OD Index。
 * ============================================================================ */

/** @brief NMT 发送 — 向指定节点发送 NMT 命令 (Start/Stop/PreOp/Reset) */
int motor_hal_nmt_send(motor_hal_t *hal, uint8_t node_id, uint8_t cmd);

/** @brief 读取故障码 (OD 0x603F), uint16 */
int motor_hal_get_fault_code(motor_hal_t *hal, uint8_t node_id, uint16_t *code);

/** @brief 读取 MOS 温度 (OD 0x2662), 单位 0.1°C */
int motor_hal_get_mos_temp(motor_hal_t *hal, uint8_t node_id, int32_t *temp);

/** @brief 读取电机线圈温度 (OD 0x2663), 单位 0.1°C */
int motor_hal_get_motor_temp(motor_hal_t *hal, uint8_t node_id, int32_t *temp);

/** @brief SDO telemetry: start background polling thread (~5ms per motor).
 *  Polls temperature (0x2663) and position (0x6064) for all registered motors. */
int motor_hal_sdo_telemetry_start(motor_hal_t *hal);
/** @brief SDO telemetry: stop background polling thread. */
int motor_hal_sdo_telemetry_stop(motor_hal_t *hal);
/** @brief Get cached SDO temperature, 0.1°C. Returns -EAGAIN if not polled yet. */
int motor_hal_get_sdo_temperature(motor_hal_t *hal, uint8_t node_id, int32_t *temp);
/** @brief Get cached SDO position, counts. */
int motor_hal_get_sdo_position(motor_hal_t *hal, uint8_t node_id, int32_t *pos);

/** @brief 读取最大电流限制 (OD 0x2538), 单位 mA */
int motor_hal_get_max_current(motor_hal_t *hal, uint8_t node_id, uint32_t *ma);

/** @brief 设置最大电流限制 (OD 0x2538), 单位 mA */
int motor_hal_set_max_current(motor_hal_t *hal, uint8_t node_id, uint32_t ma);

/** @brief 设置心跳周期 (OD 0x1017), 单位 ms (掉电恢复默认) */
int motor_hal_set_heartbeat(motor_hal_t *hal, uint8_t node_id, uint32_t ms);

/** @brief 修改电机 CAN 节点 ID (OD 0x2530), 范围 1~127, 重启生效 */
int motor_hal_set_node_id(motor_hal_t *hal, uint8_t node_id, uint8_t new_id);

/** @brief 设置 CANFD 数据段波特率 (OD 0x2540), 重启生效
 *  @param baud  1=5M, 2=4M, 3=2M, 4=1M */
int motor_hal_set_canfd_baud(motor_hal_t *hal, uint8_t node_id, uint8_t baud);

/* ============================================================================
 * 13. LED 灯控制 (OD 0x5503:06, SDO 读写, 非 RT)
 * ============================================================================ */

/** @brief 设置电机 RGB LED 灯效.
 *  control 字节: bit[7:4]=LED1~4使能掩码, bit[3:0]=led_mode_t.
 *  值 = (enable_mask|mode) | (R<<8) | (G<<16) | (B<<24)
 *  @param cfg  enable_mask + mode + R/G/B, 全零=全部熄灭 */
int motor_hal_led_set(motor_hal_t *hal, uint8_t node_id, const led_config_t *cfg);

/** @brief 读取电机当前 LED 配置 (OD 0x5503:06 SDO 回读) */
int motor_hal_led_get(motor_hal_t *hal, uint8_t node_id, led_config_t *cfg);

/* ============================================================================
 * 14. V1.1 新增对象字典 SDO 接口 (非 RT)
 * ============================================================================ */

/** @brief 读力矩传感器当前值 (OD 0x6077, RO, S16, 0.01N.m) */
int motor_hal_get_torque_sensor(motor_hal_t *hal, uint8_t node_id, int16_t *torque_001nm);

/** @brief 读母线电流 (OD 0x2661, RO, INT32, mA) */
int motor_hal_get_bus_current(motor_hal_t *hal, uint8_t node_id, int32_t *bus_ma);

/** @brief Flash 保存参数 (OD 0x1010:01, 写入任意值触发保存) */
int motor_hal_store_params(motor_hal_t *hal, uint8_t node_id);

/** @brief MIT 缩放迁移: 写 0x2546=20(Tmax) 并保存到 Flash, 适配 V2 出厂默认值 */
int motor_hal_mit_migrate_scales(motor_hal_t *hal, uint8_t node_id);

/** @brief 扭矩传感器零漂标定 (旧协议: OD 0x2531:00 写入值2, 驱动板自行归零) */
int motor_hal_torque_zero_calib(motor_hal_t *hal, uint8_t node_id);

/**
 * @brief 力矩传感器当前力矩标定 (新协议: OD 0x2531:00)
 *
 * pack: opcode=2 | (torque_mNm << 8)  24位有符号补码
 * 驱动板采集1000点样本, 计算 zero_drift = avg_raw - expected, 保存到 Flash
 *
 * @param torque_mNm  理论关节力矩, mNm, 范围 ±100000 (±100 Nm)
 * @return 0=成功, <0=失败
 */
int motor_hal_torque_calib(motor_hal_t *hal, uint8_t node_id, int32_t torque_mNm);

/* ============================================================================
 * 12. 工具函数 — 单位换算 (inline, 零开销)
 * ============================================================================ */

/** 编码器 count ,  角度 (°)。ENCODER_LOAD_RES=65536 对应 360°。 */
static inline float motor_counts_to_deg(int16_t counts) {
    return (float)counts * 360.0f / (float)ENCODER_LOAD_RES;
}

/** 角度 (°) ,  编码器 count。超出 -180~180 会被钳位。 */
static inline int16_t motor_deg_to_counts(float degrees) {
    return (int16_t)(degrees * ENCODER_LOAD_RES / 360.0f);
}

/** 温度 raw ,  °C (原始值 × 0.1) */
static inline float motor_temp_to_c(int16_t raw) {
    return raw * 0.1f;
}

/** 电流 mA ,  A */
static inline float motor_ma_to_a(int16_t ma) {
    return ma * 0.001f;
}

/* ============================================================================
 * 13. 诊断函数 — CANopen 错误码 / 状态码查表
 * ============================================================================ */

/**
 * @brief SDO Abort Code ,  可读字符串 (CiA 301, Table 52)
 * @return 错误描述, 未找到返回 NULL
 */
const char* motor_utils_sdo_abort_str(uint32_t abort_code);

/**
 * @brief NMT 状态码 ,  可读字符串
 * @return 状态描述, 未找到返回 NULL
 */
const char* motor_utils_nmt_state_str(uint8_t state);

/**
 * @brief EMCY 紧急错误码 ,  可读字符串
 * @return 错误描述, 未找到返回 NULL
 */
const char* motor_utils_emcy_str(uint16_t emcy_code);

/* =====================================================
 * 日志桥接 — 纯 C → C++ ECO_INFO_NEW
 * ===================================================== */

/** @brief 日志回调签名: C 侧 snprintf 格式化后传入, C++ 侧转发到 ECO_INFO_NEW */
typedef void (*motor_hal_log_cb_t)(const char *msg);

/** @brief 注册日志回调 (在 motor_hal_init 后调用) */
void motor_hal_set_log_callback(motor_hal_log_cb_t cb);

/** @brief 清除日志回调 */
void motor_hal_clear_log_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HAL_H */
