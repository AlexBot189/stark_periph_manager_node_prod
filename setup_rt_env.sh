#!/bin/sh
# stark_periph_node RT 环境检查和配置
# 适用: RV1126B + PREEMPT_RT 内核
# 用法: sudo ./setup_rt_env.sh

set -e

echo "=== stark_periph_node RT 环境检查 ==="

# 1. PREEMPT_RT 内核
echo -n "[1] PREEMPT_RT kernel: "
if uname -a | grep -q PREEMPT; then
    echo "✓"
else
    echo "✗ (need PREEMPT_RT kernel)"
fi

# 2. CPU isolation
echo -n "[2] CPU isolation: "
if cat /proc/cmdline | grep -q "isolcpus=2,3"; then
    echo "✓ (core 2,3 isolated)"
else
    echo "✗ 建议添加内核参数:"
    echo "   isolcpus=2,3 irqaffinity=0,1 rcu_nocbs=2,3 nohz_full=2,3"
fi

# 3. CAN 中断绑定到 core 0
echo -n "[3] CAN IRQ affinity: "
CAN_IRQ=$(cat /proc/interrupts | grep -i can | head -1 | awk '{print $1}' | tr -d ':')
if [ -n "$CAN_IRQ" ]; then
    CAN_AFFIN=$(cat /proc/irq/$CAN_IRQ/smp_affinity 2>/dev/null || echo "unknown")
    echo "IRQ=$CAN_IRQ affinity=$CAN_AFFIN"
    echo "   建议: echo 1 > /proc/irq/$CAN_IRQ/smp_affinity  (绑 core 0)"
else
    echo "CAN 未探测到 (需要 ip link set can0 up)"
fi

# 4. RT 优先级限制
echo -n "[4] RT priority limit: "
RLIMIT=$(ulimit -r 2>/dev/null || echo "unknown")
echo "$RLIMIT (需要 >= 90)"
if [ "$RLIMIT" != "unlimited" ] && [ "$RLIMIT" -lt 90 ] 2>/dev/null; then
    echo "   修改 /etc/security/limits.conf:"
    echo "   @realtime  hard  rtprio  99"
    echo "   @realtime  soft  rtprio  99"
fi

# 5. 内存锁定限制
echo -n "[5] memlock limit: "
MEMLOCK=$(ulimit -l 2>/dev/null || echo "unknown")
echo "$MEMLOCK (需要 unlimited)"
if [ "$MEMLOCK" != "unlimited" ]; then
    echo "   修改 /etc/security/limits.conf:"
    echo "   @realtime  hard  memlock  unlimited"
    echo "   @realtime  soft  memlock  unlimited"
fi

# 6. SHM 检查
echo -n "[6] /dev/shm: "
if [ -d /dev/shm ]; then
    SHM_SIZE=$(df -h /dev/shm | tail -1 | awk '{print $2}')
    echo "✓  size=$SHM_SIZE"
else
    echo "✗"
fi

# 7. CANFD 接口
echo -n "[7] CANFD interface: "
if ip link show can0 2>/dev/null | grep -q fd; then
    BITRATE=$(ip -details link show can0 2>/dev/null | grep -oP 'bitrate \K[0-9]+')
    DBITRATE=$(ip -details link show can0 2>/dev/null | grep -oP 'dbitrate \K[0-9]+')
    echo "✓ can0  arb=${BITRATE}bps data=${DBITRATE}bps"
else
    echo "✗ 需要配置:"
    echo "   ip link set can0 down"
    echo "   ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on"
    echo "   ip link set can0 up"
fi

echo ""
echo "=== 核心分配方案 ==="
echo "  Core 0: 内核 + 中断 (IRQ affinity=1)"
echo "  Core 1: 全部非 RT 任务 (主线程/log/ROS/WEB/IOT/OTA/配网) — CFS"
echo "  Core 2: 算法进程 (同事负责) — SCHED_FIFO 90"
echo "  Core 3: RT worker(SCHED_FIFO 90) + CAN recv(SCHED_FIFO 85)"
echo ""

# 快速配置 (仅当以 root 运行且 core 2,3 已隔离)
if [ "$(id -u)" = "0" ] && cat /proc/cmdline | grep -q "isolcpus=2,3"; then
    echo "=== 自动配置 ==="

    # CAN 中断绑 core 0
    if [ -n "$CAN_IRQ" ]; then
        echo "echo 1 > /proc/irq/$CAN_IRQ/smp_affinity"
        echo 1 > /proc/irq/$CAN_IRQ/smp_affinity 2>/dev/null || true
        echo "  → CAN IRQ 绑 core 0 ✓"
    fi

    # 迁移 core 2,3 上的现有进程到 core 0,1
    echo "Moving existing tasks off isolated cores..."
    for cpu in 2 3; do
        for pid in $(ps -eo pid,psr | grep " $cpu$" | awk '{print $1}' | grep -v "^$$\|^1$\|^0$" 2>/dev/null); do
            taskset -pc 0,1 $pid 2>/dev/null || true
        done
    done

    echo ""
    echo "RT 环境配置完成. 运行:"
    echo "  sudo ./stark_periph_manager_node"
    echo "  (另开终端) sudo ./algo_sim"
fi
