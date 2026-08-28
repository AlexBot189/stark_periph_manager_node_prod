/*
 * FootPressureSensor.cpp -- 足底压力传感器实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 串口线程: 阻塞 read → 环形缓冲 → 逐帧解析 → 顺序锁缓存
 * 帧解析: 移植 BatteryFrame::Unpack 模式
 */
#include "sensor/foot_pressure/FootPressureSensor.h"
#include "sensor/foot_pressure/FootPressureProtocol.h"

#include <log_helper/LogHelper.h>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <sys/ioctl.h>

namespace stark_periph_manager_node {

FootPressureSensor::FootPressureSensor()
{
}

FootPressureSensor::~FootPressureSensor()
{
    if (m_running.load(std::memory_order_acquire)) {
        Deinit();
    }
}

/* ---------- 串口配置 ---------- */

static bool _uart_config(int fd, int baud_rate)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        ECO_ERROR_NEW("[FootPressure] tcgetattr failed: {}", strerror(errno));
        return false;
    }

    /* 波特率 */
    speed_t speed;
    switch (baud_rate) {
    case 460800: speed = B460800; break;
    case 230400: speed = B230400; break;
    case 115200: speed = B115200; break;
    case 57600:  speed = B57600;  break;
    case 9600:   speed = B9600;   break;
    default:
        ECO_ERROR_NEW("[FootPressure] unsupported baud rate: {}", baud_rate);
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* 8N1, 无硬件流控 */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    /* 原始模式, 无行处理 */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;

    /* 读超时: 0.1s (10 字节间隔, 非帧超时) */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ECO_ERROR_NEW("[FootPressure] tcsetattr failed: {}", strerror(errno));
        return false;
    }

    /* 清空收发缓冲 */
    tcflush(fd, TCIOFLUSH);

    return true;
}

/* ---------- Init / Deinit ---------- */

bool FootPressureSensor::Init(const char* uart_dev, int baud_rate, int timeout_ms)
{
    if (m_ready.load(std::memory_order_acquire)) return true;

    m_timeout_ms = timeout_ms;

    /* 打开串口 */
    m_fd = open(uart_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        ECO_ERROR_NEW("[FootPressure] open({}) failed: {}", uart_dev, strerror(errno));
        return false;
    }

    if (!_uart_config(m_fd, baud_rate)) {
        close(m_fd);
        m_fd = -1;
        return false;
    }

    /* 配置完成后恢复阻塞模式, 让 read() 按 VTIME 超时等待 */
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags >= 0) {
	    fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    ECO_INFO_NEW("[FootPressure] uart={} baud={} timeout={}ms",
                 uart_dev, baud_rate, timeout_ms);

    /* 启动读线程 */
    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&FootPressureSensor::_ReaderThread, this);

    m_ready.store(true, std::memory_order_release);
    ECO_INFO_NEW("[FootPressure] init OK");
    return true;
}

void FootPressureSensor::Deinit()
{
    m_running.store(false, std::memory_order_release);

    if (m_thread.joinable()) {
        m_thread.join();
    }

    if (m_fd >= 0) {
        tcflush(m_fd, TCIOFLUSH);
        close(m_fd);
        m_fd = -1;
    }

    m_ready.store(false, std::memory_order_release);
    ECO_INFO_NEW("[FootPressure] deinit OK");
}

/* ---------- Read (非阻塞) ---------- */

void FootPressureSensor::Read(foot_pressure_data_t* out) const
{
    if (!out) return;

    m_cached.load(*out);
}

/* ---------- IsOnline ---------- */

bool FootPressureSensor::IsOnline() const
{
    /* 最近一次有效帧的时间戳非零则在 timeout 窗口内有数据 */
    if (!m_ready.load(std::memory_order_acquire)) return false;

    foot_pressure_data_t fp;
    Read(&fp);
    if (fp.timestamp_us == 0) return false;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    uint64_t elapsed = (now_us > fp.timestamp_us) ? (now_us - fp.timestamp_us) : 0;
    return (elapsed < (uint64_t)(m_timeout_ms * 1000));
}

/* ---------- GetStats ---------- */

void FootPressureSensor::GetStats(uint32_t* frames, uint32_t* errors) const
{
    if (frames) *frames = m_frame_count;
    if (errors) *errors = m_error_count;
}

/* ---------- 帧解析 (移植 BatteryFrame::Unpack 模式, 累加和校验) ---------- */

bool FootPressureSensor::_ParseFrame(const uint8_t* buf, size_t len,
                                     foot_pressure_data_t& out, size_t& consumed)
{
    consumed = 0;

    /* 数据不足最小帧长 */
    if (len < FOOT_PRESSURE_FRAME_LEN) {
        return false;
    }

    /* 搜索帧头 0xF2 */
    size_t hdr_pos = 0;
    bool found = false;
    for (size_t i = 0; i < len; i++) {
        if (buf[i] == FOOT_PRESSURE_FRAME_HEAD) {
            hdr_pos = i;
            found = true;
            break;
        }
    }

    if (!found) {
        /* 无有效帧头, 丢弃全部数据 */
        consumed = len;
        return false;
    }

    /* 帧头不在起始位置, 丢弃前方数据 */
    if (hdr_pos > 0) {
        consumed = hdr_pos;
        return false;
    }

    /* 检查帧长 */
    if (len < FOOT_PRESSURE_FRAME_LEN) {
        consumed = 0;
        return false;
    }

    /* 校验帧尾 (Byte 19) */
    if (buf[FOOT_PRESSURE_FRAME_LEN - 1] != FOOT_PRESSURE_FRAME_TAIL) {
        /* 帧尾不匹配, 从字节 1 重新搜索 */
        consumed = 1;
        return false;
    }

    /* 校验 SRC 地址 */
    if (buf[1] != FOOT_PRESSURE_FRAME_SRC) {
        consumed = FOOT_PRESSURE_FRAME_LEN;
        return false;
    }

    /* 校验累加和: Byte1~Byte17 (SRC 到最后一个数据字节) 累加取低 8 位 */
    {
        uint8_t calc_cs = 0;
        for (int i = 1; i < FOOT_PRESSURE_FRAME_CS_OFFSET; i++) {
            calc_cs += buf[i];
        }

        uint8_t rx_cs = buf[FOOT_PRESSURE_FRAME_CS_OFFSET];
        if (rx_cs != calc_cs) {
            /* 校验和不匹配, 帧头误判, 从字节 1 重新搜索 */
            consumed = 1;
            return false;
        }
    }

    /* 提取 6 个 uint16 AD 值 (大端 → 主机序, 全 16 位不做截断) */
    const uint8_t* data = buf + 6;
    uint16_t adc[FOOT_PRESSURE_PADS];
    for (int i = 0; i < FOOT_PRESSURE_PADS; i++) {
        adc[i] = ((uint16_t)data[i * 2] << 8) | (uint16_t)data[i * 2 + 1];
    }

    /* 填充输出: 左前掌/左足弓/左后跟 | 右前掌/右足弓/右后跟 */
    out.left.adc[0]  = adc[0];
    out.left.adc[1]  = adc[1];
    out.left.adc[2]  = adc[2];
    out.right.adc[0] = adc[3];
    out.right.adc[1] = adc[4];
    out.right.adc[2] = adc[5];

    /* 时间戳 */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    out.timestamp_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    /* update_cycle 由 RT 线程在下游填充, 此处不填 */
    out.update_cycle = 0;

    consumed = FOOT_PRESSURE_FRAME_LEN;
    return true;
}

/* ---------- 串口读线程 ---------- */

void FootPressureSensor::_ReaderThread()
{
    uint8_t ring_buf[FOOT_PRESSURE_RING_BUF_SIZE];
    size_t  ring_wpos = 0;

    /* 在线检测: 读超时到后判定 */
    bool was_online = false;

    ECO_INFO_NEW("[FootPressure] reader thread started");

    while (m_running.load(std::memory_order_acquire)) {
        /* 阻塞 read (VTIME=1 → 100ms 超时) */
        uint8_t tmp[256];
        ssize_t n = read(m_fd, tmp, sizeof(tmp));

        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                /* 无数据或信号打断, 继续循环 */
                continue;
            }
            ECO_ERROR_NEW("[FootPressure] read error: {}", strerror(errno));
            usleep(100000);
            continue;
        }

        if (n == 0) {
            /* VTIME 超时, 无数据 */
            continue;
        }

        /* 追加到环形缓冲 */
        for (ssize_t i = 0; i < n; i++) {
            ring_buf[ring_wpos] = tmp[i];
            ring_wpos = (ring_wpos + 1) % FOOT_PRESSURE_RING_BUF_SIZE;
        }

        /* 计算可消费数据长度 */
        size_t avail = ring_wpos;
        if (avail == 0) avail = FOOT_PRESSURE_RING_BUF_SIZE;

        /* 逐帧解析 */
        while (avail >= FOOT_PRESSURE_FRAME_LEN) {
            foot_pressure_data_t fp;
            size_t consumed = 0;

            bool ok = _ParseFrame(ring_buf, avail, fp, consumed);

            if (consumed == 0) {
                /* 数据不完整, 等下次 read */
                break;
            }

            /* 滑窗移除已消费数据 */
            size_t remaining = avail - consumed;
            if (remaining > 0) {
                memmove(ring_buf, ring_buf + consumed, remaining);
            }
            avail = remaining;

            if (!ok) {
                m_error_count++;
                continue;
            }

            /* 写入缓存 (无锁顺序锁) */
            m_cached.store(fp);

            m_frame_count++;

            /* FPS 统计: 每秒打印实际接收频率 + 最近一帧 AD 值 */
            {
                m_fps_count++;
                memcpy(&m_fps_last_frame, &fp, sizeof(fp));

                struct timespec ts_now;
                clock_gettime(CLOCK_MONOTONIC, &ts_now);
                uint64_t now_sec = (uint64_t)ts_now.tv_sec;
                if (m_fps_last_sec == 0) m_fps_last_sec = now_sec;

                if (now_sec - m_fps_last_sec >= 1) {
                    if (m_fps_count > 0) {
                        ECO_INFO_NEW("[FootPressure] FPS={} L:{} {} {}  R:{} {} {}",
                                     m_fps_count,
                                     m_fps_last_frame.left.adc[0],
                                     m_fps_last_frame.left.adc[1],
                                     m_fps_last_frame.left.adc[2],
                                     m_fps_last_frame.right.adc[0],
                                     m_fps_last_frame.right.adc[1],
                                     m_fps_last_frame.right.adc[2]);
                    } else {
                        ECO_DEBUG_NEW("[FootPressure] no frames in last 1s");
                    }
                    m_fps_count = 0;
                    m_fps_last_sec = now_sec;
                }
            }

            if (!was_online) {
                ECO_INFO_NEW("[FootPressure] sensor online");
                was_online = true;
            }
        }

        /* 更新写指针: 剩余未消费字节数 */
        ring_wpos = avail;
    }

    ECO_INFO_NEW("[FootPressure] reader thread stopped, frames={} errors={}",
                 m_frame_count, m_error_count);
}

} /* namespace stark_periph_manager_node */
