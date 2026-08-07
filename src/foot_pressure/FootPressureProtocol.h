/*
 * FootPressureProtocol.h -- 足底压力传感器协议常量
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 帧格式 (20 字节, 大端):
 *   Byte0:    帧头  0xF2
 *   Byte1:    SRC   0x01 (发送方: MCU)
 *   Byte2:    DST   0xC0 (接收方: RV1126B)
 *   Byte3:    FUNC  0xFF
 *   Byte4:    CMD   0xEE
 *   Byte5:    LEN   0x0C (数据长度 12)
 *   Byte6-17: DATA  6 x uint16 AD 值, 大端
 *   Byte18:   CS    校验和 (Byte1~Byte17 累加取低 8 位)
 *   Byte19:   帧尾  0xF1
 */
#pragma once

#include <cstdint>

#define FOOT_PRESSURE_FRAME_HEAD      0xF2
#define FOOT_PRESSURE_FRAME_TAIL      0xF1
#define FOOT_PRESSURE_FRAME_SRC       0x01
#define FOOT_PRESSURE_FRAME_DST       0xC0
#define FOOT_PRESSURE_FRAME_FUNC      0xFF
#define FOOT_PRESSURE_FRAME_CMD       0xEE
#define FOOT_PRESSURE_FRAME_DATA_LEN  12
#define FOOT_PRESSURE_FRAME_LEN       20       /* 1(hdr)+5(fixed)+12(data)+1(cs)+1(tail) */
#define FOOT_PRESSURE_FRAME_CS_OFFSET 18       /* 校验和字节在帧中的偏移 */
#define FOOT_PRESSURE_PADS            6        /* 6 路 AD */
#define FOOT_PRESSURE_RING_BUF_SIZE   4096     /* 环形缓冲 4KB */
#define FOOT_PRESSURE_STATS_INTERVAL  10000    /* 每 10000 帧打印一次间隔统计 */
