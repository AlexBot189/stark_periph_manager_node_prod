/*
 * imu_mount.h — IMU 坐标轴映射变换 (ICM/BHI360 共用)
 *
 * 将芯片坐标系的数据转换为机器人坐标系 (FLU: Front-Left-Up)。
 * 通过 axis[3] + sign[3] 描述映射关系:
 *   robot_x = sign[0] * chip[axis[0]]
 *   robot_y = sign[1] * chip[axis[1]]
 *   robot_z = sign[2] * chip[axis[2]]
 *
 * 例: robot=(-Z,-X,+Y) => axis={2,0,1}, sign={-1,-1,1}
 *
 * Copyright (c) 2026 zhiqiang.yang
 */
#pragma once

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 向量变换: robot = sign * chip[axis] */
static inline void imu_remap_vec(const int8_t axis[3], const int8_t sign[3],
                                 float *x, float *y, float *z)
{
    float v[3] = { *x, *y, *z };
    *x = (float)sign[0] * v[axis[0]];
    *y = (float)sign[1] * v[axis[1]];
    *z = (float)sign[2] * v[axis[2]];
}

/* 轴映射 -> 3x3 旋转矩阵 R (robot = R * chip, 行优先) */
static inline void imu_axis_map_to_matrix(const int8_t axis[3],
                                          const int8_t sign[3], float R[9])
{
    int i;
    for (i = 0; i < 9; i++) R[i] = 0.0f;
    R[0 * 3 + axis[0]] = (float)sign[0];
    R[1 * 3 + axis[1]] = (float)sign[1];
    R[2 * 3 + axis[2]] = (float)sign[2];
}

/* 旋转矩阵 -> 四元数 (Shepperd, w>=0 规范化) */
static inline void imu_matrix_to_quat(const float R[9],
                                     float *qw, float *qx, float *qy, float *qz)
{
    float trace = R[0] + R[4] + R[8];
    float s;
    if (trace > 0.0f) {
        s = sqrtf(trace + 1.0f) * 2.0f;
        *qw = 0.25f * s;
        *qx = (R[7] - R[5]) / s;
        *qy = (R[2] - R[6]) / s;
        *qz = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        *qw = (R[7] - R[5]) / s;
        *qx = 0.25f * s;
        *qy = (R[1] + R[3]) / s;
        *qz = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        *qw = (R[2] - R[6]) / s;
        *qx = (R[1] + R[3]) / s;
        *qy = 0.25f * s;
        *qz = (R[5] + R[7]) / s;
    } else {
        s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        *qw = (R[3] - R[1]) / s;
        *qx = (R[2] + R[6]) / s;
        *qy = (R[5] + R[7]) / s;
        *qz = 0.25f * s;
    }
    if (*qw < 0.0f) {
        *qw = -*qw; *qx = -*qx; *qy = -*qy; *qz = -*qz;
    }
}

/* 四元数变换: q_robot = q_chip ⊗ q_R^(-1) (右乘逆, Hamilton 乘法) */
static inline void imu_remap_quat(const int8_t axis[3], const int8_t sign[3],
                                  float *w, float *x, float *y, float *z)
{
    float R[9];
    float qrw, qrx, qry, qrz;
    float irw, irx, iry, irz;
    float qw = *w, qx = *x, qy = *y, qz = *z;

    imu_axis_map_to_matrix(axis, sign, R);
    imu_matrix_to_quat(R, &qrw, &qrx, &qry, &qrz);

    /* q_R 共轭 = q_R^(-1) */
    irw =  qrw; irx = -qrx; iry = -qry; irz = -qrz;

    /* Hamilton 乘法: q_robot = q_chip ⊗ q_R^(-1) */
    *w = qw*irw - qx*irx - qy*iry - qz*irz;
    *x = qw*irx + qx*irw + qy*irz - qz*iry;
    *y = qw*iry - qx*irz + qy*irw + qz*irx;
    *z = qw*irz + qx*iry - qy*irx + qz*irw;
}

#ifdef __cplusplus
}
#endif
