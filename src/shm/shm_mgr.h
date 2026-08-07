/*
 * @file stark_shm_mgr.h
 * @brief 共享内存管理 — POSIX shm 封装
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 管理 /dev/shm/stark_shm 的创建、映射和销毁.
 * 数据类型定义见 stark_shm.h (跨进程共享头文件)。
 */
#pragma once

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 共享内存段内部管理描述符 */
typedef struct {
    int     fd;         /* shm_open 文件描述符                          */
    void*   ptr;        /* mmap 映射基址, 指向 stark_shm_t 跨进程结构体     */
    size_t  size;       /* 映射区大小                                   */
    char*   name;       /* SHM 对象名                                   */
} stark_shm_mgr_t;

/**
 * @brief 创建或打开共享内存
 * @param name   SHM 名称 (如 "/stark_shm")
 * @param create true=新建 false=打开已有
 * @param size   大小 (字节)
 * @return 成功返回指针, 失败返回 NULL
 */
stark_shm_mgr_t* stark_shm_mgr_open(const char* name, bool create, size_t size);

/**
 * @brief 关闭并销毁共享内存
 */
void stark_shm_mgr_close(stark_shm_mgr_t* mgr);

#ifdef __cplusplus
}
#endif
