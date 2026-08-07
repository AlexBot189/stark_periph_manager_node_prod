/*
 * @file stark_shm_mgr.cpp
 * @brief 共享内存管理实现 — POSIX shm_open + mmap
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 管理 /dev/shm/stark_shm 的生命周期.
 * 数据类型由 stark_shm.h 定义 (跨进程共享布局)。
 */
#include "shm/shm_mgr.h"
#include <log_helper/LogHelper.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <new>

stark_shm_mgr_t* stark_shm_mgr_open(const char* name, bool create, size_t size)
{
    if (!name || size == 0) {
        ECO_ERROR("SHM: invalid name or size");
        return nullptr;
    }

    stark_shm_mgr_t* mgr = new (std::nothrow) stark_shm_mgr_t();
    if (!mgr) {
        ECO_ERROR("SHM: alloc failed");
        return nullptr;
    }

    mgr->name = new (std::nothrow) char[strlen(name) + 1];
    if (!mgr->name) {
        ECO_ERROR("SHM: name alloc failed");
        delete mgr;
        return nullptr;
    }
    strcpy(mgr->name, name);
    mgr->size = size;

    int flags = O_RDWR;
    if (create) {
        flags |= O_CREAT;
        /* 不先用 O_EXCL — 支持重新打开 */
    }

    mgr->fd = shm_open(name, flags, 0666);
    if (mgr->fd < 0) {
        ECO_ERROR("SHM: shm_open(%s) failed: %s", name, strerror(errno));
        delete[] mgr->name;
        delete mgr;
        return nullptr;
    }

    if (create) {
        if (ftruncate(mgr->fd, (off_t)size) < 0) {
            ECO_ERROR("SHM: ftruncate failed: %s", strerror(errno));
            close(mgr->fd);
            delete[] mgr->name;
            delete mgr;
            return nullptr;
        }
    }

    mgr->ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                    MAP_SHARED, mgr->fd, 0);
    if (mgr->ptr == MAP_FAILED) {
        ECO_ERROR("SHM: mmap failed: %s", strerror(errno));
        close(mgr->fd);
        delete[] mgr->name;
        delete mgr;
        return nullptr;
    }

    ECO_INFO("SHM: %s opened, size=%zu, ptr=%p",
             mgr->name, mgr->size, mgr->ptr);
    return mgr;
}

void stark_shm_mgr_close(stark_shm_mgr_t* mgr)
{
    if (!mgr) return;

    if (mgr->ptr && mgr->ptr != MAP_FAILED) {
        munmap(mgr->ptr, mgr->size);
    }
    if (mgr->fd >= 0) {
        close(mgr->fd);
    }
    /* 不调 shm_unlink — 让最后一个引用者决定 */
    ECO_INFO("SHM: %s closed", mgr->name ? mgr->name : "(null)");

    delete[] mgr->name;
    delete mgr;
}
