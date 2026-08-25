/*
 * rt_log.cpp — RT 日志实现 (init + drain 侧格式化)
 * Copyright (c) 2026 zhiqiang.yang
 */
#include "utils/rt_log.h"

#include <cstdio>
#include <cstring>
#include <unistd.h>

extern "C" {
#include "shm/shm_mgr.h"
}

/* ── 进程级上下文 (每进程一个, 定义在此) ── */
rt_log_ctx_t g_rt_log_ctx = { nullptr, 0 };

/* ── 格式串表 (由 X-Macro 编译期生成) ── */
#define RT_LOG_FMT(name, fmt) fmt,
static const char* const s_event_fmt[] = {
    RT_LOG_EVENTS(RT_LOG_FMT)
};
#undef RT_LOG_FMT

const char* rt_log_event_fmt(uint32_t event_id)
{
    if (event_id >= EV_MAX) return nullptr;
    return s_event_fmt[event_id];
}

int rt_log_init(const char* shm_name, int producer_index)
{
    if (producer_index < 0 || producer_index >= RT_LOG_MAX_PRODUCERS) {
        return -1;
    }

    stark_shm_mgr_t* mgr = stark_shm_mgr_open(shm_name, true, RT_LOG_SHM_SIZE);
    if (!mgr || !mgr->ptr) {
        return -1;
    }

    rt_log_shm_t* shm = (rt_log_shm_t*)mgr->ptr;

    /* 首次创建: 清零 + 写 magic/version + 默认开关/等级 */
    if (shm->magic != RT_LOG_MAGIC) {
        memset(shm, 0, RT_LOG_SHM_SIZE);
        shm->magic      = RT_LOG_MAGIC;
        shm->version    = RT_LOG_VERSION;
        shm->enabled    = 1;
        shm->max_level  = RT_LOG_INFO;
        shm->producer_count = 0;
    }

    /* 绑定本进程 producer 槽位 */
    rt_log_ring_t* ring = &shm->rings[producer_index];
    if (!ring->used) {
        ring->used = 1;
        ring->pid  = (uint32_t)getpid();
        ring->wr   = 0;
        ring->rd   = 0;
        ring->overflow = 0;
        __atomic_fetch_add(&shm->producer_count, 1, __ATOMIC_RELAXED);
    }

    g_rt_log_ctx.shm      = shm;
    g_rt_log_ctx.producer = producer_index;
    return 0;
}

/*
 * 格式化一条事件 (drain 侧, 非 RT)。
 * 解析格式串 specifier, 从 arg 数组取参数。
 * 支持 %u / %d / %x / %c / %llu / %lld / %%。
 */
int rt_log_format(const rt_log_event_t* ev, char* out, size_t n)
{
    if (!ev || !out || n == 0) return 0;

    const char* fmt = rt_log_event_fmt(ev->event_id);
    if (!fmt) {
        return snprintf(out, n, "event_%u(unknown)", ev->event_id);
    }

    size_t pos = 0;
    int    ai  = 0;

    for (const char* p = fmt; *p && pos + 1 < n; ) {
        if (*p != '%') {
            out[pos++] = *p++;
            continue;
        }

        p++;
        if (*p == '%') {
            out[pos++] = '%';
            p++;
            continue;
        }

        bool is_ll = false;
        if (p[0] == 'l' && p[1] == 'l') {
            is_ll = true;
            p += 2;
        }

        switch (*p) {
        case 'u':
        case 'd':
        case 'x': {
            if (is_ll) {
                uint64_t v = ((uint64_t)ev->arg[ai + 1] << 32) | ev->arg[ai];
                ai += 2;
                if (*p == 'd') {
                    pos += snprintf(out + pos, n - pos, "%lld", (long long)v);
                } else if (*p == 'x') {
                    pos += snprintf(out + pos, n - pos, "%llx", (unsigned long long)v);
                } else {
                    pos += snprintf(out + pos, n - pos, "%llu", (unsigned long long)v);
                }
            } else {
                uint32_t v = ev->arg[ai++];
                if (*p == 'd') {
                    pos += snprintf(out + pos, n - pos, "%d", (int)v);
                } else if (*p == 'x') {
                    pos += snprintf(out + pos, n - pos, "%x", v);
                } else {
                    pos += snprintf(out + pos, n - pos, "%u", v);
                }
            }
            p++;
            break;
        }
        case 'c': {
            pos += snprintf(out + pos, n - pos, "%c", (char)ev->arg[ai++]);
            p++;
            break;
        }
        default: {
            /* 不认识的 specifier: 原样输出 */
            out[pos++] = '%';
            out[pos++] = *p++;
            break;
        }
        }
    }

    out[pos] = '\0';
    return (int)pos;
}
