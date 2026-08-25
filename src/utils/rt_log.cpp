/*
 * rt_log.cpp — RT 日志实现 (init + drain 侧格式化 + 文件落地)
 * Copyright (c) 2026 zhiqiang.yang
 *
 * RT 侧 (rt_log_emit, 头文件 inline): 写共享内存无锁 ring, 零耗时.
 * drain 侧 (本文件): 轮询 ring → 查表格式化 → 写文件/console.
 * 文件滚动借鉴 loghelper 的 rotating 思想, 但自实现, 不依赖 spdlog,
 * 保证 RT 日志系统自包含 (算法进程无需链接 loghelper).
 */
#include "utils/rt_log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <string>
#include <ctime>
#include <unistd.h>
#include <sys/stat.h>

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

/* ── 环境变量解析 (借鉴 loghelper, 前缀 ECO_RT_LOG_*) ── */

static int rt_env_int(const char* name, int def)
{
    const char* v = getenv(name);
    if (v && v[0]) { int n = atoi(v); if (n > 0) return n; }
    return def;
}

/* 等级: trace/debug/info/warn/err/off 或 0~5 */
static int rt_env_level(const char* name, int def)
{
    const char* v = getenv(name);
    if (!v || !v[0]) return def;
    if (!strcmp(v, "trace")) return RT_LOG_TRACE;
    if (!strcmp(v, "debug")) return RT_LOG_DEBUG;
    if (!strcmp(v, "info"))  return RT_LOG_INFO;
    if (!strcmp(v, "warn"))  return RT_LOG_WARN;
    if (!strcmp(v, "err") || !strcmp(v, "error")) return RT_LOG_ERROR;
    if (!strcmp(v, "off"))  return RT_LOG_ERROR + 1;
    int n = atoi(v);
    if (n >= RT_LOG_TRACE && n <= RT_LOG_ERROR) return n;
    return def;
}

/* ── 文件 sink ── */

#define RT_SINK_FILE    (1u << 0)
#define RT_SINK_CONSOLE (1u << 1)

typedef struct {
    FILE*   fp;
    char    base[256];       /* 无扩展名路径 */
    size_t  max_size;        /* 单文件字节 */
    int     max_files;       /* 滚动文件数 (总文件 = max_files + 1) */
    size_t  written;         /* 当前文件已写字节 */
    int     flush_interval_s;
    time_t  last_flush;
    int     sink;            /* RT_SINK_* */
} rt_log_sink_t;

static rt_log_sink_t s_sink;

static void rt_mkdir_p(const char* path)
{
    std::string p(path);
    for (size_t i = 1; i < p.size(); i++) {
        if (p[i] == '/') {
            p[i] = '\0';
            mkdir(p.c_str(), 0777);
            p[i] = '/';
        }
    }
    mkdir(p.c_str(), 0777);
}

/* 路径 = ECO_HOME + ECO_RT_LOG_PATH + stark_rt/ */
static void rt_build_path(char* base, size_t n)
{
    std::string path;
    const char* home = getenv("ECO_HOME");
    if (home) path = home;
    const char* lp = getenv("ECO_RT_LOG_PATH");
    path += (lp && lp[0]) ? std::string(lp) : std::string("/tmp/log");
    path += "/stark_rt/";
    rt_mkdir_p(path.c_str());
    snprintf(base, n, "%sstark_rt", path.c_str());
}

/* 滚动: 删最老 + 依次重命名 + 开新文件 */
static void rt_sink_rotate(rt_log_sink_t* s)
{
    if (s->fp) { fclose(s->fp); s->fp = nullptr; }

    char p[320], q[320];
    snprintf(p, sizeof(p), "%s.%d.txt", s->base, s->max_files);
    remove(p);
    for (int i = s->max_files - 1; i >= 1; i--) {
        snprintf(q, sizeof(q), "%s.%d.txt", s->base, i);
        snprintf(p, sizeof(p), "%s.%d.txt", s->base, i + 1);
        rename(q, p);
    }
    snprintf(q, sizeof(q), "%s.txt", s->base);
    snprintf(p, sizeof(p), "%s.1.txt", s->base);
    rename(q, p);

    s->fp = fopen(q, "a");
    s->written = 0;
}

int rt_log_sink_init(void)
{
    memset(&s_sink, 0, sizeof(s_sink));

    /* sink: console/file/console_file 或 0/1/2 */
    const char* sv = getenv("ECO_RT_LOG_SINK");
    s_sink.sink = RT_SINK_CONSOLE | RT_SINK_FILE;  /* 默认 console+file */
    if (sv && sv[0]) {
        if (!strcmp(sv, "console"))      s_sink.sink = RT_SINK_CONSOLE;
        else if (!strcmp(sv, "file"))    s_sink.sink = RT_SINK_FILE;
        else if (!strcmp(sv, "console_file")) s_sink.sink = RT_SINK_CONSOLE | RT_SINK_FILE;
        else {
            int n = atoi(sv);
            if (n == 0) s_sink.sink = RT_SINK_CONSOLE;
            else if (n == 1) s_sink.sink = RT_SINK_FILE;
            else if (n == 2) s_sink.sink = RT_SINK_CONSOLE | RT_SINK_FILE;
        }
    }

    s_sink.max_size         = (size_t)rt_env_int("ECO_RT_LOG_SIZE", 10240) * 1024;  /* kb→byte, 默认10MB */
    s_sink.max_files        = rt_env_int("ECO_RT_LOG_COUNT", 10);
    s_sink.flush_interval_s = rt_env_int("ECO_RT_LOG_TIME", 5);

    if (s_sink.sink & RT_SINK_FILE) {
        rt_build_path(s_sink.base, sizeof(s_sink.base));
        char path[320];
        snprintf(path, sizeof(path), "%s.txt", s_sink.base);
        s_sink.fp = fopen(path, "a");
        if (s_sink.fp) {
            fseek(s_sink.fp, 0, SEEK_END);
            s_sink.written = (size_t)ftell(s_sink.fp);
        }
    }

    s_sink.last_flush = time(nullptr);
    return 0;
}

static void rt_sink_write_line(int level, const char* line)
{
    const char* lvl = level == RT_LOG_ERROR ? "ERR" :
                      level == RT_LOG_WARN  ? "WRN" :
                      level == RT_LOG_DEBUG ? "DBG" :
                      level == RT_LOG_TRACE ? "TRC" : "INF";

    if (s_sink.sink & RT_SINK_CONSOLE) {
        fprintf(stdout, "[RT][%s] %s\n", lvl, line);
    }
    if ((s_sink.sink & RT_SINK_FILE) && s_sink.fp) {
        size_t len = strlen(line) + 16;
        if (s_sink.written + len > s_sink.max_size) {
            rt_sink_rotate(&s_sink);
        }
        if (s_sink.fp) {
            s_sink.written += (size_t)fprintf(s_sink.fp, "[RT][%s] %s\n", lvl, line);
        }
    }

    /* flush: WARN/ERROR 立即, 否则按周期 */
    time_t now = time(nullptr);
    if (level >= RT_LOG_WARN || now - s_sink.last_flush >= s_sink.flush_interval_s) {
        if (s_sink.fp) fflush(s_sink.fp);
        if (s_sink.sink & RT_SINK_CONSOLE) fflush(stdout);
        s_sink.last_flush = now;
    }
}

void rt_log_sink_close(void)
{
    if (s_sink.fp) {
        fflush(s_sink.fp);
        fclose(s_sink.fp);
        s_sink.fp = nullptr;
    }
}

/* ── drain: 轮询所有 producer ring → 格式化 → 落地 ── */

void rt_log_drain(void)
{
    rt_log_shm_t* shm = g_rt_log_ctx.shm;
    if (!shm) return;

    for (int r = 0; r < RT_LOG_MAX_PRODUCERS; r++) {
        rt_log_ring_t* ring = &shm->rings[r];
        if (!ring->used) continue;

        uint32_t wr = __atomic_load_n(&ring->wr, __ATOMIC_RELAXED);
        uint32_t rd = __atomic_load_n(&ring->rd, __ATOMIC_RELAXED);
        while (rd != wr) {
            const rt_log_event_t* ev = &ring->events[rd];
            char buf[256];
            rt_log_format(ev, buf, sizeof(buf));
            rt_sink_write_line((int)ev->level, buf);
            rd = (rd + 1) % RT_LOG_RING_SIZE;
        }
        __atomic_store_n(&ring->rd, rd, __ATOMIC_RELAXED);
    }
}

/* ── init: 打开/创建 SHM + 绑定 producer 槽位 ── */

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

    /* 首次创建: 清零 + magic/version + 默认开关/等级 (等级由环境变量驱动) */
    if (shm->magic != RT_LOG_MAGIC) {
        memset(shm, 0, RT_LOG_SHM_SIZE);
        shm->magic      = RT_LOG_MAGIC;
        shm->version    = RT_LOG_VERSION;
        shm->enabled    = 1;
        shm->max_level  = (uint32_t)rt_env_level("ECO_RT_LOG_LEVEL", RT_LOG_INFO);
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
            out[pos++] = '%';
            out[pos++] = *p++;
            break;
        }
        }
    }

    out[pos] = '\0';
    return (int)pos;
}
