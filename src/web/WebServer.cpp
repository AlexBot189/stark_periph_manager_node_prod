/*
 * WebServer.cpp -- WebSocket debug server
 * Copyright (c) 2026 zhiqiang.yang
 *
 * Self-contained: no external WebSocket library needed.
 * Uses POSIX sockets + select(), embeds SHA1 + Base64 for handshake.
 *
 * Data flow:
 *   SHM periodic_data -> JSON -> WebSocket push -> browser charts
 *   Browser command -> WebSocket recv -> SHM mailbox / sdo_cmds
 */

#include "web/WebServer.h"
#include "log_helper/LogHelper.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdarg>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

namespace stark_periph_manager_node {

/* ================================================================
 * SHA1 (RFC 3174) -- minimal, for WebSocket handshake
 * ================================================================ */

struct Sha1Ctx {
    uint32_t state[5];
    uint64_t count;
    uint8_t  buf[64];
};

static void sha1_init(Sha1Ctx *ctx) {
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count = 0;
}

#define ROL(v,n) (((v) << (n)) | ((v) >> (32-(n))))

static void sha1_transform(uint32_t state[5], const uint8_t block[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++)
        w[i] = ((uint32_t)block[i*4] << 24) | ((uint32_t)block[i*4+1] << 16) |
               ((uint32_t)block[i*4+2] << 8) | block[i*4+3];
    for (int i = 16; i < 80; i++)
        w[i] = ROL(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3], e = state[4];
    for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20)      { f = (b & c) | (~b & d);        k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d;                  k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else             { f = b ^ c ^ d;                  k = 0xCA62C1D6; }
        uint32_t t = ROL(a, 5) + f + e + k + w[i];
        e = d; d = c; c = ROL(b, 30); b = a; a = t;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d; state[4] += e;
}

static void sha1_update(Sha1Ctx *ctx, const uint8_t *data, size_t len) {
    size_t i = (size_t)(ctx->count & 63);
    ctx->count += len;
    size_t space = 64 - i;
    if (len >= space) {
        memcpy(ctx->buf + i, data, space);
        sha1_transform(ctx->state, ctx->buf);
        for (size_t j = space; j + 63 < len; j += 64)
            sha1_transform(ctx->state, data + j);
        i = 0;
    } else { i += len; }
    memcpy(ctx->buf + i, data + len - (i - (size_t)((ctx->count - len) & 63)), 0);
    (void)i; /* unused after update, using simpler finalize */
}

static void sha1_update_simple(Sha1Ctx *ctx, const uint8_t *data, size_t len) {
    while (len > 0) {
        size_t i = (size_t)(ctx->count & 63);
        size_t n = 64 - i;
        if (n > len) n = len;
        memcpy(ctx->buf + i, data, n);
        ctx->count += n;
        data += n; len -= n;
        if ((ctx->count & 63) == 0)
            sha1_transform(ctx->state, ctx->buf);
    }
}

static void sha1_final(Sha1Ctx *ctx, uint8_t digest[20]) {
    uint64_t bits = ctx->count * 8;
    size_t i = (size_t)(ctx->count & 63);
    ctx->buf[i++] = 0x80;
    if (i > 56) {
        memset(ctx->buf + i, 0, 64 - i);
        sha1_transform(ctx->state, ctx->buf);
        i = 0;
    }
    memset(ctx->buf + i, 0, 56 - i);
    for (int j = 0; j < 8; j++)
        ctx->buf[56 + j] = (uint8_t)(bits >> (56 - j * 8));
    sha1_transform(ctx->state, ctx->buf);
    for (int j = 0; j < 5; j++) {
        digest[j*4]     = (uint8_t)(ctx->state[j] >> 24);
        digest[j*4 + 1] = (uint8_t)(ctx->state[j] >> 16);
        digest[j*4 + 2] = (uint8_t)(ctx->state[j] >> 8);
        digest[j*4 + 3] = (uint8_t)(ctx->state[j]);
    }
}

/* ================================================================
 * Base64 encode
 * ================================================================ */

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string base64_encode(const uint8_t *data, size_t len) {
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t)data[i] << 16;
        if (i + 1 < len) v |= (uint32_t)data[i+1] << 8;
        if (i + 2 < len) v |= data[i+2];
        out += b64_table[(v >> 18) & 0x3F];
        out += b64_table[(v >> 12) & 0x3F];
        out += (i + 1 < len) ? b64_table[(v >> 6) & 0x3F] : '=';
        out += (i + 2 < len) ? b64_table[v & 0x3F] : '=';
    }
    return out;
}

/* ================================================================
 * WebSocket frame encode (text, masked=false server->client)
 * ================================================================ */

static std::string ws_frame_text(const std::string &payload) {
    std::string frame;
    frame.reserve(payload.size() + 10);
    frame += (char)0x81;  /* FIN + text opcode */
    size_t len = payload.size();
    if (len <= 125) {
        frame += (char)len;
    } else if (len <= 65535) {
        frame += (char)126;
        frame += (char)((len >> 8) & 0xFF);
        frame += (char)(len & 0xFF);
    } else {
        frame += (char)127;
        for (int i = 7; i >= 0; i--)
            frame += (char)((len >> (i * 8)) & 0xFF);
    }
    frame += payload;
    return frame;
}

/* ================================================================
 * WebSocket frame decode (client->server, masked)
 * Returns payload or empty string on error.
 * ================================================================ */

static std::string ws_frame_decode(const uint8_t *data, size_t len) {
    if (len < 2) return "";
    bool masked = (data[1] & 0x80) != 0;
    size_t plen = data[1] & 0x7F;
    size_t hdr = 2;
    if (plen == 126) { if (len < 4) return ""; plen = ((size_t)data[2] << 8) | data[3]; hdr = 4; }
    else if (plen == 127) { if (len < 10) return ""; plen = 0; for (int i = 0; i < 8; i++) plen = (plen << 8) | data[2+i]; hdr = 10; }
    uint8_t mask[4] = {0,0,0,0};
    if (masked) {
        if (len < hdr + 4) return "";
        memcpy(mask, data + hdr, 4);
        hdr += 4;
    }
    if (len < hdr + plen) return "";
    std::string out(plen, '\0');
    for (size_t i = 0; i < plen; i++)
        out[i] = (char)(data[hdr + i] ^ (masked ? mask[i & 3] : 0));
    return out;
}

/* ================================================================
 * HTTP request line parser
 * ================================================================ */

struct HttpReq {
    std::string method;
    std::string path;
    std::string key;  /* Sec-WebSocket-Key */
};

static HttpReq parse_http_req(const std::string &raw) {
    HttpReq r;
    size_t pos = raw.find(' ');
    if (pos == std::string::npos) return r;
    r.method = raw.substr(0, pos);
    size_t pos2 = raw.find(' ', pos + 1);
    if (pos2 == std::string::npos) return r;
    r.path = raw.substr(pos + 1, pos2 - pos - 1);
    size_t kp = raw.find("Sec-WebSocket-Key:");
    if (kp != std::string::npos) {
        kp += 18;
        while (kp < raw.size() && raw[kp] == ' ') kp++;
        size_t ke = raw.find("\r\n", kp);
        if (ke == std::string::npos) ke = raw.size();
        r.key = raw.substr(kp, ke - kp);
    }
    return r;
}

/* ================================================================
 * WebSocket handshake response builder
 * ================================================================ */

static std::string ws_handshake_resp(const std::string &key) {
    std::string accept = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    Sha1Ctx ctx;
    sha1_init(&ctx);
    sha1_update_simple(&ctx, (const uint8_t*)accept.data(), accept.size());
    uint8_t digest[20];
    sha1_final(&ctx, digest);
    std::string b64 = base64_encode(digest, 20);

    std::string resp;
    resp += "HTTP/1.1 101 Switching Protocols\r\n";
    resp += "Upgrade: websocket\r\n";
    resp += "Connection: Upgrade\r\n";
    resp += "Sec-WebSocket-Accept: " + b64 + "\r\n";
    resp += "Access-Control-Allow-Origin: *\r\n\r\n";
    return resp;
}

static const char kHttpOkHeader[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n"
    "Content-Length: ";

/* ================================================================
 * JSON serialization (manual, no nlohmann/json dependency)
 * Pushes from SHM periodic_data -- richer than fb_buffer for viz.
 * ================================================================ */

static std::string serialize_to_json(stark_shm_t *shm, const WebServer::CmdTrack &track,
                                      const std::string &can_rx_json,
                                      const std::string &trace_json) {
    if (!shm) return "{}";

    PeriodicUploadData *d = &shm->periodic_data;

    /* SHM buffer ready age */
    uint32_t shm_age = 0;
    {
        uint32_t active = __atomic_load_n(&shm->active_idx, __ATOMIC_ACQUIRE);
        feedback_frame_t *fb = &shm->fb_buffer[active];
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
        if (now_us > fb->ts_shm_write)
            shm_age = (uint32_t)(now_us - fb->ts_shm_write);
    }

    char buf[4096];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"ts\":%u,"
        "\"ver\":%u,"
        "\"cycle\":%u,"
        "\"motor_ts\":%u,"
        "\"imu_ts\":%u,"
        "\"sensor_ts\":%u,"
        "\"rt_mode\":%u,"
        "\"state\":%u,"
        "\"calib\":%u,"
        "\"online\":%u,"
        /* right motor (ID=1) */
        "\"m1_angle\":%.1f,"
        "\"m1_vel\":%d,"
        "\"m1_cur\":%d,"
        "\"m1_busI\":%d,"
        "\"m1_temp\":%.2f,"
        "\"m1_fault\":%d,"
        "\"m1_state\":%d,"
        "\"m1_hall_a\":%u,"
        "\"m1_hall_b\":%u,"
        "\"m1_hall_c\":%u,"
        "\"m1_torque\":%u,"
        "\"m1_torque_fb\":%.2f,"
        "\"m1_torque_valid\":%u,"
        "\"m1_knee\":%d,"
        "\"m1_land\":%u,"
        /* left motor (ID=2) */
        "\"m2_angle\":%.1f,"
        "\"m2_vel\":%d,"
        "\"m2_cur\":%d,"
        "\"m2_busI\":%d,"
        "\"m2_temp\":%.2f,"
        "\"m2_fault\":%d,"
        "\"m2_state\":%d,"
        "\"m2_hall_a\":%u,"
        "\"m2_hall_b\":%u,"
        "\"m2_hall_c\":%u,"
        "\"m2_torque\":%u,"
        "\"m2_torque_fb\":%.2f,"
        "\"m2_torque_valid\":%u,"
        "\"m2_knee\":%d,"
        "\"m2_land\":%u,"
        /* IMU */
        "\"imu_roll\":%.1f,"
        "\"imu_pitch\":%.1f,"
        "\"imu_yaw\":%.1f,"
        "\"imu_acc_x\":%.3f,"
        "\"imu_acc_y\":%.3f,"
        "\"imu_acc_z\":%.3f,"
        "\"imu_gyro_x\":%.2f,"
        "\"imu_gyro_y\":%.2f,"
        "\"imu_gyro_z\":%.2f,"
        "\"imu_quat_w\":%.4f,"
        "\"imu_quat_x\":%.4f,"
        "\"imu_quat_y\":%.4f,"
        "\"imu_quat_z\":%.4f,"
        "\"imu_press\":%.1f,"
        "\"spi_torque\":%.1f,"
        "\"spi_valid\":%u,"
        "\"spi_torque2\":%.1f,"
        "\"spi_valid2\":%u,"
        /* 指令追踪 */
        "\"cmd_cur1\":%d,"
        "\"cmd_cur2\":%d,"
        "\"cmd_pos1\":%d,"
        "\"cmd_pos2\":%d,"
        "\"cmd_vel1\":%d,"
        "\"cmd_vel2\":%d,"
        "\"cmd_tq1\":%d,"
        "\"cmd_tq2\":%d,"
        "\"cmd_cur1_valid\":%d,"
        "\"cmd_cur2_valid\":%d,"
        "\"cmd_pos1_valid\":%d,"
        "\"cmd_pos2_valid\":%d,"
        "\"cmd_vel1_valid\":%d,"
        "\"cmd_vel2_valid\":%d,"
        "\"cmd_tq1_valid\":%d,"
        "\"cmd_tq2_valid\":%d,"
        "\"btn_state\":%u,"
        "\"btn_seq\":%u,"
        /* RT 状态 */
        "\"rt_priority\":%u,"
        "\"rt_cpu\":%u,"
        "\"perf_trace\":%u,"
        /* RT 抖动与状态 */
        "\"jitter_max\":%u,"
        "\"overrun\":%u,"
        "\"period_us\":%u,"
        "\"shm_age\":%u,"
        "\"foot_l1\":%u,"
        "\"foot_l2\":%u,"
        "\"foot_l3\":%u,"
        "\"foot_r1\":%u,"
        "\"foot_r2\":%u,"
        "\"foot_r3\":%u",
        d->timestamp_ms,
        shm->periodic_version,
        d->frame_cycle,
        d->motor_ts_us,
        d->imu_ts_us,
        d->sensor_ts_us,
        shm->rt_mode,
        shm->node_state,
        shm->calib_state,
        shm->motor_online,
        (float)d->motor_abs_angle / 10.0f,
        d->RealtimeVelocity,
        d->cal_Iq_current,
        d->cal_bus_current * 10,
        (float)d->motor_temp / 100.0f,
        d->fault_code,
        d->motor_state,
        d->hall_a_data, d->hall_b_data, d->hall_c_data,
        d->df181_torque,
        (float)d->torque_feedback * 0.05f,
        d->torque_valid,
        d->knee_hall,
        d->key_landing,
        (float)d->motor_abs_angle_left / 10.0f,
        d->RealtimeVelocity_left,
        d->cal_Iq_current_left,
        d->cal_bus_current_left * 10,
        (float)d->motor_temp_left / 100.0f,
        d->fault_code_left,
        d->motor_state_left,
        d->hall_a_data_left, d->hall_b_data_left, d->hall_c_data_left,
        d->df181_torque_left,
        (float)d->torque_feedback_left * 0.05f,
        d->torque_valid_left,
        d->knee_hall_left,
        d->key_landing_left,
        d->gyro_roll, d->gyro_pitch, d->gyro_yaw,
        d->acc_x, d->acc_y, d->acc_z,
        d->gyro_dps_x, d->gyro_dps_y, d->gyro_dps_z,
        d->quat_w, d->quat_x, d->quat_y, d->quat_z,
        d->air_pressure,
        d->spi_torque,
        d->spi_valid,
        d->spi_torque_left,
        d->spi_valid_left,
        /* 指令追踪 */
        track.cur_m1, track.cur_m2,
        track.pos_m1, track.pos_m2,
        track.vel_m1, track.vel_m2,
        track.tq_m1, track.tq_m2,
        (int)track.cur_valid_m1, (int)track.cur_valid_m2,
        (int)track.pos_valid_m1, (int)track.pos_valid_m2,
        (int)track.vel_valid_m1, (int)track.vel_valid_m2,
        (int)track.tq_valid_m1, (int)track.tq_valid_m2,
        (unsigned)__atomic_load_n(&shm->btn_report_state, __ATOMIC_ACQUIRE),
        (unsigned)__atomic_load_n(&shm->btn_report_seq, __ATOMIC_ACQUIRE),
        /* RT 状态 */
        shm->rt_priority,
        shm->rt_cpu,
        shm->perf_trace_enabled,
        /* RT 抖动与状态 */
        shm->cycle_jitter_max_us,
        shm->cycle_overrun_count,
        shm->period_us,
        shm_age,
        d->foot_pressure.left.adc[0],
        d->foot_pressure.left.adc[1],
        d->foot_pressure.left.adc[2],
        d->foot_pressure.right.adc[0],
        d->foot_pressure.right.adc[1],
        d->foot_pressure.right.adc[2]
    );
    (void)n;

    /* append can_rx JSON */
    std::string json(buf);
    json += can_rx_json;
    json += trace_json;
    json += "}";
    return json;
}

/* ================================================================
 * BuildTraceJson — 读逐帧监控 SHM, 构造增量 trace JSON 片段
 *
 * 增量语义: 只推上次 head 之后的新样本 (5ms 推送约 5 帧, 不额外耗时),
 * 前端自行累积滚动窗口. 首次或游标异常时对齐到最近 256 帧.
 * ================================================================ */

std::string WebServer::BuildTraceJson()
{
    if (!m_trace_shm) return "";

    uint32_t ctrl_h   = __atomic_load_n(&m_trace_shm->ctrl_head,   __ATOMIC_ACQUIRE);
    uint32_t fb_h     = __atomic_load_n(&m_trace_shm->fb_head,     __ATOMIC_ACQUIRE);
    uint32_t jitter_h = __atomic_load_n(&m_trace_shm->jitter_head, __ATOMIC_ACQUIRE);

    std::string out;
    out += ",\"trace_head\":[" + std::to_string(ctrl_h) + "," + std::to_string(fb_h) + "," + std::to_string(jitter_h) + "]";

    /* 分段耗时统计区 (min/avg/max, 唯一写者, WebServer 只读) */
    {
        auto stat_json = [](const trace_stat_t& s) -> std::string {
            uint32_t n = s.count;
            if (n == 0) return "{\"min\":0,\"avg\":0,\"max\":0,\"n\":0}";
            uint32_t avg = (uint32_t)(s.sum_us / n);
            char tmp[64];
            snprintf(tmp, sizeof(tmp), "{\"min\":%u,\"avg\":%u,\"max\":%u,\"n\":%u}",
                     s.min_us, avg, s.max_us, n);
            return std::string(tmp);
        };
        out += ",\"trace_stat\":{"
               "\"up_seg1\":"  + stat_json(m_trace_shm->up_seg1)  + ","
               "\"up_seg2\":"  + stat_json(m_trace_shm->up_seg2)  + ","
               "\"up_total\":" + stat_json(m_trace_shm->up_total) + ","
               "\"dn_seg1\":"  + stat_json(m_trace_shm->dn_seg1)  + ","
               "\"dn_seg2\":"  + stat_json(m_trace_shm->dn_seg2)  + ","
               "\"dn_total\":" + stat_json(m_trace_shm->dn_total) +
               "}";
    }

    /* ctrl ring 增量 (单帧/多帧控制 e2e) */
    {
        uint32_t last = m_trace_last_ctrl_head;
        if (!m_trace_synced || last > ctrl_h) {
            last = (ctrl_h > 512) ? (ctrl_h - 512) : 0;
        }
        uint32_t count = ctrl_h - last;
        if (count > 512) count = 512;
        out += ",\"trace_ctrl\":[";
        for (uint32_t i = 0; i < count; i++) {
            uint32_t cursor = last + i;
            const trace_sample_t* s = &m_trace_shm->ctrl_samples[cursor % STARK_TRACE_CTRL_RING];
            char tmp[48];
            snprintf(tmp, sizeof(tmp), "%s[%u,%u,%u,%u]",
                     i ? "," : "", s->cycle, s->kind, s->e2e_us, s->motor_id);
            out += tmp;
        }
        out += "]";
        m_trace_last_ctrl_head = last + count;
    }

    /* fb ring 增量 (反馈帧 e2e) */
    {
        uint32_t last = m_trace_last_fb_head;
        if (!m_trace_synced || last > fb_h) {
            last = (fb_h > 512) ? (fb_h - 512) : 0;
        }
        uint32_t count = fb_h - last;
        if (count > 512) count = 512;
        out += ",\"trace_fb\":[";
        for (uint32_t i = 0; i < count; i++) {
            uint32_t cursor = last + i;
            const trace_sample_t* s = &m_trace_shm->fb_samples[cursor % STARK_TRACE_FB_RING];
            char tmp[32];
            snprintf(tmp, sizeof(tmp), "%s[%u,%u]", i ? "," : "", s->cycle, s->e2e_us);
            out += tmp;
        }
        out += "]";
        m_trace_last_fb_head = last + count;
    }

    /* jitter ring 增量 */
    {
        uint32_t last = m_trace_last_jitter_head;
        if (!m_trace_synced || last > jitter_h) {
            last = (jitter_h > 1024) ? (jitter_h - 1024) : 0;
        }
        uint32_t count = jitter_h - last;
        if (count > 1024) count = 1024;
        out += ",\"trace_jitter\":[";
        for (uint32_t i = 0; i < count; i++) {
            uint32_t cursor = last + i;
            const trace_sample_t* s = &m_trace_shm->jitter_samples[cursor % STARK_TRACE_JITTER_RING];
            char tmp[32];
            snprintf(tmp, sizeof(tmp), "%s[%u,%u]", i ? "," : "", s->cycle, s->e2e_us);
            out += tmp;
        }
        out += "]";
        m_trace_last_jitter_head = last + count;
    }

    m_trace_synced = true;
    return out;
}

/* ================================================================
 * Command dispatcher -- parse JSON command from client
 * Extended with all demo_algo commands:
 *   single-frame: cur/pos/vel (SDO/PDO, single or dual motor)
 *   management:   enable/disable/estop/clearf/calib
 *   LED:          led (mask, mode, r, g, b)
 *   status:       btn (read button state)
 * ================================================================ */

static void dispatch_command(stark_shm_t *shm, motor_hal_t *hal,
                              const std::string &json,
                              WebServer::CmdTrack &track) {
    if (!shm) return;
    auto get_str = [&](const char *key) -> std::string {
        std::string pat = std::string("\"") + key + "\":\"";
        size_t p = json.find(pat);
        if (p == std::string::npos) return "";
        p += pat.size();
        size_t e = json.find('"', p);
        if (e == std::string::npos) return "";
        return json.substr(p, e - p);
    };
    auto get_int = [&](const char *key, int def = 0) -> int {
        std::string pat = std::string("\"") + key + "\":";
        size_t p = json.find(pat);
        if (p == std::string::npos) return def;
        p += pat.size();
        while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) p++;
        char *end;
        long v = strtol(json.c_str() + p, &end, 10);
        return (int)v;
    };
    auto get_bool = [&](const char *key, bool def = false) -> bool {
        std::string pat = std::string("\"") + key + "\":";
        size_t p = json.find(pat);
        if (p == std::string::npos) return def;
        p += pat.size();
        while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) p++;
        if (json[p] == 't' || json[p] == 'T') return true;
        if (json[p] == '1') return true;
        return false;
    };

    std::string cmd   = get_str("cmd");
    std::string mode  = get_str("mode");  /* "pdo" / "sdo" */

    /* single motor for management commands */
    int motor_id = get_int("motor", 0);
    /* dual motor values for control commands */
    int m1_val = get_int("m1", 0);
    int m2_val = get_int("m2", 0);
    int acc    = get_int("acc", 0);
    int vel_ov = get_int("vel", 0);
    /* old-style single motor fields */
    int value = get_int("value", 0);
    int motor_old = get_int("motor_single", 0);

    /* support old-style {"motor":N,"value":V} for backward compat */
    if (motor_old > 0 && motor_old <= 2 && value != 0) {
        motor_id = motor_old;
        m1_val = value;
    }
    /* support mixed {"motor":N,"value":V} */
    if (motor_id > 0 && motor_id <= 2 && m1_val == 0 && value != 0) {
        m1_val = value;
    }

    ECO_INFO_NEW("[WebServer] cmd={} mode={} motor={} m1={} m2={} acc={} vel={}",
           cmd, mode, motor_id, m1_val, m2_val, acc, vel_ov);

    /* ================================================================
     * management commands
     * ================================================================ */
    if (cmd == "enable" || cmd == "disable" || cmd == "estop" || cmd == "clearf") {
        int ids[2] = {0, 0};
        int n = 0;
        if (motor_id == 3) { ids[0] = 1; ids[1] = 2; n = 2; }  /* both */
        else if (motor_id >= 1 && motor_id <= 2) { ids[0] = motor_id; n = 1; }
        else { ECO_INFO_NEW("[WebServer] mgmt: invalid motor={}", motor_id); return; }

        for (int i = 0; i < n; i++) {
            int id = ids[i];
            int idx = id - 1;
            uint8_t ctype = 0;
            if (cmd == "enable")       ctype = STARK_CMD_ENABLE;
            else if (cmd == "disable") ctype = STARK_CMD_DISABLE;
            else if (cmd == "estop")   ctype = STARK_CMD_ESTOP;
            else if (cmd == "clearf")  ctype = STARK_CMD_CLEAR_FAULT;
            shm->mgmt_cmd[idx] = ctype;
            shm->mgmt_seq[idx]++;
            ECO_INFO_NEW("[WebServer] mgmt M{}: {}", id, cmd);
        }
        return;
    }

    if (cmd == "calib") {
        shm->calib_requested = 1;
        ECO_INFO_NEW("[WebServer] calibration requested");
        return;
    }

    if (cmd == "mit_migrate") {
        if (motor_id < 1 || motor_id > 2) {
            ECO_INFO_NEW("[WebServer] mit_migrate: invalid motor={}", motor_id);
            return;
        }
        int idx = motor_id - 1;
        sdo_cmd_slot_t *s = &shm->sdo_cmds[idx];
        memset(s, 0, sizeof(*s));
        s->motor_id = (uint8_t)motor_id;
        s->cmd      = STARK_CMD_SDO_MIT_MIGRATE;
        __atomic_store_n(&shm->sdo_seq[idx],
            __atomic_load_n(&shm->sdo_seq[idx], __ATOMIC_RELAXED) + 1,
            __ATOMIC_RELEASE);
        ECO_INFO_NEW("[WebServer] mit_migrate M{}: Tmax=20 + save Flash", motor_id);
        return;
    }

    if (cmd == "calib_torque") {
        int torque_mNm = get_int("value", 0);
        if (motor_id < 1 || motor_id > 2) {
            ECO_INFO_NEW("[WebServer] calib_torque: invalid motor={}", motor_id);
            return;
        }
        if (torque_mNm < -100000 || torque_mNm > 100000) {
            ECO_INFO_NEW("[WebServer] calib_torque: torque {} mNm out of [-100000,100000]", torque_mNm);
            return;
        }
        int idx = motor_id - 1;
        sdo_cmd_slot_t *s = &shm->sdo_cmds[idx];
        memset(s, 0, sizeof(*s));
        s->motor_id = (uint8_t)motor_id;
        s->cmd      = STARK_CMD_SDO_TORQUE_CALIB;
        s->value    = torque_mNm;
        __atomic_store_n(&shm->sdo_seq[idx],
            __atomic_load_n(&shm->sdo_seq[idx], __ATOMIC_RELAXED) + 1,
            __ATOMIC_RELEASE);
        ECO_INFO_NEW("[WebServer] calib_torque M{}: {} mNm", motor_id, torque_mNm);
        return;
    }

    /* ================================================================
     * LED control: {"cmd":"led","motor":N,"mask":240,"mode":0,"r":255,"g":0,"b":0}
     * ================================================================ */
    if (cmd == "led") {
        int mask  = get_int("mask", 0);
        int lmode = get_int("lmode", 0);  /* "lmode" to avoid clash with "mode" */
        int r = get_int("r", 0);
        int g = get_int("g", 0);
        int b = get_int("b", 0);
        if (motor_id <= 0 || motor_id > 2) {
            ECO_INFO_NEW("[WebServer] LED: invalid motor={}", motor_id);
            return;
        }
        int idx = motor_id - 1;
        shm->led_cfg[idx].enable_mask = (uint8_t)mask;
        shm->led_cfg[idx].mode        = (uint8_t)lmode;
        shm->led_cfg[idx].r           = (uint8_t)r;
        shm->led_cfg[idx].g           = (uint8_t)g;
        shm->led_cfg[idx].b           = (uint8_t)b;
        __atomic_add_fetch(&shm->led_seq[idx], 1, __ATOMIC_RELEASE);
        ECO_INFO_NEW("[WebServer] LED M{}: mask=0x{:02X} mode={} RGB={},{},{}",
               motor_id, mask, lmode, r, g, b);
        return;
    }

    /* ================================================================
     * cansend: {"cmd":"cansend","iface":"can0","id":"601","data":"2B4010000F000000"}
     * ================================================================ */
    if (cmd == "cansend") {
        std::string iface  = get_str("iface");
        std::string id_hex = get_str("id");
        std::string data   = get_str("data");
        bool        is_fd  = get_bool("is_fd");
        if (iface.empty())  iface = "can0";
        if (id_hex.empty() || data.empty()) {
            ECO_INFO_NEW("[WebServer] cansend: missing id or data");
            return;
        }
        /* sanitize: strip 0x prefix and whitespace */
        if (id_hex.size() >= 2 && (id_hex[0] == '0' || id_hex[0] == '0') &&
            (id_hex[1] == 'x' || id_hex[1] == 'X'))
            id_hex = id_hex.substr(2);
        /* auto-detect CANFD by data length (>8 bytes = CANFD) */
        if (!is_fd && data.length() > 16) is_fd = true;
        /* build command: '#' for classical CAN, '##' for CANFD */
        std::string shell = "cansend " + iface + " " + id_hex + (is_fd ? "##" : "#") + data;
        ECO_INFO_NEW("[WebServer] cansend: {}", shell);
        int ret = system(shell.c_str());
        if (ret != 0) {
            ECO_INFO_NEW("[WebServer] cansend failed: {}", ret);
        }
        return;
    }

    /* ================================================================
     * control commands: cur / pos / vel (SDO or PDO)
     * Supports single motor (m1 only), dual same (m1=m2), dual diff (m1!=m2)
     * ================================================================ */
    if (cmd == "cur" || cmd == "pos" || cmd == "vel") {
        int ids[2] = {0, 0};
        int vals[2] = {0, 0};
        int n_motors = 0;

        if (motor_id == 3) {
            /* both motors */
            ids[0] = 1; vals[0] = (m1_val != 0) ? m1_val : m2_val;
            ids[1] = 2; vals[1] = (m2_val != 0) ? m2_val : vals[0];
            n_motors = 2;
        } else if (motor_id >= 1 && motor_id <= 2) {
            ids[0] = motor_id;
            vals[0] = (m1_val != 0) ? m1_val : value;  /* m1_val or old-style value */
            n_motors = 1;
        } else {
            ECO_INFO_NEW("[WebServer] ctrl: invalid motor={}", motor_id);
            return;
        }

        /* track cmd values */
        for (int i = 0; i < n_motors; i++) {
            int id = ids[i];
            int v  = vals[i];
            if (id == 1) {
                if (cmd == "cur")       { track.cur_m1 = v; track.cur_valid_m1 = true; }
                else if (cmd == "pos")  { track.pos_m1 = v; track.pos_valid_m1 = true; }
                else if (cmd == "vel")  { track.vel_m1 = v; track.vel_valid_m1 = true; }
                else if (cmd == "tq")   { track.tq_m1  = v; track.tq_valid_m1  = true; }
            } else if (id == 2) {
                if (cmd == "cur")       { track.cur_m2 = v; track.cur_valid_m2 = true; }
                else if (cmd == "pos")  { track.pos_m2 = v; track.pos_valid_m2 = true; }
                else if (cmd == "vel")  { track.vel_m2 = v; track.vel_valid_m2 = true; }
                else if (cmd == "tq")   { track.tq_m2  = v; track.tq_valid_m2  = true; }
            }
        }

        std::string ctrl_msg = std::string("[WebServer] ") + cmd + " M:";
        for (int i = 0; i < n_motors; i++)
            ctrl_msg += " " + std::to_string(ids[i]) + "=" + std::to_string(vals[i]);
        if (acc) ctrl_msg += " acc=" + std::to_string(acc);
        if (vel_ov) ctrl_msg += " vel=" + std::to_string(vel_ov);
        ECO_INFO_NEW("[WebServer] {}", ctrl_msg);

        /* dispatch to SHM */
        if (mode == "pdo") {
            /* PDO: write to SHM mailbox (RT thread processes) */
            uint8_t ctype = 0;
            if (cmd == "cur")       ctype = STARK_CMD_TORQUE;
            else if (cmd == "vel")  ctype = STARK_CMD_PV;
            else if (cmd == "pos")  ctype = STARK_CMD_POS;

            for (int i = 0; i < n_motors; i++) {
                int id = ids[i];
                int v  = vals[i];

                uint64_t w = __atomic_load_n(&shm->mailbox.seq_write, __ATOMIC_ACQUIRE);
                uint64_t r = __atomic_load_n(&shm->mailbox.seq_read, __ATOMIC_ACQUIRE);
                if (w - r >= STARK_MBOX_DEPTH) { ECO_INFO_NEW("[WebServer] PDO mailbox full"); return; }

                uint32_t midx = w & (STARK_MBOX_DEPTH - 1);
                motor_command_t *c = &shm->mailbox.frames[midx].cmd[id - 1];
                memset(c, 0, sizeof(*c));
                c->motor_id = (uint8_t)id;
                c->cmd      = ctype;

                if (cmd == "pos") {
                    /* value is deg*100, convert to counts */
                    float deg = (float)v / 100.0f;
                    c->value = (int32_t)(deg * 16384.0f / 360.0f);
                    if (acc)     c->value2 = acc * 100;    /* RPM/s * 100 */
                    if (vel_ov)  c->feedforward = vel_ov;  /* RPM */
                } else if (cmd == "vel") {
                    c->value = v * 100;  /* RPM * 100 */
                    if (acc) c->value2 = acc * 100;
                } else {
                    c->value = v;  /* mA */
                }

                __atomic_store_n(&shm->mailbox.seq_write, w + 1, __ATOMIC_RELEASE);
            }
        } else if (mode == "sdo") {
            /* SDO: write to sdo_cmds (main loop processes) */
            for (int i = 0; i < n_motors; i++) {
                int id = ids[i];
                int v  = vals[i];

                uint8_t ctype = 0;
                if (cmd == "cur")       ctype = STARK_CMD_SDO_CUR;
                else if (cmd == "vel")  ctype = STARK_CMD_SDO_VEL;
                else if (cmd == "pos")  ctype = STARK_CMD_SDO_POS;

                uint32_t midx = (uint32_t)(id - 1);
                sdo_cmd_slot_t *s = &shm->sdo_cmds[midx];
                memset(s, 0, sizeof(*s));
                s->motor_id = (uint8_t)id;
                s->cmd      = ctype;
                if (cmd == "pos") {
                    s->value = v;          /* deg*100 */
                    s->value2 = acc;       /* accel RPM/s */
                    s->feedforward = vel_ov; /* vel RPM */
                } else if (cmd == "vel") {
                    s->value = v * 100;    /* RPM*100 */
                    s->value2 = acc;
                } else {
                    s->value = v;          /* mA */
                }
                __atomic_store_n(&shm->sdo_seq[midx],
                    __atomic_load_n(&shm->sdo_seq[midx], __ATOMIC_RELAXED) + 1,
                    __ATOMIC_RELEASE);
            }
        }
        return;
    }

    if (cmd == "report_start" || cmd == "report_stop") {
        return; /* handled in PullLoop */
    }

    ECO_INFO_NEW("[WebServer] unknown command: {}", cmd);
}

/* ================================================================
 * Constructor / Destructor
 * ================================================================ */

WebServer::WebServer(stark_shm_t* shm, uint16_t port, uint32_t push_period_ms)
    : m_shm(shm)
    , m_port(port)
    , m_push_period_us(push_period_ms * 1000U)
    , m_running(false)
    , m_frame_count(0)
    , m_fail_count(0)
{
    ECO_INFO_NEW("[WebServer] push_period={}ms ({}us)", push_period_ms, m_push_period_us);
}

WebServer::~WebServer()
{
    if (m_running.load(std::memory_order_acquire)) {
        Stop();
    }
}

/* ================================================================
 * Update() -- IListener
 * ================================================================ */

void WebServer::Update(const boost::any& data)
{
    (void)data;
}

/* ================================================================
 * Start / Stop
 * ================================================================ */

void WebServer::Start()
{
    if (m_running.load(std::memory_order_acquire)) return;

    m_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_listen_fd < 0) {
        ECO_INFO_NEW("[WebServer] socket() failed: {}", strerror(errno));
        return;
    }

    int opt = 1;
    setsockopt(m_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(m_port);

    if (bind(m_listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ECO_INFO_NEW("[WebServer] bind(:{}) failed: {}", m_port, strerror(errno));
        close(m_listen_fd);
        m_listen_fd = -1;
        return;
    }

    if (listen(m_listen_fd, 5) < 0) {
        ECO_INFO_NEW("[WebServer] listen() failed: {}", strerror(errno));
        close(m_listen_fd);
        m_listen_fd = -1;
        return;
    }

    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&WebServer::PullLoop, this);

    /* start candump RX watcher */
    StartCandump("can0");

    ECO_INFO_NEW("[WebServer] listening on :{}", m_port);
}

void WebServer::Stop()
{
    m_running.store(false, std::memory_order_release);

    /* stop candump */
    StopCandump();

    /* wake up select() */
    if (m_listen_fd >= 0) {
        shutdown(m_listen_fd, SHUT_RDWR);
    }

    if (m_thread.joinable()) {
        m_thread.join();
    }

    for (int fd : m_clients) close(fd);
    m_clients.clear();
    if (m_listen_fd >= 0) {
        close(m_listen_fd);
        m_listen_fd = -1;
    }
    ECO_INFO_NEW("[WebServer] stopped. frames={} failures={}",
           m_frame_count, m_fail_count);
}

/* ================================================================
 * candump RX -- popen("candump -x <iface>") reader thread
 * ================================================================ */

void WebServer::StartCandump(const char* iface)
{
    if (m_candump_running.load(std::memory_order_acquire)) return;

    std::string cmd = std::string("candump -x ") + iface;
    m_candump_fp = popen(cmd.c_str(), "r");
    if (!m_candump_fp) {
        ECO_INFO_NEW("[WebServer] candump: popen({}) failed: {}", cmd, strerror(errno));
        return;
    }

    /* set non-blocking so fgets doesn't hang during Stop */
    int fd = fileno(m_candump_fp);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    m_candump_running.store(true, std::memory_order_release);
    m_candump_thread = std::thread(&WebServer::CandumpLoop, this);
    ECO_INFO_NEW("[WebServer] candump started on {}", iface);
}

void WebServer::StopCandump()
{
    if (!m_candump_running.load(std::memory_order_acquire)) return;
    m_candump_running.store(false, std::memory_order_release);

    /* close the pipe to unblock fgets */
    if (m_candump_fp) {
        pclose(m_candump_fp);
        m_candump_fp = nullptr;
    }

    if (m_candump_thread.joinable()) {
        m_candump_thread.join();
    }
    ECO_INFO_NEW("[WebServer] candump stopped");
}

void WebServer::CandumpLoop()
{
    char line[256];
    while (m_candump_running.load(std::memory_order_acquire)) {
        if (!m_candump_fp) break;

        char* ret = fgets(line, sizeof(line), m_candump_fp);
        if (!ret) {
            usleep(5000);  /* no data yet, wait a bit */
            continue;
        }

        /* strip trailing newline */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';
        if (len == 0) continue;

        std::lock_guard<std::mutex> lk(m_can_rx_mutex);
        if (m_can_rx_buf.size() >= (size_t)CANDUMP_BUF_SIZE)
            m_can_rx_buf.erase(m_can_rx_buf.begin());
        m_can_rx_buf.push_back(std::string(line));
        m_can_rx_seq++;
    }
}

/* ================================================================
 * send_all -- write() with partial-write loop
 * ================================================================ */

static bool send_all(int fd, const void *data, size_t len) {
    const char *p = (const char*)data;
    while (len > 0) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) return false;
        p += n; len -= (size_t)n;
    }
    return true;
}

/* ================================================================
 * HTTP handler -- serves the debug HTML page
 * ================================================================ */

static std::string g_html_cache;
static std::mutex  g_html_mutex;

static void handle_http(int fd) {
    std::string body;
    {
        std::lock_guard<std::mutex> lk(g_html_mutex);
        if (g_html_cache.empty()) {
            /* try to read from file, fallback to embedded inline page */
            const char *paths[] = {
                "/data/stark/web/stark_node_debug.html",
                "stark_periph_node/src/web/stark_node_debug.html",
                NULL
            };
            for (int i = 0; paths[i]; i++) {
                FILE *fp = fopen(paths[i], "r");
                if (fp) {
                    fseek(fp, 0, SEEK_END);
                    long sz = ftell(fp);
                    rewind(fp);
                    g_html_cache.resize((size_t)sz);
                    fread(&g_html_cache[0], 1, (size_t)sz, fp);
                    fclose(fp);
                    ECO_INFO_NEW("[WebServer] loaded debug page from {} ({} bytes)",
                           paths[i], sz);
                    break;
                }
            }
            if (g_html_cache.empty()) {
                g_html_cache = "<html><body><h1>stark_node debug page not found</h1></body></html>";
            }
        }
        body = g_html_cache;
    }
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "%s%zu\r\n\r\n", kHttpOkHeader, body.size());
    std::string resp(hdr);
    resp += body;
    send_all(fd, resp.data(), resp.size());
}

/* ================================================================
 * PullLoop -- main event loop with select()
 * ================================================================ */

void WebServer::PullLoop()
{
    uint64_t last_push_us = 0;
    uint64_t local_count  = 0;
    uint64_t last_diag_us = 0;  /* for interval diagnostics */

    while (m_running.load(std::memory_order_acquire)) {

        /* build fd_set */
        fd_set rfds;
        FD_ZERO(&rfds);
        if (m_listen_fd >= 0) FD_SET(m_listen_fd, &rfds);
        int maxfd = m_listen_fd;

        {
            std::lock_guard<std::mutex> lk(m_clients_mutex);
            for (int fd : m_clients) {
                FD_SET(fd, &rfds);
                if (fd > maxfd) maxfd = fd;
            }
        }

        /* select timeout = remaining time until next push, capped at 20ms */
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000UL + (uint64_t)ts.tv_nsec / 1000UL;
        long remaining = (long)(m_push_period_us - (now_us - last_push_us));
        if (remaining < 0) remaining = 0;
        if (remaining > 20000) remaining = 20000;
        struct timeval tv = {0, remaining};
        int ret = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        /* --- accept new connections --- */
        if (ret > 0 && m_listen_fd >= 0 && FD_ISSET(m_listen_fd, &rfds)) {
            int client = accept(m_listen_fd, NULL, NULL);
            if (client >= 0) {
                /* read HTTP upgrade request */
                char rbuf[4096];
                ssize_t nr = recv(client, rbuf, sizeof(rbuf) - 1, 0);
                if (nr > 0) {
                    rbuf[nr] = '\0';
                    std::string raw(rbuf, (size_t)nr);
                    HttpReq req = parse_http_req(raw);

                    if (!req.key.empty()) {
                        /* WebSocket upgrade */
                        std::string resp = ws_handshake_resp(req.key);
                        if (send_all(client, resp.data(), resp.size())) {
                            std::lock_guard<std::mutex> lk(m_clients_mutex);
                            m_clients.push_back(client);
                            ECO_INFO_NEW("[WebServer] WS client #{} connected",
                                   m_clients.size());
                            goto next_iter;
                        }
                    }

                    /* plain HTTP -- serve debug page */
                    if (req.path == "/" || req.path == "/index.html") {
                        handle_http(client);
                    }
                }
                close(client);
            }
        }

        /* --- recv from WebSocket clients --- */
        if (ret > 0) {
            std::lock_guard<std::mutex> lk(m_clients_mutex);
            for (auto it = m_clients.begin(); it != m_clients.end(); ) {
                int fd = *it;
                if (FD_ISSET(fd, &rfds)) {
                    uint8_t rbuf[4096];
                    ssize_t nr = recv(fd, rbuf, sizeof(rbuf), 0);
                    if (nr <= 0) {
                        close(fd);
                        it = m_clients.erase(it);
                        ECO_INFO_NEW("[WebServer] WS client fd={} disconnected", fd);
                        continue;
                    }
                    std::string msg = ws_frame_decode(rbuf, (size_t)nr);
                    if (!msg.empty()) {
                        /* handle report start/stop directly */
                        if (msg.find("\"report_start\"") != std::string::npos) {
                            m_push_enabled.store(true);
                            ECO_INFO_NEW("[WebServer] report started");
                        } else if (msg.find("\"report_stop\"") != std::string::npos) {
                            m_push_enabled.store(false);
                            ECO_INFO_NEW("[WebServer] report paused");
                        } else if (msg.find("\"perf_reset\"") != std::string::npos) {
                            if (m_shm) __atomic_store_n(&m_shm->perf_reset_request, 1, __ATOMIC_RELEASE);
                            /* 同时清零逐帧 trace 统计区: min/avg/max 重新累计.
                             * 与写者并发的最坏影响 = 重置瞬间丢失/多出 1 个样本, 可忽略. */
                            if (m_trace_shm) {
                                memset(&m_trace_shm->up_seg1,  0, sizeof(trace_stat_t));
                                memset(&m_trace_shm->up_seg2,  0, sizeof(trace_stat_t));
                                memset(&m_trace_shm->up_total, 0, sizeof(trace_stat_t));
                                memset(&m_trace_shm->dn_seg1,  0, sizeof(trace_stat_t));
                                memset(&m_trace_shm->dn_seg2,  0, sizeof(trace_stat_t));
                                memset(&m_trace_shm->dn_total, 0, sizeof(trace_stat_t));
                            }
                            ECO_INFO_NEW("[WebServer] perf max reset requested");
                        } else {
                            dispatch_command(m_shm, m_motor_hal, msg, m_last_cmd);
                        }
                    }
                }
                ++it;
            }
        }

        /* --- data push (every 20ms ≈ 50Hz) --- */
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint64_t now_us = (uint64_t)ts.tv_sec * 1000000UL +
                              (uint64_t)ts.tv_nsec / 1000UL;

            if (now_us - last_push_us >= m_push_period_us &&
                m_push_enabled.load(std::memory_order_acquire)) {
                last_push_us = now_us;

                if (m_shm) {
                    /* build can_rx JSON fragment */
                    std::string can_rx_json;
                    {
                        std::lock_guard<std::mutex> lk(m_can_rx_mutex);
                        can_rx_json += ",\"can_rx_seq\":" + std::to_string(m_can_rx_seq) + ",";
                        can_rx_json += "\"can_rx\":[";
                        for (size_t i = 0; i < m_can_rx_buf.size(); i++) {
                            if (i) can_rx_json += ",";
                            /* escape JSON string */
                            std::string escaped;
                            for (char c : m_can_rx_buf[i]) {
                                if (c == '"') escaped += "\\\"";
                                else if (c == '\\') escaped += "\\\\";
                                else if (c == '\n') escaped += "\\n";
                                else if (c == '\r') escaped += "";
                                else escaped += c;
                            }
                            can_rx_json += "\"" + escaped + "\"";
                        }
                        can_rx_json += "]";
                    }
                    std::string trace_json = BuildTraceJson();
                    std::string json = serialize_to_json(m_shm, m_last_cmd, can_rx_json, trace_json);
                    std::string frame = ws_frame_text(json);

                    std::lock_guard<std::mutex> lk(m_clients_mutex);
                    for (auto it = m_clients.begin(); it != m_clients.end(); ) {
                        if (!send_all(*it, frame.data(), frame.size())) {
                            close(*it);
                            it = m_clients.erase(it);
                        } else {
                            m_frame_count++;
                            ++it;
                        }
                    }
                }

                local_count++;
                /* diagnostics: print actual push interval */
                {
                    uint64_t delta = (last_diag_us > 0) ? (now_us - last_diag_us) / 1000 : 0;
                    ECO_DEBUG_NEW("[WebServer] diag: push#{} interval={}ms (expected={}ms)",
                           local_count, delta, m_push_period_us / 1000);
                }
                last_diag_us = now_us;
                if (local_count % 500 == 0) { /* every ~10s */
                    ECO_DEBUG_NEW("[WebServer] push {} frames, {} clients",
                           m_frame_count, m_clients.size());
                }
            }
        }

next_iter:;
    }

    /* cleanup */
    for (int fd : m_clients) close(fd);
    m_clients.clear();
    if (m_listen_fd >= 0) { close(m_listen_fd); m_listen_fd = -1; }
}

}  /* namespace stark_periph_manager_node */
