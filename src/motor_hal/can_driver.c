/**
 * @file can_driver.c
 * @brief Linux SocketCAN (CANFD) 驱动封装
 *
 * SocketCAN RAW socket + FD 帧支持。
 */

#include "motor_hal_types.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>

/* =====================================================
 * 内部数据结构
 * ===================================================== */

typedef struct {
    char     iface[16];
    int      sock_fd;
    uint64_t tx_count;
    uint64_t rx_count;
    uint64_t tx_err;
    uint64_t rx_err;
} can_driver_t;

/* =====================================================
 * 公共接口
 * ===================================================== */

int can_driver_open(const char *iface, uint32_t arb_bitrate, uint32_t data_bitrate,
                    can_driver_t **out);

void can_driver_close(can_driver_t *drv);

int can_driver_send(can_driver_t *drv, const canfd_frame_t *frame);

int can_driver_recv(can_driver_t *drv, canfd_frame_t *frame, int timeout_ms);

int can_driver_set_filter(can_driver_t *drv, uint32_t can_id, uint32_t can_mask);

int can_driver_fd(can_driver_t *drv);

const char* can_driver_iface(can_driver_t *drv);

void can_driver_stats(can_driver_t *drv, uint64_t *tx, uint64_t *rx,
                      uint64_t *txe, uint64_t *rxe);

/* =====================================================
 * 实现
 * ===================================================== */

int can_driver_open(const char *iface, uint32_t arb_bitrate __attribute__((unused)), uint32_t data_bitrate __attribute__((unused)),
                    can_driver_t **out)
{
    *out = calloc(1, sizeof(can_driver_t));
    if (!*out) return -ENOMEM;

    can_driver_t *drv = *out;
    strncpy(drv->iface, iface, sizeof(drv->iface) - 1);
    drv->sock_fd = -1;

    /* 1. 创建 CAN_RAW socket */
    int fd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) { free(*out); *out = NULL; return -errno; }

    /* 2. 启用 CANFD */
    int enable = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
        int e = errno;
        close(fd); free(*out); *out = NULL;
        return -e;
    }

    /* 3. 错误帧过滤 */
    can_err_mask_t err_mask = CAN_ERR_MASK;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    /* 4. 获取接口索引 */
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        int e = errno;
        close(fd); free(*out); *out = NULL;
        return -e;
    }

    /* 5. 绑定 */
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        int e = errno;
        close(fd); free(*out); *out = NULL;
        return -e;
    }

    drv->sock_fd = fd;
    return 0;
}

void can_driver_close(can_driver_t *drv)
{
    if (!drv) return;
    if (drv->sock_fd >= 0) {
        close(drv->sock_fd);
        drv->sock_fd = -1;
    }
    free(drv);
}

int can_driver_send(can_driver_t *drv, const canfd_frame_t *frame)
{
    if (!drv || drv->sock_fd < 0) return -ENODEV;

    struct canfd_frame cfd;
    memset(&cfd, 0, sizeof(cfd));
    /* 11-bit 标准帧: 不加 CAN_EFF_FLAG */
    cfd.can_id = frame->id & CAN_SFF_MASK;
    if (frame->is_fd && frame->use_brs) cfd.flags |= CANFD_BRS;
    cfd.len = frame->dlc;
    memcpy(cfd.data, frame->data, frame->dlc);

    int n = write(drv->sock_fd, &cfd, sizeof(cfd));
    if (n < 0) { drv->tx_err++; return -errno; }
    drv->tx_count++;
    return n;
}

int can_driver_recv(can_driver_t *drv, canfd_frame_t *frame, int timeout_ms)
{
    if (!drv || drv->sock_fd < 0) return -ENODEV;

    /* select 实现超时 */
    if (timeout_ms >= 0) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(drv->sock_fd, &fds);
        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        int ret = select(drv->sock_fd + 1, &fds, NULL, NULL, &tv);
        if (ret == 0) return 0;
        if (ret < 0)  return -errno;
    }

    struct canfd_frame cfd;
    memset(&cfd, 0, sizeof(cfd));
    int n = read(drv->sock_fd, &cfd, sizeof(cfd));
    if (n < 0) { drv->rx_err++; return -errno; }

    frame->id    = cfd.can_id & CAN_SFF_MASK;
    frame->dlc   = cfd.len;
    frame->is_fd = (cfd.flags & CANFD_BRS) != 0;
    memset(frame->data, 0, CANFD_MAX_DLC);
    memcpy(frame->data, cfd.data, cfd.len);

    drv->rx_count++;
    return n;
}

int can_driver_set_filter(can_driver_t *drv, uint32_t can_id, uint32_t can_mask)
{
    if (!drv || drv->sock_fd < 0) return -ENODEV;
    struct can_filter filter;
    /* 11-bit 标准帧过滤: mask 加 EFF_FLAG 确保只匹配标准帧 */
    filter.can_id   = can_id & CAN_SFF_MASK;
    filter.can_mask = (can_mask & CAN_SFF_MASK) | CAN_EFF_FLAG;
    return setsockopt(drv->sock_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                      &filter, sizeof(filter));
}

int can_driver_fd(can_driver_t *drv)
{
    if (!drv) return -1;
    return drv->sock_fd;
}

const char* can_driver_iface(can_driver_t *drv)
{
    return drv ? drv->iface : NULL;
}

void can_driver_stats(can_driver_t *drv, uint64_t *tx, uint64_t *rx,
                      uint64_t *txe, uint64_t *rxe)
{
    if (!drv) return;
    if (tx)  *tx  = drv->tx_count;
    if (rx)  *rx  = drv->rx_count;
    if (txe) *txe = drv->tx_err;
    if (rxe) *rxe = drv->rx_err;
}
