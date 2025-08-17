/*
* ZHIYUAN USBCANFD device driver
* Copyright (C) Guangzhou ZHIYUAN Electronics Co., Ltd. All rights reserved.
*/
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/crc16.h>

#include "usbcanfd.h"
#define DEVICE_ALIAS "NIO_USBCANFD200U_400U_VERS_1.7"
#define USBCANFD_MODULE_INFO  "zhiyuan usbcanfd"
#define USBCANFD_DRIVER_NAME  "usbcanfd"
#define USBCANFD_DRIVER_VERS  "2.10"
#define USBCANFD_USB_MAX_RX_URBS  16
static const struct usb_device_id usbcanfd_id_table[] = {
    { USB_DEVICE(UCFD_VID, UCFD_PID) },
    { USB_DEVICE(0x3068, 0x0009) },
    { 0 }
};
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(USBCANFD_MODULE_INFO);
MODULE_VERSION(USBCANFD_DRIVER_VERS);
MODULE_DEVICE_TABLE(usb, usbcanfd_id_table);
MODULE_ALIAS(DEVICE_ALIAS);

static unsigned dbg = 0;
module_param(dbg, uint, 0644);
#define _DBG_(fmt, args...)  if (dbg) { printk(KERN_ERR USBCANFD_DRIVER_NAME ": %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##args); }
#define _MSG_(fmt, args...)  printk(KERN_ERR USBCANFD_DRIVER_NAME ": %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##args)
#define _INF_(fmt, args...)  printk(KERN_INFO USBCANFD_DRIVER_NAME ": %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ##args)

static unsigned cfg_term_res = 1;
static unsigned cfg_rx_urbs = USBCANFD_USB_MAX_RX_URBS;
static unsigned cfg_rx_size = 0x4000;
static unsigned cfg_cmd_timeout = 500;
static unsigned cfg_tx_timeout = 20;
static unsigned cfg_lin_master = 0;
static unsigned cfg_lin_mode = 0;
module_param(cfg_term_res, uint, 0644);
module_param(cfg_rx_urbs, uint, 0644);
module_param(cfg_rx_size, uint, 0644);
module_param(cfg_cmd_timeout, uint, 0644);
module_param(cfg_tx_timeout, uint, 0644);
module_param(cfg_lin_master, uint, 0644);
module_param(cfg_lin_mode, uint, 0644);

#define CAN_TX_FIFO_DEPTH  10240
#define LIN_TX_FIFO_DEPTH  10240
#define CAN_TX_FIFO_MASK  (CAN_TX_FIFO_DEPTH-1)
#define LIN_TX_FIFO_MASK  (LIN_TX_FIFO_DEPTH-1)

typedef enum {
    CAN_PORT = 0,
    LIN_PORT = 1,
} port_type;

typedef struct {
    int echo;
    bool is_canfd;
    bool if_loopback;
    union {
        struct can_frame can;
        struct canfd_frame canfd;
    };
} usbcanfd_can_frame;

typedef struct {
    int echo;
    u32 id;
    u8 len;
    u8 dat[8];
} usbcanfd_lin_frame;

typedef struct {
    u32 w;
    u32 r;
    usbcanfd_can_frame dat[CAN_TX_FIFO_DEPTH];
} usbcanfd_can_tx_fifo;

typedef struct {
    u32 w;
    u32 r;
    usbcanfd_lin_frame dat[LIN_TX_FIFO_DEPTH];
} usbcanfd_lin_tx_fifo;

typedef struct usbcanfd_port_t {
    struct can_priv can;
    struct net_device *ndev;
    struct usbcanfd_device_t *parent;
    struct can_berr_counter bec;
    unsigned index;
    bool ndev_rdy;
    port_type type;
} usbcanfd_port;

typedef struct usbcanfd_can_port_t {
    usbcanfd_port port;
    usbcanfd_can_tx_fifo tx_fifo;
} usbcanfd_can_port;

typedef struct usbcanfd_lin_port_t {
    usbcanfd_port port;
    usbcanfd_lin_tx_fifo tx_fifo;
    char cfg[4096];
} usbcanfd_lin_port;

typedef struct usbcanfd_device_t {
    usbcanfd_port *can_ports[UCFD_CAN_PORTS];
    usbcanfd_port *lin_ports[UCFD_LIN_PORTS];
    struct usb_device *udev;
    struct mutex cmd_lock;
    struct usb_anchor rx_anchor;
    struct task_struct *tx_thread;
    struct {
        wait_queue_head_t wait;
        atomic_t pend;
    } tx_signal;
    bool stop;
    int open_count;
    bool rx_running;
    int can_clk;
    UCFD_CMD cmd;
    UCFD_ACK ack;
    UCFD_TXCMD txcmd;
    UCFD_TXACK txack;
    unsigned n_can_ports;
    unsigned n_lin_ports;
    unsigned index;
    unsigned max_tx_can_cnt;
    unsigned max_packet_size;
    s64 ts_delta;
    void *rxbuf[USBCANFD_USB_MAX_RX_URBS];
    dma_addr_t rxbuf_dma[USBCANFD_USB_MAX_RX_URBS];
    char sn[21];
} usbcanfd_device;

#define UCFD_SIG_INIT(s)    init_waitqueue_head(&(s)->wait)
#define UCFD_SIG_PEND(s)    atomic_xchg(&(s)->pend, 1)
#define UCFD_SIG_WAIT(s,t)  wait_event_interruptible_timeout((s)->wait, !atomic_read(&(s)->pend), (t))
#define UCFD_SIG_WAKE(s)    atomic_xchg(&(s)->pend, 0),wake_up_interruptible(&(s)->wait)

#define IOCTL_LIN_CTL  _IOWR(100, 0, void*)

static int usbcanfd_device_reset(struct usb_device *dev) {
    int err = 0;
    u16 *buf = kmalloc(2, GFP_KERNEL);
    if (!buf) {
        return 0;
    }

    err = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), 0xFF, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, 0, 0, (void *)buf, 2, USB_CTRL_GET_TIMEOUT);
    if (err < 0) {
        _MSG_("usbcanfd reset failed, %d", err);
    } else {
        _INF_("usbcanfd reset success!");
    }

    kfree(buf);

    return err;
}

static int usbcanfd_chkack(usbcanfd_device *d, u32 len)
{
    UCFD_CMD *cmd = &d->cmd;
    UCFD_ACK *ack = &d->ack;
    if (ack->cmd != cmd->cmd) {
        _MSG_("ack.id(0x%02x) != cmd.id(0x%02x)", ack->cmd, cmd->cmd);
        return -1;
    }
    if (ack->idx != cmd->idx) {
        _MSG_("ack.idx(0x%02x) != cmd.idx(0x%02x)", ack->idx, cmd->idx);
        return -1;
    }
    if (ack->err) {
        _MSG_("ack.err(0x%02x) != 0", ack->err);
        return -1;
    }
    if (ack->len != len) {
        _MSG_("ack.len(0x%02x) != 0x%02x", ack->len, len);
        return -1;
    }
    return 0;
}

static int usbcanfd_cmd(usbcanfd_device *d)
{
    UCFD_CMD *cmd = &d->cmd;
    UCFD_ACK *ack = &d->ack;
    int ret = -1;
    u32 len = 0;
    u16 crc;
    u8 i = 0;
    u8 err = 0;
    _DBG_("+++ cmd=0x%02x, len=%d", cmd->cmd, cmd->len);
    for (i = 0; i < 3; i++) {
        do {
            crc = crc16(0xffff, (u8*)cmd, CMD_HDR_SIZE + cmd->len);
            SET_U16(cmd->dat + cmd->len, crc);
            ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_CMD),
                &cmd->hdr, CMD_HDR_SIZE, &len, cfg_cmd_timeout);
            if (ret < 0) {
                _MSG_("usb_bulk_msg(cmd.hdr) failed: ret=%d", ret);
                break;
            }
            ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_CMD),
                cmd->dat, CMD_DAT_SIZE, &len, cfg_cmd_timeout);
            if (ret < 0) {
                _MSG_("usb_bulk_msg(cmd.dat) failed: ret=%d", ret);
                //break;
            }
            ret = usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_ACK),
                &ack->hdr, ACK_HDR_SIZE, &len, cfg_cmd_timeout);
            if (ret < 0) {
                _MSG_("usb_bulk_msg(ack.hdr) failed: ret=%d", ret);
                break;
            }
            ret = usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_ACK),
                ack->dat, ACK_DAT_SIZE, &len, cfg_cmd_timeout);
            if (ret < 0) {
                _MSG_("usb_bulk_msg(ack.dat) failed: ret=%d", ret);
                break;
            }
            ret = len;
        } while (0);
        if (ret >= 0) {
            break;
        } else {
            err = 1;
        }
    }

    if (err) {
        _MSG_("[%s] cmd(%02X) retry=%d, len=%d, ret=%d", d->sn, cmd->cmd, i, cmd->len, ret);
    }

    if (ret < 0) {
        _MSG_("[%s] cmd(%02X) len=%d failed: ret=%d", d->sn, cmd->cmd, cmd->len, ret);
    }
    _DBG_("--- ret=%d", ret);
    return ret;
}

static int usbcanfd_tx(usbcanfd_device *d)
{
    UCFD_TXCMD *cmd = &d->txcmd;
    UCFD_TXACK *ack = &d->txack;
    int ret = -1;
    u32 len = 0;
    u16 crc;
    _DBG_("+++");
    do {
        crc = crc16(0xffff, (u8*)cmd, CMD_HDR_SIZE + cmd->len);
        SET_U16(cmd->dat + cmd->len, crc);
        ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_TXCMD),
            cmd, CMD_HDR_SIZE + cmd->len + CRC_SIZE, &len, 1000);
        if (ret < 0) {
            _DBG_("usb_bulk_msg(cmd.hdr) failed: ret=%d", ret);
            break;
        }
        ret = usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_TXACK),
            ack, sizeof(UCFD_TXACK), &len, 1000 + cfg_tx_timeout);
        if (ret < 0) {
            _DBG_("usb_bulk_msg(ack) failed: ret=%d", ret);
            break;
        }
        if (ack->hdr != CMD_HDR || ack->cmd != cmd->cmd || ack->err) {
            _DBG_("ack error");
            ret = -1;
            break;
        }
        ret = 0;
    } while (0);
    _DBG_("--- ret=%d", ret);
    return ret;
}

/*
 * 硬件时间戳加偏移得到当前帧系统时间戳
 */
static void usbcanfd_get_lin_ts(usbcanfd_device *d, struct sk_buff *skb, u64 us)
{
    struct skb_shared_hwtstamps *ts = skb_hwtstamps(skb);
    s64 ns = us * NSEC_PER_USEC;
    if (!d->ts_delta) {
        d->ts_delta = ktime_get_real_ns() - ns;
        _DBG_("ts_delta=0x%016llx", d->ts_delta);
    }
    ns += d->ts_delta;
    ts->hwtstamp = ns_to_ktime(ns);
}

/*
 * 硬件时间戳加偏移得到当前帧系统时间戳
 */
static void usbcanfd_get_can_ts(usbcanfd_device *d, struct sk_buff *skb, const UCFD_CAN_MSG_HDR *hdr)
{
    struct skb_shared_hwtstamps *ts = skb_hwtstamps(skb);
    s64 ns = 0;
    s64 us = 0;
    if (hdr->inf.ts_us) {
        us = hdr->ts | ((u64)hdr->pad << 32);
        us += (u64)hdr->inf.ts_ov_cnt << 48;
        _DBG_("ts unit us, ts=%u", hdr->ts);
    } else {
        us = (u64)hdr->ts | ((u64)hdr->inf.ts_ov_cnt << 32);
        us *= 100;
        _DBG_("ts unit 100us, ts=%u", hdr->ts);
    }
    ns = us * NSEC_PER_USEC;
    if (!d->ts_delta) {
        d->ts_delta = ktime_get_real_ns() - ns;
        _DBG_("ts_delta=0x%016llx", d->ts_delta);
    }
    ns += d->ts_delta;
    ts->hwtstamp = ns_to_ktime(ns);
}

static void usbcanfd_upload_callback(struct urb *urb)
{
    usbcanfd_device *d = urb->context;
    UCFD_CMD *pkt  = (UCFD_CMD*)urb->transfer_buffer;
    UCFD_XFER *xfer = (UCFD_XFER*)pkt->dat;
    UCFD_20_MSG *msg20 = xfer->msgs.can20;
    UCFD_FD_MSG *msgfd = xfer->msgs.canfd;
    UCFD_ERR_MSG *msgerr = xfer->msgs.canerr;
    UCFD_LIN_MSG *msglin = xfer->msgs.lin;
    usbcanfd_port *can_port = NULL;
    usbcanfd_port *lin_port = NULL;
    struct net_device *ndev = NULL;
    struct net_device_stats *stats = NULL;
    struct sk_buff *skb = NULL;
    struct canfd_frame *f = NULL;
    bool canfd = false;
    bool canerr = false;
    bool lin = false;
    bool msg_canfd = false;
    int ret;
    u16 i;

    switch (urb->status) {
    case -ENOENT:
    case -EPIPE:
    case -EPROTO:
        _MSG_("urb cancelled: status=%d", urb->status);
        return;
    case -ESHUTDOWN:
        _DBG_("urb cancelled: status=%d", urb->status);
        d->stop = true;
        return;
    default: break;
    }

    if ((pkt->hdr != CMD_HDR) ||
        (pkt->cmd != XFER_CANFD && pkt->cmd != XFER_CAN20 && pkt->cmd != XFER_CANERR && pkt->cmd != XFER_LIN) ||
        (pkt->len > (cfg_rx_size - CMD_HDR_SIZE - CRC_SIZE)))
    {
        _MSG_("usb packet error");
        return;
    }
    canfd = pkt->cmd == XFER_CANFD;
    canerr = pkt->cmd == XFER_CANERR;
    lin = pkt->cmd == XFER_LIN;

    _DBG_("urb.len=%d, frames.cnt=%d, type=%s", urb->actual_length, xfer->cnt,
        canerr ? "canerr" : (canfd ? "canfd" : (lin ? "lin" : "can20")));

    for (i = 0; i < xfer->cnt; i++, msg20++, msgfd++, msgerr++, msglin++) {
        if (lin) {
            if (msglin->hdr.chn >= d->n_lin_ports) continue;
            lin_port = d->lin_ports[msglin->hdr.chn];
        } else if (canerr) {
            if (msgerr->hdr.chn >= d->n_can_ports) continue;
            can_port = d->can_ports[msgerr->hdr.chn];
        } else if (canfd) {
            if (msgfd->hdr.chn >= d->n_can_ports) continue;
            can_port = d->can_ports[msgfd->hdr.chn];
            msg_canfd = msgfd->hdr.inf.fmt ? true :false;
        } else {
            if (msg20->hdr.chn >= d->n_can_ports) continue;
            can_port = d->can_ports[msg20->hdr.chn];
            msg_canfd = msg20->hdr.inf.fmt ? true :false;
        }

        ndev = lin ? lin_port->ndev : can_port->ndev;
        stats = &ndev->stats;

        if (!netif_device_present(ndev)) {
            _MSG_("!netif_device_present");
            continue;
        }

        skb = !canerr ? (msg_canfd ? alloc_canfd_skb(ndev, &f) : alloc_can_skb(ndev, (struct can_frame**)&f)) : alloc_can_err_skb(ndev, (struct can_frame**)&f);
        if (!skb) {
            _MSG_("alloc_xxx_skb failed");
            continue;
        }

        if (lin) {
            u8 *p = f->data;
            usbcanfd_get_lin_ts(d, skb, msglin->hdr.ts);
            f->flags = 0;
            f->can_id = msglin->hdr.pid & 0x3f;
            if (cfg_lin_mode == 0) {
                f->can_id |= (u32)(msglin->hdr.inf.val) << 8;
            }
            if (msglin->hdr.inf.val > 0x3FF) {
                f->can_id |= CAN_EFF_FLAG;
            }
            f->len = msglin->hdr.len;
            memcpy(f->data, msglin->dat, msglin->hdr.len);
            _DBG_("can%d.lin: id=%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x", msglin->hdr.chn, f->can_id, f->len, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        } else if (canerr) {
            usbcanfd_get_can_ts(d, skb, &msgerr->hdr);
            f->can_id = CAN_ERR_FLAG;
            f->can_id |= CAN_ERR_BUSERROR;
            f->len = 0;
            can_port->can.can_stats.bus_error++;
        } else if (canfd) {
            u8 *p = f->data;
            usbcanfd_get_can_ts(d, skb, &msgfd->hdr);
            if (msgfd->hdr.inf.brs) f->flags |= CANFD_BRS;
            if (msgfd->hdr.inf.est) f->flags |= CANFD_ESI;
            f->can_id = msgfd->hdr.id;
            if (msgfd->hdr.inf.sdf) f->can_id |= CAN_RTR_FLAG;
            if (msgfd->hdr.inf.sef) f->can_id |= CAN_EFF_FLAG;
            f->len = msgfd->hdr.len;
            memcpy(f->data, msgfd->dat, msgfd->hdr.len);
            _DBG_("can%d.canfd: id=%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x", msgfd->hdr.chn, f->can_id, f->len, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        } else {
            u8 *p = f->data;
            usbcanfd_get_can_ts(d, skb, &msg20->hdr);
            f->can_id = msg20->hdr.id;
            if (msg20->hdr.inf.brs) f->flags |= CANFD_BRS;
            if (msg20->hdr.inf.est) f->flags |= CANFD_ESI;
            if (msg20->hdr.inf.sdf) f->can_id |= CAN_RTR_FLAG;
            if (msg20->hdr.inf.sef) f->can_id |= CAN_EFF_FLAG;
            f->len = msg20->hdr.len;
            memcpy(f->data, msg20->dat, msg20->hdr.len);
            _DBG_("can%d.can20: id=%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x", msg20->hdr.chn, f->can_id, f->len, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        }

        stats->rx_packets++;
        stats->rx_bytes += f->len;
        netif_rx(skb);
    }

    usb_fill_bulk_urb(urb, d->udev, usb_rcvbulkpipe(d->udev, EP_RX),
        urb->transfer_buffer, cfg_rx_size, usbcanfd_upload_callback, d);
    ret = usb_submit_urb(urb, GFP_ATOMIC);
    if (ret) {
        _MSG_("usb_submit_urb failed: ret=%d", ret);
        if (ret == -ENODEV) {
            _MSG_("netif_device_detach");
            for (i = 0; i < d->n_can_ports; i++) {
                netif_device_detach(d->can_ports[i]->ndev);
            }
            for (i = 0; i < d->n_lin_ports; i++) {
                netif_device_detach(d->lin_ports[i]->ndev);
            }
        }
    }
}

static int usbcanfd_cmd_open(usbcanfd_port *port)
{
    usbcanfd_device *d = port->parent;
    int ret = -1;
    UCFD_CMD *cmd = &d->cmd;
    UCFD_CAN_INIT *init = (UCFD_CAN_INIT*)cmd->dat;
    struct can_bittiming *abt = &port->can.bittiming;
    struct can_bittiming *dbt = &port->can.data_bittiming;
    bool fd = !!(port->can.ctrlmode & CAN_CTRLMODE_FD);
    _DBG_("+++");
    mutex_lock(&d->cmd_lock);
    do {
        if (d->open_count == 0) {
            UCFD_CMD_INIT(cmd, CMD_OPEN, 0, 0);
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _MSG_("usbcanfd_cmd(CMD_OPEN) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_OPEN) succeeded");
        }
        d->open_count++;

        if (port->type == CAN_PORT) {
            _MSG_("abt: tseg1=%d, tseg2=%d, sjw=%d, brp=%d, dbt: tseg1=%d, tseg2=%d, sjw=%d, brp=%d",
                abt->phase_seg1 + abt->prop_seg, abt->phase_seg2, abt->sjw, abt->brp,
                dbt->phase_seg1 + dbt->prop_seg, dbt->phase_seg2, dbt->sjw, dbt->brp);

            if (!fd) dbt = abt;

            UCFD_CMD_INIT(cmd, CMD_CAN_INIT, port->index, sizeof(UCFD_CAN_INIT));
            init->clk = d->can_clk;
            init->aset.tseg1 = abt->phase_seg1 + abt->prop_seg - 1;
            init->aset.tseg2 = abt->phase_seg2 - 1;
            init->aset.sjw = abt->sjw - 1;
            init->aset.smp = 0;
            init->aset.brp = abt->brp - 1;
            init->dset.tseg1 = dbt->phase_seg1 + dbt->prop_seg - 1;
            init->dset.tseg2 = dbt->phase_seg2 - 1;
            init->dset.sjw = dbt->sjw - 1;
            init->dset.smp = 0;
            init->dset.brp = dbt->brp - 1;
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _MSG_("usbcanfd_cmd(CMD_CAN_INIT) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_CAN_INIT) succeeded");

            cfg_tx_timeout = min(4000u, cfg_tx_timeout);
            cfg_tx_timeout = max(1u, cfg_tx_timeout);
            UCFD_CMD_INIT(cmd, CMD_CAN_TX_TIMEOUT, 0, 4);
            *(u32*)(cmd->dat) = cfg_tx_timeout;
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _MSG_("usbcanfd_cmd(CMD_CAN_TX_TIMEOUT) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_CAN_TX_TIMEOUT) succeeded");

            UCFD_CMD_INIT(cmd, CMD_CAN_TRES, port->index, 1);
            cmd->dat[0] = cfg_term_res ? 0x55 : 0xaa;
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _MSG_("usbcanfd_cmd(CMD_CAN_TRES) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_CAN_TRES) succeeded");

            UCFD_CMD_INIT(cmd, CMD_CAN_START, port->index, 0);
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _MSG_("usbcanfd_cmd(CMD_CAN_START) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_CAN_START) succeeded");
        } else {
            usbcanfd_lin_port *lin = (usbcanfd_lin_port*)port;
            _DBG_("lin%d.cfg: %s", port->index, lin->cfg);
            UCFD_CMD_INIT(cmd, CMD_LIN_INIT, port->index, strlen(lin->cfg));
            memcpy(cmd->dat, lin->cfg, strlen(lin->cfg));
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _MSG_("usbcanfd_cmd(CMD_LIN_INIT) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_LIN_INIT) succeeded");
        }
    } while (0);
    mutex_unlock(&d->cmd_lock);
    _DBG_("--- ret=%d", ret);
    return 0;
}

static int usbcanfd_cmd_stop(usbcanfd_port *port, bool close)
{
    usbcanfd_device *d = port->parent;
    int ret = -1;
    UCFD_CMD *cmd = &d->cmd;
    _DBG_("+++");
    mutex_lock(&d->cmd_lock);
    do {
        if (port->type == CAN_PORT) {
            UCFD_CMD_INIT(cmd, CMD_CAN_STOP, port->index, 0);
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _DBG_("usbcanfd_cmd(CMD_CAN_STOP) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_CAN_STOP) succeeded");
            ret = 0;
        } else {
            char *s = (char*)cmd->dat;
            int len = sprintf(s, "{\"LIN%d\":{\"Enable\":0}}", port->index);
            UCFD_CMD_INIT(cmd, CMD_LIN_INIT, port->index, len);
            sprintf(s, "{\"LIN%d\":{\"Enable\":0}}", port->index);
            ret = usbcanfd_cmd(d);
            if (ret < 2 || usbcanfd_chkack(d, 0)) {
                _DBG_("usbcanfd_cmd(CMD_LIN_INIT) failed");
                break;
            }
            _DBG_("usbcanfd_cmd(CMD_LIN_INIT) succeeded");
            ret = 0;
        }

        if (!close) break;

        UCFD_CMD_INIT(cmd, CMD_CLOSE, 0, 0);
        ret = usbcanfd_cmd(d);
        if (ret < 2 || usbcanfd_chkack(d, 0)) {
            _DBG_("usbcanfd_cmd(CMD_CLOSE) failed");
            break;
        }
        _DBG_("usbcanfd_cmd(CMD_CLOSE) succeeded");
        ret = 0;
    } while (0);
    mutex_unlock(&d->cmd_lock);
    _DBG_("--- ret=%d", ret);
    return ret;
}

static int usbcanfd_start(usbcanfd_device *d, usbcanfd_port *port)
{
    int err, i;
    for (i = 0; !d->rx_running && i < cfg_rx_urbs; i++) {
        struct urb *urb = NULL;
        u8 *buf = NULL;
        _DBG_("initializing urb[%d] ...", i);
        do {
            err = -ENOMEM;
            urb = usb_alloc_urb(0, GFP_KERNEL);
            if (!urb) {
                _MSG_("usb_alloc_urb failed");
                break;
            }
            buf = usb_alloc_coherent(d->udev, cfg_rx_size, GFP_ATOMIC, &urb->transfer_dma);
            if (!buf) {
                _MSG_("usb_alloc_coherent failed");
                break;
            }
            usb_fill_bulk_urb(urb, d->udev, usb_rcvbulkpipe(d->udev, EP_RX),
                buf, cfg_rx_size, usbcanfd_upload_callback, d);
            urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
            usb_anchor_urb(urb, &d->rx_anchor);
            err = usb_submit_urb(urb, GFP_KERNEL);
            _DBG_("usb_submit_urb(urb[%d]): err=%d", i, err);
            if (err) {
                _MSG_("usb_submit_urb(urb[%d]) failed: err=%d", i, err);
            }
        } while (0);
        if (!err) {
            usb_free_urb(urb);
            d->rxbuf[i] = buf;
            d->rxbuf_dma[i] = urb->transfer_dma;
            continue;
        }
        if (buf) {
            usb_unanchor_urb(urb);
            usb_free_coherent(d->udev, cfg_rx_size, buf, urb->transfer_dma);
        }
        if (urb) usb_free_urb(urb);
    }
    if (!d->rx_running && i < cfg_rx_urbs)
        _MSG_("rx_urbs(%d) < cfg_rx_urbs(%d)", i, cfg_rx_urbs);
    d->rx_running = true;
    err = usbcanfd_cmd_open(port);
    if (err) {
        _MSG_("cmd(open) failed: err=%d", err);
        return -EFAULT;
    }
    port->can.state = CAN_STATE_ERROR_ACTIVE;
    return 0;
}

static int usbcanfd_open(struct net_device *ndev)
{
    usbcanfd_port *port = netdev_priv(ndev);
    usbcanfd_device *d = port->parent;
    int err;
    _DBG_("*** open_count=%d, port=%d", d->open_count, port->index);
    err = open_candev(ndev);
    if (err) {
        _MSG_("open_candev(port%d) failed: err=%d", port->index, err);
        return err;
    }
    err = usbcanfd_start(d, port);
    if (err) {
        _MSG_("usbcanfd_start(port%d) failed: err=%d", port->index, err);
        if (err == -ENODEV) {
            _MSG_("netif_device_detach(port%d)", port->index);
            netif_device_detach(port->ndev);
        }
        close_candev(ndev);
        return err;
    }
    _DBG_("netif_start_queue(port%d)", port->index);
    netif_start_queue(ndev);
    return 0;
}

static int usbcanfd_stop(struct net_device *ndev)
{
    usbcanfd_port *port = netdev_priv(ndev);
    usbcanfd_device *d = port->parent;
    int err;
    d->open_count--;
    _DBG_("*** open_count=%d, port=%d", d->open_count, port->index);

    err = usbcanfd_cmd_stop(port, !d->open_count);
    if (err) { _DBG_("usbcanfd_cmd_stop failed: err=%d", err); }

    port->can.state = CAN_STATE_STOPPED;
    netif_stop_queue(port->ndev);
    close_candev(port->ndev);
    _DBG_("netif_stop_queue(port%d)", port->index);
    return 0;
}

static u32 usbcanfd_can_tx(usbcanfd_device *d, usbcanfd_port *port)
{
    struct net_device_stats *stats = &port->ndev->stats;
    usbcanfd_can_tx_fifo *fifo = &((usbcanfd_can_port*)port)->tx_fifo;
    u32 i, cnt = min(fifo->w - fifo->r, d->max_tx_can_cnt);

    if (cnt) {
        UCFD_TXCMD *cmd = &d->txcmd;
        UCFD_XFER *xfer = (UCFD_XFER*)cmd->dat;
        UCFD_FD_MSG *msg = xfer->msgs.canfd;
        UCFD_CAN_TX_INIT(cmd, port->index, cnt);
        _DBG_("cnt=%u", cnt);
        for (i = 0; i < cnt; i++, msg++, fifo->r++) {
            u32 pos = fifo->r & CAN_TX_FIFO_MASK;
            usbcanfd_can_frame *p = &fifo->dat[pos];
            struct canfd_frame *f = &p->canfd;
            u8 *t = f->data;

            _DBG_("+++ can%d: idx=%d, fd=%d, id=0x%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x",
                port->index, i, p->is_canfd, f->can_id, f->len, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]);

            msg->hdr.id = f->can_id;
            msg->hdr.chn = port->index;
            msg->hdr.len = f->len;
            msg->hdr.inf.txm = (0 && p->if_loopback) ? UCFD_SR_NORM : UCFD_TX_NORM;
            msg->hdr.inf.fmt = p->is_canfd;
            msg->hdr.inf.sdf = !!(f->can_id & CAN_RTR_FLAG);
            msg->hdr.inf.sef = !!(f->can_id & CAN_EFF_FLAG);
            msg->hdr.inf.brs = p->is_canfd && (f->flags & CANFD_BRS);
            msg->hdr.inf.est = p->is_canfd && (f->flags & CANFD_ESI);
            memcpy(msg->dat, f->data, f->len);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5,12,0))
            stats->tx_bytes += can_get_echo_skb(port->ndev, pos, NULL);
#else
            stats->tx_bytes += can_get_echo_skb(port->ndev, pos);
#endif
            stats->tx_packets++;
        }
        if (usbcanfd_tx(d)) {
            _MSG_("usbcanfd_tx failed");
        } else {
            _DBG_("usbcanfd_tx succeeded");
        }
        if (port->can.state != CAN_STATE_STOPPED) {
            netif_wake_queue(port->ndev);
        }
    }

    return fifo->w - fifo->r;
}

static u32 usbcanfd_lin_tx(usbcanfd_device *d, usbcanfd_port *port)
{
    struct net_device_stats *stats = &port->ndev->stats;
    usbcanfd_lin_tx_fifo *fifo = &((usbcanfd_lin_port*)port)->tx_fifo;
    u32 i, cnt = min(fifo->w - fifo->r, d->max_tx_can_cnt);

    if (cnt) {
        UCFD_TXCMD *cmd = &d->txcmd;
        UCFD_XFER *xfer = (UCFD_XFER*)cmd->dat;
        UCFD_LIN_MSG *msg = xfer->msgs.lin;
        UCFD_LIN_TX_INIT(cmd, port->index, cnt);
        _DBG_("cnt=%u", cnt);
        for (i = 0; i < cnt; i++, msg++, fifo->r++) {
            u32 pos = fifo->r & LIN_TX_FIFO_MASK;
            usbcanfd_lin_frame *p = &fifo->dat[pos];
            u8 *t = p->dat;

            _DBG_("+++ lin%d: idx=%d, id=0x%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x",
                port->index, i, p->id, p->len, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]);

            msg->hdr.ts = 0;
            msg->hdr.chn = port->index;
            if (cfg_lin_mode == 0) {
                msg->hdr.pid = p->id & 0xff;
                msg->hdr.inf.txm = (p->id >> 8) & 3;
                msg->hdr.inf.ckm = (p->id >> 10) & 3;
            } else {
                bool master = cfg_lin_master & (1 << port->index);
                msg->hdr.pid = (p->id & 0xff) + (master ? 0 : 0x40);
                msg->hdr.inf.txm = master ? ((p->id >> 8) & 3) : 1;
                msg->hdr.inf.ckm = 0;
            }
            msg->hdr.inf.wk0 = (p->id >> 12) & 1;
            msg->hdr.inf.rs0 = 0;
            msg->hdr.pad = 0;
            msg->hdr.cks = 0;
            msg->hdr.len = p->len;
            memcpy(msg->dat, p->dat, p->len);
            if (msg->hdr.inf.wk0) {
                msg->hdr.ts = 1000; //wakeup time, unit:us
                msg->hdr.pad = 250; //keep high time after wakeup 
	    }

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5,12,0))
            stats->tx_bytes += can_get_echo_skb(port->ndev, pos, NULL);
#else
            stats->tx_bytes += can_get_echo_skb(port->ndev, pos);
#endif
            stats->tx_packets++;
        }
        if (usbcanfd_tx(d)) {
            _MSG_("usbcanfd_tx failed");
        } else {
            _DBG_("usbcanfd_tx succeeded");
        }
        if (port->can.state != CAN_STATE_STOPPED) {
            netif_wake_queue(port->ndev);
        }
    }

    return fifo->w - fifo->r;
}

static int usbcanfd_tx_thread(void *arg)
{
    usbcanfd_device *d = (usbcanfd_device*)arg;
    u32 left;
    int i;
    _DBG_("+++");
    while (!d->stop) {
        left = 0;
        UCFD_SIG_PEND(&d->tx_signal);
        for (i = 0; i < d->n_can_ports; i++) {
            left += usbcanfd_can_tx(d, d->can_ports[i]);
        }
        for (i = 0; i < d->n_lin_ports; i++) {
            left += usbcanfd_lin_tx(d, d->lin_ports[i]);
        }
        if (!left) {
            UCFD_SIG_WAIT(&d->tx_signal, 100);
        }
    }
    _DBG_("---");
    return 0;
}

static netdev_tx_t usbcanfd_can_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    usbcanfd_port *port = netdev_priv(ndev);
    usbcanfd_device *d = port->parent;
    usbcanfd_can_tx_fifo *fifo = &((usbcanfd_can_port*)port)->tx_fifo;
    netdev_tx_t ret = NETDEV_TX_OK;
    bool loopback = !!(port->can.ctrlmode & CAN_CTRLMODE_LOOPBACK);
    bool fd = can_is_canfd_skb(skb);
    int size = fd ? sizeof(struct canfd_frame) : sizeof(struct can_frame);
    struct canfd_frame *src = (struct canfd_frame *)skb->data;
    usbcanfd_can_frame *dst;
    u32 filled, space;
    u8 *p = src->data;
    _DBG_("+++ can%d: fd=%d, id=0x%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x",
        port->index, fd, src->can_id, src->len, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
    do {
        if (d->stop) {
            _DBG_("netif_stop_queue: usb err");
            netif_stop_queue(port->ndev);
            ret = NETDEV_TX_BUSY;
            break;
        }

        if (can_dropped_invalid_skb(ndev, skb)) {
            _DBG_("can_dropped_invalid_skb");
            ret = NETDEV_TX_OK;
            break;
        }
        filled = fifo->w - fifo->r;
        space = CAN_TX_FIFO_DEPTH - filled;
        if (space <= 1) {
            netif_stop_queue(port->ndev);
            if (!space) {
                ret = NETDEV_TX_BUSY;
                break;
            }
        }
        dst = &fifo->dat[fifo->w & CAN_TX_FIFO_MASK];
        dst->echo = fifo->w & CAN_TX_FIFO_MASK;
        dst->is_canfd = fd;
        dst->if_loopback = loopback;
        memcpy(&dst->canfd, src, size);
        fifo->w++;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5,12,0))
        can_put_echo_skb(skb, port->ndev, dst->echo, 0);
#else
        can_put_echo_skb(skb, port->ndev, dst->echo);
#endif

    } while (0);
    UCFD_SIG_WAKE(&d->tx_signal);
    _DBG_("--- ret=%d", ret);
    return ret;
}

static netdev_tx_t usbcanfd_lin_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    usbcanfd_port *port = netdev_priv(ndev);
    usbcanfd_device *d = port->parent;
    usbcanfd_lin_tx_fifo *fifo = &((usbcanfd_lin_port*)port)->tx_fifo;
    netdev_tx_t ret = NETDEV_TX_OK;
    struct can_frame *src = (struct can_frame *)skb->data;
    usbcanfd_lin_frame *dst;
    u32 filled, space;
    u8 *p = src->data;
    _DBG_("+++ can%d: id=0x%08x, len=%d, dat=%02x %02x %02x %02x %02x %02x %02x %02x",
        port->index, src->can_id, src->len, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
    do {
        if (can_dropped_invalid_skb(ndev, skb)) {
            _DBG_("can_dropped_invalid_skb");
            ret = NETDEV_TX_OK;
            break;
        }
        filled = fifo->w - fifo->r;
        space = LIN_TX_FIFO_DEPTH - filled;
        if (space <= 1) {
            netif_stop_queue(port->ndev);
            if (!space) {
                ret = NETDEV_TX_BUSY;
                break;
            }
        }
        dst = &fifo->dat[fifo->w & LIN_TX_FIFO_MASK];
        dst->echo = fifo->w & LIN_TX_FIFO_MASK;
        dst->id = src->can_id;
        dst->len = src->can_dlc;
        memcpy(&dst->dat, src->data, src->len);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5,12,0))
        can_put_echo_skb(skb, port->ndev, dst->echo, 0);
#else
        can_put_echo_skb(skb, port->ndev, dst->echo);
#endif

        fifo->w++;
    } while (0);
    UCFD_SIG_WAKE(&d->tx_signal);
    _DBG_("--- ret=%d", ret);
    return ret;
}

static netdev_tx_t usbcanfd_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    usbcanfd_port *port = netdev_priv(ndev);
    if (port->type == CAN_PORT)
        return usbcanfd_can_start_xmit(skb, ndev);
    else
        return usbcanfd_lin_start_xmit(skb, ndev);
}

static const struct net_device_ops usbcanfd_ndev_ops = {
    .ndo_open = usbcanfd_open,
    .ndo_stop = usbcanfd_stop,
    .ndo_start_xmit = usbcanfd_start_xmit,
    .ndo_change_mtu = can_change_mtu,
};

static const struct can_bittiming_const usbcanfd_a_bittiming_const = {
    .name = USBCANFD_DRIVER_NAME,
    .tseg1_min = 1,
    .tseg1_max = 256,
    .tseg2_min = 1,
    .tseg2_max = 128,
    .sjw_max = 128,
    .brp_min = 1,
    .brp_max = 512,
    .brp_inc = 1,
};
static const struct can_bittiming_const usbcanfd_d_bittiming_const = {
    .name = USBCANFD_DRIVER_NAME,
    .tseg1_min = 1,
    .tseg1_max = 16,
    .tseg2_min = 1,
    .tseg2_max = 16,
    .sjw_max = 16,
    .brp_min = 1,
    .brp_max = 32,
    .brp_inc = 1,
};

static int usbcanfd_get_berr(const struct net_device *ndev, struct can_berr_counter *bec)
{
    usbcanfd_port *p = netdev_priv(ndev);
    bec->txerr = p->bec.txerr;
    bec->rxerr = p->bec.rxerr;
    _DBG_("txerr=0x%02x, rxerr=0x%02x", bec->txerr, bec->rxerr);
    return 0;
}

ssize_t usbcanfd_sysfs_show_can_ports(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = container_of(dev, struct usb_interface, dev);
    usbcanfd_device *d = usb_get_intfdata(intf);
    ssize_t len = 0;
    int i;

    for (i = 0; i < d->n_can_ports; i++) {
        len += sprintf(buf + len, "%s", d->can_ports[i]->ndev->name);
        if (i < d->n_can_ports - 1) {
            len += sprintf(buf + len, ",");
        }
    }
    len += sprintf(buf + len, "\n");

    return len;
}

ssize_t usbcanfd_sysfs_store_can_ports(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
    return cnt;
}

static void usbcanfd_del_lin_port(usbcanfd_device *d, int idx)
{
    usbcanfd_port *port = d->lin_ports[idx];
    _DBG_("+++");
    if (port) {
        struct net_device *ndev = port->ndev;
        if (ndev) {
            netdev_info(ndev, "device disconnected\n");
            if (port->ndev_rdy)
                unregister_netdev(ndev);
            free_candev(ndev);
            _DBG_("free_candev(port%d)", idx);
        }
        d->lin_ports[idx] = NULL;
    }
    _DBG_("---");
}

static int usbcanfd_add_lin_port(struct usb_interface *intf, usbcanfd_device *d, int idx)
{
    int err = -ENODEV;
    struct net_device *ndev = NULL;
    usbcanfd_port *port = NULL;
    _DBG_("+++ lin%d", idx);
    do {
        ndev = alloc_candev(sizeof(usbcanfd_lin_port), LIN_TX_FIFO_DEPTH);
        if (!ndev) {
            _MSG_("alloc_candev(%d) failed", (unsigned)sizeof(usbcanfd_lin_port));
            break;
        }
        _DBG_("alloc_candev(%d) succeeded", (unsigned)sizeof(usbcanfd_lin_port));

        strcpy(ndev->name, "lin%d");

        d->lin_ports[idx] = port = netdev_priv(ndev);
        port->ndev = ndev;
        port->parent = d;
        port->index = idx;
        port->ndev_rdy = false;
        port->type = LIN_PORT;

        port->can.state = CAN_STATE_STOPPED;
        port->can.clock.freq = d->can_clk;
        port->can.bittiming_const = &usbcanfd_a_bittiming_const;
        port->can.data_bittiming_const = &usbcanfd_d_bittiming_const;
        port->can.do_set_mode = NULL;
        port->can.do_get_berr_counter = usbcanfd_get_berr;
        port->can.ctrlmode_supported = CAN_CTRLMODE_FD | CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_LOOPBACK;

        ndev->netdev_ops = &usbcanfd_ndev_ops;

        SET_NETDEV_DEV(ndev, &intf->dev);
        err = register_candev(ndev);
        if (err) {
            netdev_err(ndev, "register_candev failed: err=%d\n", err);
            break;
        }
        _DBG_("register_candev(lin%d) succeeded", idx);
        port->ndev_rdy = true;
        err = 0;
    } while (0);
    if (err) usbcanfd_del_lin_port(d, idx);
    _DBG_("--- lin%d, err=%d", idx, err);
    return err;
}

static void usbcanfd_del_can_port(usbcanfd_device *d, int idx)
{
    usbcanfd_port *port = d->can_ports[idx];
    _DBG_("+++");
    if (port) {
        struct net_device *ndev = port->ndev;
        if (ndev) {
            netdev_info(ndev, "device disconnected\n");
            if (port->ndev_rdy)
                unregister_netdev(ndev);
            free_candev(ndev);
            _DBG_("free_candev(port%d)", idx);
        }
        d->can_ports[idx] = NULL;
    }
    _DBG_("---");
}

static int usbcanfd_add_can_port(struct usb_interface *intf, usbcanfd_device *d, int idx)
{
    int err = -ENODEV;
    struct net_device *ndev = NULL;
    usbcanfd_port *port = NULL;
    _DBG_("+++ can%d", idx);
    do {
        ndev = alloc_candev(sizeof(usbcanfd_can_port), CAN_TX_FIFO_DEPTH);
        if (!ndev) {
            _MSG_("alloc_candev(%d) failed", (unsigned)sizeof(usbcanfd_lin_port));
            break;
        }
        _DBG_("alloc_candev(%d) succeeded", (unsigned)sizeof(usbcanfd_lin_port));

        d->can_ports[idx] = port = netdev_priv(ndev);
        port->ndev = ndev;
        port->parent = d;
        port->index = idx;
        port->ndev_rdy = false;
        port->type = CAN_PORT;

        port->can.state = CAN_STATE_STOPPED;
        port->can.clock.freq = d->can_clk;
        port->can.bittiming_const = &usbcanfd_a_bittiming_const;
        port->can.data_bittiming_const = &usbcanfd_d_bittiming_const;
        port->can.do_set_mode = NULL;
        port->can.do_get_berr_counter = usbcanfd_get_berr;
        port->can.ctrlmode_supported = CAN_CTRLMODE_FD | CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_LOOPBACK;

        ndev->netdev_ops = &usbcanfd_ndev_ops;

        SET_NETDEV_DEV(ndev, &intf->dev);
        err = register_candev(ndev);
        if (err) {
            netdev_err(ndev, "register_candev failed: err=%d\n", err);
            break;
        }
        _DBG_("register_candev(can%d) succeeded", idx);
        port->ndev_rdy = true;
        err = 0;
    } while (0);
    if (err) usbcanfd_del_can_port(d, idx);
    _DBG_("--- can%d, err=%d", idx, err);
    return err;
}

ssize_t usbcanfd_sysfs_show_lin0_cfg(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = container_of(dev, struct usb_interface, dev);
    usbcanfd_device *d = usb_get_intfdata(intf);
    usbcanfd_lin_port *p = (usbcanfd_lin_port*)d->lin_ports[0];
    return sprintf(buf, "%s\n", p->cfg);
}

ssize_t usbcanfd_sysfs_show_lin1_cfg(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = container_of(dev, struct usb_interface, dev);
    usbcanfd_device *d = usb_get_intfdata(intf);
    usbcanfd_lin_port *p = (usbcanfd_lin_port*)d->lin_ports[1];
    return sprintf(buf, "%s\n", p->cfg);
}

ssize_t usbcanfd_sysfs_store_lin0_cfg(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
    struct usb_interface *intf = container_of(dev, struct usb_interface, dev);
    usbcanfd_device *d = usb_get_intfdata(intf);
    usbcanfd_lin_port *p = (usbcanfd_lin_port*)d->lin_ports[0];
    memcpy(p->cfg, buf, min(cnt, 4095lu));
    return cnt;
}

ssize_t usbcanfd_sysfs_store_lin1_cfg(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
    struct usb_interface *intf = container_of(dev, struct usb_interface, dev);
    usbcanfd_device *d = usb_get_intfdata(intf);
    usbcanfd_lin_port *p = (usbcanfd_lin_port*)d->lin_ports[1];
    memcpy(p->cfg, buf, min(cnt, 4095lu));
    return cnt;
}

ssize_t usbcanfd_sysfs_show_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t usbcanfd_sysfs_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
    struct usb_interface *intf = container_of(dev, struct usb_interface, dev);
    usbcanfd_device *d = usb_get_intfdata(intf);
    if (cnt >= 5 && !memcmp(buf, "reset", 5)) {
        usbcanfd_device_reset(d->udev);
    }
    return cnt;
}

static DEVICE_ATTR(lin0_cfg, 0644, usbcanfd_sysfs_show_lin0_cfg, usbcanfd_sysfs_store_lin0_cfg);
static DEVICE_ATTR(lin1_cfg, 0644, usbcanfd_sysfs_show_lin1_cfg, usbcanfd_sysfs_store_lin1_cfg);
static DEVICE_ATTR(can_ports, 0644, usbcanfd_sysfs_show_can_ports, usbcanfd_sysfs_store_can_ports);
static DEVICE_ATTR(reset, 0644, usbcanfd_sysfs_show_reset, usbcanfd_sysfs_store_reset);

static void usbcanfd_sysfs_create(struct device *dev)
{
    device_create_file(dev, &dev_attr_lin0_cfg);
    device_create_file(dev, &dev_attr_lin1_cfg);
    device_create_file(dev, &dev_attr_can_ports);
    device_create_file(dev, &dev_attr_reset);
}

static void usbcanfd_sysfs_remove(struct device *dev)
{
    device_remove_file(dev, &dev_attr_lin0_cfg);
    device_remove_file(dev, &dev_attr_lin1_cfg);
    device_remove_file(dev, &dev_attr_can_ports);
    device_remove_file(dev, &dev_attr_reset);
}

static void usbcanfd_disconnect(struct usb_interface *intf)
{
    usbcanfd_device *d = usb_get_intfdata(intf);
    unsigned i;
    _DBG_("+++");
    usbcanfd_sysfs_remove(&intf->dev);
    usb_set_intfdata(intf, NULL);
    if (d) {
        d->stop = true;
        if (d->tx_thread) kthread_stop(d->tx_thread);
        for (i = 0; i < d->n_can_ports; i++) {
            usbcanfd_del_can_port(d, i);
        }
        for (i = 0; i < d->n_lin_ports; i++) {
            usbcanfd_del_lin_port(d, i);
        }
        usb_kill_anchored_urbs(&d->rx_anchor);
        for (i = 0; i < cfg_rx_urbs; i++) {
            usb_free_coherent(d->udev, cfg_rx_size, d->rxbuf[i], d->rxbuf_dma[i]);
        }
        kfree(d);
    }
    _DBG_("---");
}

static u16 usbcanfd_proto_version_get(struct usb_device *dev) 
{
    int err = 0;
    u16 ver = 0;
    u16 *buf = kmalloc(2, GFP_KERNEL);
    if (!buf) {
        return 0;
    }

    err = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), 0, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, 0, 0, (void *)buf, 4, USB_CTRL_GET_TIMEOUT);
    if (err < 0) {
        kfree(buf);
        _MSG_("get protocol version failed, %u", err);
        return 0;
    }
	
    ver = *buf;
    _DBG_("protoco ver:%u.%2x", (ver >> 8) & 0xFF, ver & 0xFF);

    kfree(buf);

    return ver;
}

static u32 usbcanfd_packet_max_size_get(struct usb_device *dev) 
{
    int err = 0;
    unsigned int size = 0;
    unsigned int *buf = kmalloc(4, GFP_KERNEL);
    if (!buf) {
        return 1024;
    }

    err = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), 1, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, 0, 0, (void *)buf, 4, USB_CTRL_GET_TIMEOUT);
    if (err < 0) {
        kfree(buf);
        _MSG_("get packet size failed, %u", err);
        return 1024;
    }

    size = *buf;
    _DBG_("max packet size:%u", size);

    kfree(buf);

    return size;
}

static u8 usbcanfd_cmd_is_upload_us(usbcanfd_device *d) 
{
    int ret = 0;
    UCFD_CMD *cmd = &d->cmd;
    UCFD_CMD_INIT(cmd, CMD_CAN_TRANS_PROP_GET, 0, 0);
    ret = usbcanfd_cmd(d);
    if (ret < 2 || usbcanfd_chkack(d, 1)) {
        _MSG_("usbcanfd_cmd(CMD_CAN_TRANS_PROP_GET) failed");
        return 0xFF;
    }

    return d->ack.dat[0];
}

static int usbcanfd_cmd_upload_us_set(usbcanfd_device *d, bool us) 
{
    int ret = 0;
    UCFD_CMD *cmd = &d->cmd;
    UCFD_CMD_INIT(cmd, CMD_CAN_TRANS_PROP_SET, 0, 1);
    cmd->dat[0] = us ? 1 : 0;
    ret = usbcanfd_cmd(d);
    if (ret < 2 || usbcanfd_chkack(d, 0)) {
        _MSG_("usbcanfd_cmd(CMD_CAN_TRANS_PROP_SET) failed");
        return -1;
    }

    return 0;
}

static int usbcanfd_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(intf);
    usbcanfd_device *d = NULL;
    UCFD_CMD *cmd;
    int err = -ENODEV;
    int ret;
    int i;
    u16 ver = 0;

    if (cfg_rx_urbs > USBCANFD_USB_MAX_RX_URBS) {
        cfg_rx_urbs = USBCANFD_USB_MAX_RX_URBS;
    }


    _DBG_("+++");
    do {
        d = kzalloc(sizeof(*d), GFP_KERNEL);
        if (!d) {
            _MSG_("kzalloc(%d) failed", (unsigned)sizeof(*d));
            break;
        }
        _DBG_("kzalloc(%d) succeeded", (unsigned)sizeof(*d));
        usb_set_intfdata(intf, d);
        d->udev = udev;
        mutex_init(&d->cmd_lock);
        init_usb_anchor(&d->rx_anchor);
        UCFD_SIG_INIT(&d->tx_signal);

        strcpy(d->sn, "unknown");

	    ver = usbcanfd_proto_version_get(udev);
	    if (ver >= 0x113) {
            d->max_packet_size = usbcanfd_packet_max_size_get(udev);
	    } else {
            d->max_packet_size = 1024;
	    }

        if (ver >= 0x116) {
            if (!usbcanfd_cmd_is_upload_us(d)) {
                usbcanfd_cmd_upload_us_set(d, 1);
            }
        }

        d->max_tx_can_cnt = d->max_packet_size / 1024 * 12;
        if (d->max_tx_can_cnt > MAX_TX_CNT) {
            d->max_tx_can_cnt = MAX_TX_CNT;
        }
    	_INF_("max tx cnt:%u", d->max_tx_can_cnt);

        cmd = &d->cmd;
        UCFD_CMD_INIT(cmd, CMD_DEV_INFO, 0, 0);
        ret = usbcanfd_cmd(d);
        if (ret < 2 || usbcanfd_chkack(d, 0x44)) {
            _MSG_("usbcanfd_cmd(CMD_DEV_INFO) failed");
            break;
        }
        _DBG_("usbcanfd_cmd(CMD_DEV_INFO) succeeded");
        {
            UCFD_DEV_INFO *inf = (UCFD_DEV_INFO*)d->ack.dat;
            d->sn[20] = '\0';
            memcpy(d->sn, inf->sn, 20);
            _INF_("DEV_INFO: can_ports=%d, lin_ports=%d, hw=0x%04x, sw=0x%04x, boot=0x%04x, id=%s, sn=%s",
                inf->can_ports,
                inf->lin_ports,
                inf->hw,
                inf->sw,
                inf->boot,
                inf->id,
                d->sn);
            if (inf->hw >= 0x0300 || inf->can_ports == 4) {
                d->can_clk = 80000000;
            } else {
                d->can_clk = 60000000;
            }
            d->n_can_ports = min(UCFD_CAN_PORTS, (int)inf->can_ports);
            d->n_lin_ports = min(UCFD_LIN_PORTS, (int)inf->lin_ports);
        }

        for (i = 0; i < d->n_can_ports; i++) {
            usbcanfd_add_can_port(intf, d, i);
        }
        for (i = 0; i < d->n_lin_ports; i++) {
            usbcanfd_add_lin_port(intf, d, i);
        }

        d->tx_thread = kthread_run(usbcanfd_tx_thread, d, "%s", USBCANFD_DRIVER_NAME);

        usbcanfd_sysfs_create(&intf->dev);
        err = 0;
    } while (0);
    if (err) {
        usbcanfd_device_reset(udev);
        usbcanfd_disconnect(intf);
    }
    _DBG_("--- err=%d", err);
    return err;
}

static struct usb_driver usbcanfd_driver = {
    .name = USBCANFD_DRIVER_NAME,
    .id_table = usbcanfd_id_table,
    .probe = usbcanfd_probe,
    .disconnect = usbcanfd_disconnect,
};
module_usb_driver(usbcanfd_driver);
