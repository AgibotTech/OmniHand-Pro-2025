#pragma once

#define UCFD_VID  0x04cc
#define UCFD_PID  0x1240

#define UCFD_CAN_PORTS  4
#define UCFD_LIN_PORTS  2

#define MAX_EPS  5
#define EP_CMD  0x01
#define EP_ACK  0x81
#define EP_TXCMD  0x02
#define EP_TXACK  0x82
#define EP_RX  0x83

#define EP_IN_SIZE  1024
#define EP_OUT_SIZE  1024

#define GET_U16(p)  (*(u16*)(p))
#define GET_U32(p)  (*(u32*)(p))
#define SET_U16(p,v)  (*(u16*)(p) = (u16)(v))

#define CMD_TIMEOUT  2000
#define TX_TIMEOUT  10000

#define MAX_TX_CNT  96u

#define CMD_HDR  0x55
#define CRC_SIZE  2
#define CMD_HDR_SIZE  8
#define ACK_HDR_SIZE  12
#define CMD_DAT_SIZE  EP_OUT_SIZE
#define ACK_DAT_SIZE  EP_IN_SIZE

enum {
    CMD_OPEN = 0x01, /**< 设备启动 */
    CMD_CLOSE = 0x02, /**< 设备关闭 */
    CMD_SECURE = 0x03, /**< 安全验证 */
    CMD_LOGLEN = 0x04, /**< 获取日志大小 */
    CMD_LOGMSG = 0x05, /**< 获取日志数据 */
    CMD_RTC = 0x06, /**< 设置时间 */
    CMD_TEST = 0x07, /**< 测试命令 */
    CMD_CAN_INIT = 0x11, /**< CAN初始化 */
    CMD_CAN_START = 0x12, /**< 启动 */
    CMD_CAN_STOP = 0x13, /**< 停止 */
    CMD_CAN_FILTER = 0x14, /**< 配置滤波表 */
    CMD_CAN_ERRCNT = 0x15, /**< 获取错误计数 */
    CMD_CAN_TTX = 0x16, /**< 定时发送配置 */
    CMD_CAN_TTX_CTL = 0x17, /**< 定时发送控制 */
    CMD_CAN_TRES = 0x18, /**< 终端电阻设置 */
    CMD_DEV_INFO = 0x41, /**< 获取设备信息 */
    CMD_CAN_TX_TIMEOUT = 0x44, /**< 设置发送超时 */
    CMD_CAN_TRANS_PROP_GET = 0x46, 
    CMD_CAN_TRANS_PROP_SET = 0x47, 
    CMD_LIN_INIT = 0x60, /**< LIN初始化 */
};

enum {
    XFER_CAN20 = 0xf1, /**< CAN2.0帧 */
    XFER_CANFD = 0xf2, /**< CANFD帧 */
    XFER_CANERR = 0xf3, /**< CAN错误帧 */
    XFER_LIN= 0xf4, /**< LIN帧 */
};

#pragma pack(push, 1)

typedef enum {
    UCFD_TX_NORM = 0, /**< normal transmission */
    UCFD_TX_ONCE = 1, /**< single-shot transmission */
    UCFD_SR_NORM = 2, /**< self reception */
    UCFD_SR_ONCE = 3, /**< single-shot transmission & self reception */
} UCFD_CAN_TX_MODE;

/** CAN filter configuration */
typedef struct {
    u8 type; /**< 0-std_frame, 1-ext_frame */
    u8 pad[3];
    u32 sid; /**< start-id */
    u32 eid; /**< end-id */
} UCFD_CAN_FILTER;

/** controller initialization */
typedef struct {
    u32 clk; /**< clock(Hz) */
    u32 mode; /**< bit0-normal/listen_only, bit1-ISO/BOSCH */
    struct {
        u8 tseg1;
        u8 tseg2;
        u8 sjw;
        u8 smp;
        u16 brp;
    } aset;
    struct {
        u8 tseg1;
        u8 tseg2;
        u8 sjw;
        u8 smp;
        u16 brp;
    } dset;
} UCFD_CAN_INIT;

/** CAN message info */
typedef struct {
    u32 txm : 4; /**< TX-mode, @see UCFD_CAN_TX_MODE */
    u32 fmt : 4; /**< 0-CAN2.0, 1-CANFD */
    u32 sdf : 1; /**< 0-data_frame, 1-remote_frame */
    u32 sef : 1; /**< 0-std_frame, 1-ext_frame */
    u32 err : 1; /**< error flag */
    u32 brs : 1; /**< bit-rate switch */
    u32 est : 1; /**< error state */
    u32 tx  : 1; /**< tx flag */
    u32 echo : 1; /**< tx flag */
    u32 qsend_flag: 1; /**< qsend flag */
    u32 ts_ov_cnt : 4; /**< ts ov */
    u32 ts_us: 1; /**< us ts */
    u32 pad : 11;
} UCFD_CAN_MSG_INF;

/** LIN message info */
typedef union {
    struct {
        u16 txm : 2; /**< TX-mode */
        u16 ckm: 2; /**< Checksum-mode */
        u16 wk0 : 1; /**< wakeup */
        u16 rs0: 11; /**< reserved */
    };
    struct {
        u16 dir : 1; /**< direction */
        u16 err : 1; /**< error */
        u16 est : 4; /**< error-stage */
        u16 ers : 4; /**< error-reason */
        u16 wk1 : 1; /**< wakeup */
        u16 rs1: 5; /**< reserved */
    };
    u16 val;
} UCFD_LIN_MSG_INF;

/** CAN message header */
typedef struct {
    u32 ts; /**< timestamp */
    u32 id; /**< CAN-ID */
    UCFD_CAN_MSG_INF inf; /**< @see UCFD_CAN_MSG_INF */
    u16 pad;
    u8 chn; /**< channel */
    u8 len; /**< data length */
} UCFD_CAN_MSG_HDR;

/** LIN message header */
typedef struct {
    u64 ts; /**< timestamp */
    u8 chn; /**< channel */
    u8 pid; /**< pid */
    UCFD_LIN_MSG_INF inf; /**< @see UCFD_LIN_MSG_INF */
    u16 pad;
    u8 cks; /**< checksum */
    u8 len; /**< data length */
} UCFD_LIN_MSG_HDR;

/** CAN2.0-frame */
typedef struct {
    UCFD_CAN_MSG_HDR hdr;
    u8 dat[8];
} UCFD_20_MSG;

/** CANFD-frame */
typedef struct {
    UCFD_CAN_MSG_HDR hdr;
    u8 dat[64];
} UCFD_FD_MSG;

/** CANERR-frame */
typedef struct {
    UCFD_CAN_MSG_HDR hdr;
    u8 dat[8];
} UCFD_ERR_MSG;

/** LIN-frame */
typedef struct {
    UCFD_LIN_MSG_HDR hdr;
    u8 dat[8];
} UCFD_LIN_MSG;

typedef struct {
    u8 hdr; /**< 命令头 */
    u8 cmd; /**< 命令 */
    u8 ext; /**< 命令扩展参数 */
    u8 idx; /**< 命令序号 */
    u32 len; /**< 数据长度 */
    u8 dat[CMD_DAT_SIZE]; /**< 数据区 */
} UCFD_CMD;

typedef struct {
    u8 hdr; /**< 命令头 */
    u8 cmd; /**< 命令 */
    u8 ext; /**< 命令扩展参数 */
    u8 idx; /**< 命令序号 */
    u8 err; /**< 错误状态 */
    u8 pad[3];
    u32 len; /**< 数据长度 */
    u8 dat[CMD_DAT_SIZE]; /**< 数据区 */
} UCFD_ACK;

typedef struct {
    u8 hdr; /**< 命令头 */
    u8 cmd; /**< 命令 */
    u8 ext; /**< 命令扩展参数 */
    u8 idx; /**< 命令序号 */
    u32 len; /**< 数据长度 */
    u8 dat[0x4000 - 8]; /**< 数据区 */
} UCFD_TXCMD;

typedef struct {
    u8 hdr; /**< 命令头 */
    u8 cmd; /**< 命令 */
    u8 ext; /**< 命令扩展参数 */
    u8 idx; /**< 命令序号 */
    u8 err; /**< 错误状态 */
    u8 sent; /**< 报文成功计数 */
    u8 crc[CRC_SIZE];
} UCFD_TXACK;

typedef struct {
    u16 hw;
    u16 sw;
    u16 boot;
    u8 id[40];
    u8 sn[20];
    u8 can_ports;
    u8 lin_ports;
} UCFD_DEV_INFO;

typedef struct {
    u16 cnt;
    u16 pad;
    union {
        UCFD_20_MSG can20[MAX_TX_CNT];
        UCFD_FD_MSG canfd[MAX_TX_CNT];
        UCFD_ERR_MSG canerr[MAX_TX_CNT];
        UCFD_LIN_MSG lin[MAX_TX_CNT];
        u8 dat[1020];
    } msgs;
} UCFD_XFER;

#pragma pack(pop)

#define UCFD_CMD_INIT(_buf,_cmd,_ext,_len)  {memset(_buf,0,sizeof(UCFD_CMD));_buf->hdr=CMD_HDR;_buf->cmd=_cmd;      _buf->ext=_ext;_buf->len=_len;}
#define UCFD_CAN_TX_INIT(_buf,_ext,_cnt)    {memset(_buf,0,sizeof(UCFD_CMD));_buf->hdr=CMD_HDR;_buf->cmd=XFER_CANFD;_buf->ext=_ext;_buf->len=4+(_cnt)*sizeof(UCFD_FD_MSG); _buf->dat[0]=(_cnt);}
#define UCFD_LIN_TX_INIT(_buf,_ext,_cnt)    {memset(_buf,0,sizeof(UCFD_CMD));_buf->hdr=CMD_HDR;_buf->cmd=XFER_LIN;  _buf->ext=_ext;_buf->len=4+(_cnt)*sizeof(UCFD_LIN_MSG);_buf->dat[0]=(_cnt);}

