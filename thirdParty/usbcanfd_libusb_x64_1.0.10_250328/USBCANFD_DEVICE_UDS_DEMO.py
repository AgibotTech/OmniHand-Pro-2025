'''
if there is no receive device(only usbcanfd) , the ZCAN_MSG_INFO "txm" use txtype --2, self test
if there is a same baudrate receive device ,  the ZCAN_MSG_INFO "txm" use txtype --0, normal send

ZLG  Zhiyuan Electronics
'''

from ctypes import *
import threading
import time
import datetime

lib = cdll.LoadLibrary("./libusbcanfd.so")

CMD_CAN_FILTER                  = 0x14   # 滤波
CMD_CAN_TTX                     = 0x16   # 定时发送
CMD_CAN_TTX_CTL                 = 0x17   # 使能定时发送
CMD_CAN_TRES                    = 0x18   # CAN终端电阻
ZCAN_CMD_SET_CHNL_RECV_MERGE    = 0x32   # 设置合并接收 0:不合并接收;1:合并接收
ZCAN_CMD_GET_CHNL_RECV_MERGE    = 0x33   # 获取是否开启合并接收 0:不合并接收;1:合并接收
CMD_SET_SN                      = 0x42   # 获取SN号
CMD_GET_SN                      = 0x43   # 设置SN号
CMD_CAN_TX_TIMEOUT              = 0x44   # 发送超时
ZCAN_CMD_GET_SEND_QUEUE_SIZE    = 0x100  # 获取队列大小，uint32_t
ZCAN_CMD_GET_SEND_QUEUE_SPACE   = 0x101  # 获取队列剩余空间, uint32_t
ZCAN_CMD_SET_SEND_QUEUE_CLR     = 0x102  # 清空发送队列,1：清空
ZCAN_CMD_SET_SEND_QUEUE_EN      = 0x103  # 开启发送队列,1：使能

USBCANFD = c_uint32(33)  # 设备类型号
MAX_CHANNELS = 2         # 通道最大数量
g_thd_run = 1            # 线程运行标志
threads = []             # 接收线程
can_or_canfd = 0;	     # 是否用CANFD 0-can，1-canfd

# can/canfd messgae info
class ZCAN_MSG_INFO(Structure):
    _fields_ = [("txm", c_uint, 4),  # TXTYPE:0 normal,1 once, 2self
                ("fmt", c_uint, 4),  # 0-can2.0 frame,  1-canfd frame
                ("sdf", c_uint, 1),  # 0-data frame, 1-remote frame
                ("sef", c_uint, 1),  # 0-std_frame, 1-ext_frame
                ("err", c_uint, 1),  # error flag
                ("brs", c_uint, 1),  # bit-rate switch ,0-Not speed up ,1-speed up
                ("est", c_uint, 1),  # error state
                ("tx", c_uint, 1),      # received valid, tx frame
                ("echo", c_uint, 1),    # tx valid, echo frame
                ("qsend_100us", c_uint, 1), # queue send delay unit, 1-100us, 0-ms
                ("qsend", c_uint, 1),       # send valid, queue send frame
                ("pad", c_uint, 15)]


# CAN Message Header
class ZCAN_MSG_HDR(Structure):
    _fields_ = [("ts", c_uint32),  # timestamp
                ("id", c_uint32),  # can-id
                ("inf", ZCAN_MSG_INFO),
                ("pad", c_uint16),
                ("chn", c_uint8),  # channel
                ("len", c_uint8)]  # data length

# CAN2.0-frame
class ZCAN_20_MSG(Structure):
    _fields_ = [("hdr", ZCAN_MSG_HDR),
                ("dat", c_ubyte*8)]


# CANFD frame
class ZCAN_FD_MSG(Structure):
    _fields_ = [("hdr", ZCAN_MSG_HDR),
                ("dat", c_ubyte*64)]

# filter_set
class ZCAN_FILTER(Structure):
    _fields_ = [("type", c_uint8),  # 0-std_frame,1-extend_frame
                ("pad", c_uint8*3),  # reserved
                ("sid", c_uint32),  # start_ID
                ("eid", c_uint32)]  # end_ID


class ZCAN_FILTER_TABLE(Structure):
    _fields_ = [("size", c_uint32),  # 滤波数组table实际生效部分的长度
                ("table", ZCAN_FILTER*64)]


class abit_config(Structure):
    _fields_ = [("tseg1", c_uint8),
                ("tseg2", c_uint8),
                ("sjw", c_uint8),
                ("smp", c_uint8),
                ("brp", c_uint16)]


class dbit_config(Structure):
    _fields_ = [("tseg1", c_uint8),
                ("tseg2", c_uint8),
                ("sjw", c_uint8),
                ("smp", c_uint8),
                ("brp", c_uint16)]


class ZCANFD_INIT(Structure):
    _fields_ = [("clk", c_uint32),
                ("mode", c_uint32),
                ("abit", abit_config),
                ("dbit", dbit_config)]

# Terminating resistor
class Resistance(Structure):
    _fields_ = [("res", c_uint8)]

# autosend
class ZCAN_TTX(Structure):
    _fields_ = [("interval", c_uint32),   # 定时发送周期，单位百微秒
                ("repeat", c_uint16),     # 发送次数，0等于循环发
                ("index", c_uint8),       # 定时发送列表的帧索引号，也就是第几条定时发送报文
                ("flags", c_uint8),       # 0-此帧禁用定时发送，1-此帧使能定时发送
                ("msg", ZCAN_FD_MSG)]     # CANFD帧结构体

# autosend list
class ZCAN_TTX_CFG(Structure):
    _fields_ = [("size", c_uint32),       # 实际生效的数组的长度
                ("table", ZCAN_TTX * 8)]  # 最大设置8条
    
############  uds resoponse #############
class PARAM_DATA(Structure):
    _pack_ = 1
    _fields_ = [("data", c_ubyte*4096)]


class DATA_BUFFER(Structure):
    _pack_ = 1
    _fields_ = [("data", c_ubyte*4096)]


class POSITIVE_DATA(Structure):
    _pack_ = 1
    _fields_ = [("sid", c_ubyte),
                ("data_len", c_uint),
                ]


class NEGATIVE_DATA(Structure):
    _pack_ = 1
    _fields_ = [("neg_code", c_ubyte),
                ("sid", c_ubyte),
                ("error_code", c_ubyte),
                ]


class RESPONSE_DATA(Union):
    _pack_ = 1
    _fields_ = [("positive", POSITIVE_DATA),
                ("negative", NEGATIVE_DATA),
                ("raw", c_byte*8),
                ]


class ZCAN_UDS_RESPONSE(Structure):
    _pack_ = 1
    _fields_ = [
        ("status", c_byte),  # 见ZCAN_UDS_ERROR说明
        ("reserved", c_byte*6),
        ("type", c_byte),  # 0-消极响应,1-积极响应
        ("response", RESPONSE_DATA),
    ]

#######################  uds request #################

class ZCAN_UDS_SESSION_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("p2_timeout", c_uint),
                # 收到消极响应错误码为0x78后的超时时间(ms)。因PC定时器误差，建议设置不小于200ms
                ("enhanced_timeout", c_uint),
                # 接收到非本次请求服务的消极响应时是否需要判定为响应错误
                ("check_any_negative_response", c_ubyte, 1),
                # 抑制响应时是否需要等待消极响应，等待时长为响应超时时间
                ("wait_if_suppress_response", c_ubyte, 1),
                ("flag", c_ubyte, 6),  # 保留
                ("reserved0", c_byte*7)
                ]

class ZCAN_UDS_TRANS_PARAM(Structure):
    _pack_ = 1
    _fields_ = [
        ("version", c_byte),  # 0-2004版本，1-2016版本
        ("max_data_len", c_byte),  # 单帧最大数据长度, can:8, canfd:64
        # 本程序发送流控时用，连续帧之间的最小间隔, 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        ("local_st_min", c_byte),
        ("block_size", c_byte),  # 流控帧的块大小
        ("fill_byte", c_byte),  # 无效字节的填充数据
        ("ext_frame", c_byte),  # 0:标准帧 1:扩展帧
        # 是否忽略ECU返回流控的STmin，强制使用本程序设置的 remote_st_min
        ("is_modify_ecu_st_min", c_byte),
        # 发送多帧时用, is_ignore_ecu_st_min = 1 时有效, 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        ("remote_st_min", c_byte),
        ("fc_timeout", c_uint),  # 接收流控超时时间(ms), 如发送首帧后需要等待回应流控帧
        ("fill_mode", c_ubyte),  # 0-FILL_MODE_SHORT,1-FILL_MODE_NONE,2-FILL_MODE_MAX
        ("reserved0", c_byte*3),
    ]

class ZCAN_UDS_REQUEST(Structure):
    _pack_ = 1
    _fields_ = [("req_id", c_uint),  # 请求的事务ID，范围0~65535，本次请求的唯一标识
                ("channel", c_ubyte),  # 设备通道索引
                ("frame_type", c_ubyte),  # 0-can,1-CANFD,2-CANFD加速
                ("reserved0", c_byte*2),
                ("src_addr", c_uint),  # 请求ID
                ("dst_addr", c_uint),  # 响应ID
                ("suppress_response", c_byte),  # 1-抑制响应
                ("sid", c_ubyte),  # 请求服务id
                ("reserved1", c_byte*6),
                ("session_param", ZCAN_UDS_SESSION_PARAM),  # 会话层参数
                ("trans_param", ZCAN_UDS_TRANS_PARAM),  # 传输层参数
                ("data", POINTER(c_ubyte)),  # 请求参数
                ("data_len", c_uint),  # 请求参数长度
                ("reserved2", c_uint),
                ]


# 通道初始化，并开启接收线程
def canfd_start(DevType, DevIdx, ChIdx):
# 波特率结构体，数据根据zcanpro的波特率计算器得出
    canfd_init = ZCANFD_INIT()
    canfd_init.clk = 60000000
    canfd_init.mode = 0

    canfd_init.abit.tseg1 = 14  # 仲裁域
    canfd_init.abit.tseg2 = 3
    canfd_init.abit.sjw = 2
    canfd_init.abit.smp = 0   # smp是采样点，不涉及波特率计算
    canfd_init.abit.brp = 5

    canfd_init.dbit.tseg1 = 10  # 数据域
    canfd_init.dbit.tseg2 = 2
    canfd_init.dbit.sjw = 2
    canfd_init.dbit.smp = 0
    canfd_init.dbit.brp = 1

    # 初始化通道
    ret = lib.VCI_InitCAN(DevType, DevIdx, ChIdx, byref(canfd_init))
    if ret == 0:
        print("InitCAN(%d) fail" % i)
        exit(0)

    # 使能终端电阻
    Res = c_uint8(1)
    lib.VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TRES, byref(Res))

    # 启动通道
    ret = lib.VCI_StartCAN(DevType, DevIdx, ChIdx)
    if ret == 0:
        print("StartCAN(%d) fail" % i)
        exit(0)

    thread = threading.Thread(target=rx_thread, args=(DevType, DevIdx, i,))
    threads.append(thread) # 独立接收线程
    thread.start()


# 接收线程函数（加了ID判断）
def rx_thread(DevType, DevIdx, ChIdx):
    global g_thd_run

    while g_thd_run == 1:
        time.sleep(0.1)
        count = lib.VCI_GetReceiveNum(DevType, DevIdx, ChIdx) # CAN 报文数量
        if count > 0:
            can_data = (ZCAN_20_MSG * count)()
            rcount = lib.VCI_Receive(DevType, DevIdx, ChIdx, byref(can_data), count, 100) # 读报文
            for i in range(rcount):
                if ((can_data[i].hdr.id & 0x1FFFFFFF) != 0x731 and (can_data[i].hdr.id & 0x1FFFFFFF) != 0x7B1):
                    continue
                print("[%u] chn: %d " %(can_data[i].hdr.ts, ChIdx), end='')
                print("TX  " if can_data[i].hdr.inf.tx == 1 else "RX  ", end='')       # 判断是否回显
                print("CAN ID: 0x%x "%(can_data[i].hdr.id & 0x1FFFFFFF), end='')
                print("扩展帧  " if can_data[i].hdr.inf.sef == 1 else "标准帧  ", end='')
                print("Data: ", end='')
                if(can_data[i].hdr.inf.sdf == 0):   #数据帧
                    for j in range(can_data[i].hdr.len):
                        print("%02x " % can_data[i].dat[j], end='')
                print("")

        count = lib.VCI_GetReceiveNum(DevType, DevIdx, (0x80000000 + ChIdx)) # CANFD 报文数量
        if count > 0:
            canfd_data = (ZCAN_FD_MSG * count)()
            rcount = lib.VCI_ReceiveFD(DevType, DevIdx, ChIdx, byref(canfd_data), count, 100)
            for i in range(rcount):
                if ((canfd_data[i].hdr.id & 0x1FFFFFFF) != 0x731 and (canfd_data[i].hdr.id & 0x1FFFFFFF) != 0x7B1):
                    continue
                print("[%u] chn: %d " %(canfd_data[i].hdr.ts, ChIdx), end='')
                print("TX  " if canfd_data[i].hdr.inf.tx == 1 else "RX  ", end='')
                print("CANFD加速 " if canfd_data[i].hdr.inf.brs == 1 else "CANFD  ", end='')
                print("ID: 0x%x "%(canfd_data[i].hdr.id & 0x1FFFFFFF), end='')
                print("扩展帧  " if canfd_data[i].hdr.inf.sef == 1 else "标准帧  ", end='')

                print("Data: ", end='')
                for j in range(canfd_data[i].hdr.len):
                    print("%02x " % canfd_data[i].dat[j], end='')
                print("")


# 打印响应结果
def response_print(response, databuf):
    if ret == 1:  # 等于1代表此次request有效
        if response.status == 0:
            if response.type == 0:
                print("消极响应：%s %s %s" % (hex(response.response.negative.neg_code), hex(
                    response.response.negative.sid), hex(response.response.negative.error_code)))
            if response.type == 1:
                print("积极响应,响应id:%s,参数长度:%d,参数内容：%s" % (hex(response.response.positive.sid), response.response.positive.data_len, ''.join(
                    hex(databuf.data[i])+' 'for i in range(response.response.positive.data_len))))
        if response.status == 1:
            print("响应超时")
        if response.status == 2:
            print("传输失败，请检查链路层，或请确认流控帧是否回复")
        if response.status == 3:
            print("取消请求")
        if response.status == 4:
            print("抑制响应")
        if response.status == 5:
            print("忙碌中")
        if response.status == 6:
            print("请求参数错误")
    else:
        print("请求异常")

# UDS请求
def request_uds(DevType, DevIdx, ChIdx, req_id):
    request = ZCAN_UDS_REQUEST()
    memset(byref(request), 0, sizeof(request))
    request.req_id = req_id     # 请求事务ID，建议可以直接传通道句柄
    request.channel = ChIdx     # 通道号
    request.frame_type = 0      # 0-can,1-canfd,2-canfd加速
    request.src_addr = 0x731        # 物理地址
    request.dst_addr = 0x7B1        # 响应地址 
    request.suppress_response = 0   # 1-抑制响应
    request.sid = 0x10              # 请求服务id

    request.session_param.p2_timeout = 2000         # P2 响应超时时间(ms) 
    request.session_param.enhanced_timeout = 5000   # p2* 收到消极响应错误码为0x78后的超时时间(ms)
    request.session_param.check_any_negative_response = 0
    request.session_param.wait_if_suppress_response = 0

    request.trans_param.version = 0         # 0: ISO-15765-2004   1: ISO-15765-2016
    request.trans_param.max_data_len = 8    # 帧数据最长长度
    request.trans_param.local_st_min = 0    # 连续帧之间的最小间隔
    request.trans_param.block_size = 8      # 流控帧的块大小
    request.trans_param.fill_byte = 0x00    # 无效字节的填充数据
    request.trans_param.ext_frame = 0       # 0-标准帧，1-扩展帧
    request.trans_param.is_modify_ecu_st_min = 0    # 是否忽略ECU返回流控的STmin
    request.trans_param.remote_st_min = 0           # 忽略后，多帧时的帧间隔
    request.trans_param.fc_timeout = 500            # 等待流控超时时间(ms)
    request.trans_param.fill_mode = 0               # 0-就近填充，1-不填充，2全填充

    # 构造请求数据
    param_data_len = 1
    param_data = PARAM_DATA()  # 参数长度
    data_1 = [0x03]  # 实际参数内容
    for i in range(param_data_len):
        param_data.data[i] = data_1[i] & 0xff

    request.data = param_data.data
    request.data_len = param_data_len

    response = ZCAN_UDS_RESPONSE()
    memset(byref(response), 0, sizeof(response))

    databuf = DATA_BUFFER()
    if (lib.VCI_UDS_Request(DevType, DevIdx, byref(request), byref(response), byref(databuf), sizeof(databuf))):
        response_print(response, databuf)


# 主函数
if __name__ == "__main__":
    DEVICE_INDEX = c_uint32(0)  # 设备索引
    CHANNELS_INDEX = 0          # 测试请求的通道号

    # 打开设备
    ret = lib.VCI_OpenDevice(USBCANFD, DEVICE_INDEX, 0)
    if ret == 0:
        print("Open device fail")
        exit(0)
    else:
        print("Open device success")

    # 打开通道
    for i in range(MAX_CHANNELS):
        canfd_start(USBCANFD, DEVICE_INDEX, i)  # 初始化通道，并且开启接收线程

    # UDS请求
    request_uds(USBCANFD, DEVICE_INDEX, CHANNELS_INDEX, 0)


    # 阻塞等待
    input()
    g_thd_run = 0

    # 等待所有线程完成
    for thread in threads:
        thread.join()

    for i in range(MAX_CHANNELS):
        ret = lib.VCI_ResetCAN(USBCANFD, DEVICE_INDEX, i)
        if ret == 0:
            print("ResetCAN(%d) fail" % i)

    ret = lib.VCI_CloseDevice(USBCANFD, DEVICE_INDEX)
    if ret == 0:
        print("Close device fail")
    else:
        print("Close device success")
    del lib
