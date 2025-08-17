#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "zcan.h"

typedef unsigned int UINT;
typedef unsigned char U8;
typedef unsigned int U32;
#define msleep(ms)  usleep((ms)*1000)

#define ZUDS_ERROR_OK                   0    // 没错误
#define ZUDS_ERROR_TIMEOUT              1    // 响应超时
#define ZUDS_ERROR_TRANSPORT            2    // 发送数据失败
#define ZUDS_ERROR_CANCEL               3    // 取消请求
#define ZUDS_ERROR_SUPPRESS_RESPONSE    4    // 抑制响应
#define ZUDS_ERROR_OTHTER               100

#define USBCANFD 33
#define MAX_CHANNELS  2
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000


int can_or_canfd = 0;	        // 是否用CANFD 0-can，1-canfd

// 接收线程上下文
typedef struct
{
    int dev_type;   // 设备类型
    int dev_idx;    // 设备索引
    int chn_idx;    // 通道号
    int total;      // 接收总数
    int stop;       // 线程结束标志
} THREAD_CTX;

THREAD_CTX rx_ctx[MAX_CHANNELS];        // 接收线程上下文
pthread_t rx_threads[MAX_CHANNELS]; // 接收线程

// 接收线程函数（加了ID判断）
void *rx_thread(void *data)
{
    THREAD_CTX *ctx = (THREAD_CTX *)data;
    int DevType = ctx->dev_type;
    int DevIdx  = ctx->dev_idx;
    int chn_idx = ctx->chn_idx;

    ZCAN_20_MSG can_data[RX_BUFF_SIZE];
    ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];

    while (!ctx->stop)
    {
        memset(can_data, 0, sizeof(can_data));
        memset(canfd_data, 0, sizeof(canfd_data));

        int rcount = VCI_Receive(DevType, DevIdx, chn_idx, can_data, RX_BUFF_SIZE, RX_WAIT_TIME);      // CAN
        for (int i = 0; i < rcount; ++i)
        {
            if ((can_data[i].hdr.id & 0x1FFFFFFF) != 0x731 && (can_data[i].hdr.id & 0x1FFFFFFF) != 0x7B1)       // 这里只打印请求相关的报文
                continue;
            printf("[%u] ",can_data[i].hdr.ts);
            printf("chn: %d  ", chn_idx);
            printf("%s  ", can_data[i].hdr.inf.tx == 1 ? "Tx" : "Rx");      // 判断是否回显报文
            printf("ID: 0x%X CAN  ", can_data[i].hdr.id);
            printf("%s  ", can_data[i].hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

            printf("Data: ");
            if(can_data[i].hdr.inf.sdf == 0){       // 数据帧
                for (int j = 0; j < can_data[i].hdr.len; ++j)
                    printf("%02x ", can_data[i].dat[j]);
            }
            printf("\n");
        }
        ctx->total += rcount;

        rcount = VCI_ReceiveFD(DevType, DevIdx, chn_idx, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
        for (int i = 0; i < rcount; ++i)
        {   
            if ((canfd_data[i].hdr.id & 0x1FFFFFFF) != 0x731 && (canfd_data[i].hdr.id & 0x1FFFFFFF) != 0x7B1)       // 这里只打印请求相关的报文
                continue;

            printf("[%u] ",canfd_data[i].hdr.ts);
            printf("chn: %d  ", chn_idx);
            printf("%s  ", canfd_data[i].hdr.inf.tx == 1 ? "Tx" : "Rx");  // 判断是否回显报文
            printf("ID: 0x%x ", canfd_data[i].hdr.id);
            printf("CANFD%s  ", canfd_data[i].hdr.inf.brs == 1 ? "加速" : "");
            printf("%s  ", canfd_data[i].hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

            printf("Data: ");
            for (int j = 0; j < canfd_data[i].hdr.len; ++j)
                printf("%02x ", canfd_data[i].dat[j]);
            printf("\n");
        }
        ctx->total += rcount;
        msleep(10);
    }
    // printf("chn: %d receive %d\n", chn_idx, ctx->total);
    pthread_exit(0);
}

// 通道初始化函数
static int can_start(int DevType, int DevIdx, int chnIdx) { 
    ZCAN_INIT init;         // 波特率结构体，数据根据zcanpro的波特率计算器得出
    init.clk = 60000000;    // clock: 60M(V1.01) 80M(V1.03即以上)
    init.mode = 0;          // 0-正常
    
    init.aset.tseg1 = 14;   // 仲裁域 500kbps
    init.aset.tseg2 = 3;
    init.aset.sjw = 2;
    init.aset.smp = 0;
    init.aset.brp = 5;

    init.dset.tseg1 = 10;   // 数据域 2000kbps
    init.dset.tseg2 = 2;
    init.dset.sjw = 2;
    init.dset.smp = 0;
    init.dset.brp = 1;

    if (!VCI_InitCAN(DevType, DevIdx, chnIdx, &init))    // 初始化通道
    {
        printf("InitCAN(%d) fail\n", chnIdx);
        return 0;
    }
    // printf("InitCAN(%d) success\n", chnIdx);
    
    U32 on = 1;
    if (!VCI_SetReference(DevType, DevIdx, chnIdx, CMD_CAN_TRES, &on)) // 终端电阻
    {   
        printf("CMD_CAN_TRES(%d) fail\n", chnIdx);
    }

    // U32 tx_timeout = 2000; // 2 seconds
    // if (!VCI_SetReference(DevType, DevIdx, 0, CMD_CAN_TX_TIMEOUT, &tx_timeout)) {        // 发送超时时间
    //     printf("CMD_CAN_TX_TIMEOUT failed\n");
    // }

    if (!VCI_StartCAN(DevType, DevIdx, chnIdx))          // 启动通道
    {
        printf("StartCAN(%d) fail\n", chnIdx);
        return 0;
    }
    // printf("StartCAN(%d) success\n", chnIdx);

    rx_ctx[chnIdx].dev_type = DevType;
    rx_ctx[chnIdx].dev_idx = DevIdx;
    rx_ctx[chnIdx].chn_idx = chnIdx;
    rx_ctx[chnIdx].total = 0;
    rx_ctx[chnIdx].stop = 0;
    pthread_create(&rx_threads[chnIdx], NULL, rx_thread, &rx_ctx[chnIdx]); // 创建接收线程
}

// 请求结果判断
void response_print(ZCAN_UDS_RESPONSE resp, U8 *dataBuf){
    switch (resp.status){
	case ZUDS_ERROR_OK:
		if (resp.type == ZCAN_UDS_RT_POSITIVE)          // 积极响应
		{
			printf("积极响应: 服务ID:%X, 参数长度:%d, 参数:", resp.positive.sid, resp.positive.data_len);
            for (U32 i = 0; i < resp.positive.data_len; i++) {
                printf("%02x ", dataBuf[i]);
            }
			printf("\n");
		}
		else if (resp.type == ZCAN_UDS_RT_NEGATIVE)     // 消极响应
		{
			printf("消极响应: %02X %02X %02X", resp.negative.neg_code, resp.negative.sid, resp.negative.error_code);
		}
		break;
	case ZUDS_ERROR_TIMEOUT:
		printf("响应超时");
		break;
	case ZUDS_ERROR_TRANSPORT:
		printf("发送帧数据失败");
		break;
	case ZUDS_ERROR_CANCEL:
		printf("请求中止");
		break;
	case ZUDS_ERROR_SUPPRESS_RESPONSE:
		printf("抑制响应");
		break;
	default:
		printf("其他错误");
		break;
	}
    printf("\n");
}

// UDS 请求
void uds_test(int DevType, int DevIdx, U32 chnIdx, U32 req_id) { 
    ZCAN_UDS_REQUEST request;  
    ZCAN_UDS_RESPONSE resp;
    U8 resp_data[4096];
    U8 data[0x10000];
    memset(&request, 0, sizeof(request));
    memset(&resp, 0, sizeof(resp));
    memset(resp_data, 0, sizeof(resp_data));
    memset(data, 0, sizeof(data));

    request.req_id = req_id;        // 请求事务ID
    request.channel = chnIdx;       // 通道号
    if(!can_or_canfd)
        request.frame_type = ZCAN_UDS_FRAME_CAN;    // 帧类型
    else
        request.frame_type = ZCAN_UDS_FRAME_CANFD;  // ZCAN_UDS_FRAME_CANFD_BRS
    request.src_addr = 0x731;       // 物理地址
    request.dst_addr = 0x7B1;       // 响应地址
    request.suppress_response = 0;  // 1-抑制响应 
    request.sid = 0x10;             // 请求服务id

    request.session_param.timeout = 2000;                   // P2 响应超时时间(ms)
    request.session_param.enhanced_timeout = 5000;          // P2* 收到消极响应错误码为0x78后的超时时间(ms)
    request.session_param.check_any_negative_response = 0;
    request.session_param.wait_if_suppress_response = 0;

    if(!can_or_canfd){
        request.trans_param.version = ZCAN_UDS_TRANS_VER_0;     // ISO-15765-2004
        request.trans_param.max_data_len = 8;                   // 帧数据最长长度
    }
    else{
        request.trans_param.version = ZCAN_UDS_TRANS_VER_0;     // ISO-15765-2016
        request.trans_param.max_data_len = 64;
    }
    
    request.trans_param.local_st_min = 0;           // 连续帧之间的最小间隔
    request.trans_param.block_size= 0;              // 流控帧的块大小
    request.trans_param.fill_byte = 0x00;           // 无效字节的填充数据
    request.trans_param.ext_frame = 0;              // 0-标准帧 1-扩展帧
    request.trans_param.is_modify_ecu_st_min = 0;   // 是否忽略ECU返回流控的STmin
    request.trans_param.remote_st_min = 0;          // 忽略后，多帧时的帧间隔
    request.trans_param.fc_timeout = 1000;              // 等待流控超时时间(ms)
    request.trans_param.fill_mode = FILL_MODE_SHORT;    // 0-就近填充，1-不填充，2全填充

    // 构造请求数据
    request.data = data;
    request.data_len = 1;
    if (request.data_len >sizeof(data)) {
        request.data_len = sizeof(data); 
    }
    data[0] = (U8)0x03;

    // UDS请求
    if (VCI_UDS_Request(DevType, DevIdx, &request, &resp, resp_data, sizeof(resp_data)) == 1)
        response_print(resp, resp_data);  // 打印结果
}

int main(int argc, char* argv[]) {
    int DevType = USBCANFD;    // 设备类型号 33-usbcanfd
    int DevIdx = 0;                 // 设备索引号
    int chnIdx = 0;                 // 通道号

    // 打开设备
    if (!VCI_OpenDevice(DevType, DevIdx, 0)) {
        printf("Open device fail\n");
        return 0;
    }
    printf("Open device success\n");

    // 通道初始化
    can_start(DevType, DevIdx, chnIdx);  

    // UDS 请求
    uds_test(DevType, DevIdx, chnIdx, 0);

    // 阻塞等待
    getchar();
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        rx_ctx[i].stop = 1;
        pthread_join(rx_threads[i], NULL);
    }

    // 复位通道
    if (!VCI_ResetCAN(DevType, DevIdx, chnIdx))
        printf("ResetCAN(%d) fail\n", chnIdx);
        
    // 关闭设备
    VCI_CloseDevice(DevType, DevIdx);
    printf("Close device success\n");
    return 0;
}
