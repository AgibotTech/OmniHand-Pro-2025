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

#define USBCANFD  33
#define MAX_CHANNELS  1     // 最大通道数量
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000

// 接收线程上下文
typedef struct
{
    int dev_type;   // 设备类型
    int dev_idx;    // 设备索引
    int chn_idx;    // 通道号
    int total;      // 接收总数
    int stop;       // 线程结束标志
} THREAD_CTX;

// 构建 CAN 帧
void construct_can_frame(ZCAN_20_MSG *can_msg, UINT id, int chn_idx, int pad) {
    memset(can_msg, 0, sizeof(ZCAN_20_MSG));
    can_msg->hdr.inf.txm = 0;     // 0-正常发送
    can_msg->hdr.inf.fmt = 0;     // 0-CAN
    can_msg->hdr.inf.sdf = 0;     // 0-数据帧, 1-远程帧
    can_msg->hdr.inf.sef = 0;     // 0-标准帧, 1-扩展帧
    // can_msg->hdr.inf.echo = 1;    // 发送回显

    can_msg->hdr.id  = id;        // ID
    can_msg->hdr.chn = chn_idx;   // 通道
    can_msg->hdr.len = 8;        // 数据长度

    // 队列发送
    if(pad > 0){
        can_msg->hdr.pad = pad;              // 发送后延迟 pad ms
        can_msg->hdr.inf.qsend = 1;          // 队列发送帧，仅判断首帧
        can_msg->hdr.inf.qsend_100us = 0;    // 队列发送单位，0-ms，1-100us
    }

    for (int i = 0; i < can_msg->hdr.len; i++)
        can_msg->dat[i] = i;
}

// 构建 CANFD 帧
void construct_canfd_frame(ZCAN_FD_MSG *canfd_msg, UINT id, int chn_idx, int pad) {
    memset(canfd_msg, 0, sizeof(ZCAN_FD_MSG));
    canfd_msg->hdr.inf.txm = 0;     // 0-正常发送
    canfd_msg->hdr.inf.fmt = 1;     // 0-CAN, 1-CANFD
    canfd_msg->hdr.inf.sdf = 0;     // 0-数据帧 CANFD只有数据帧!
    canfd_msg->hdr.inf.sef = 1;     // 0-标准帧, 1-扩展帧
    canfd_msg->hdr.inf.brs = 1;     // canfd 加速
    // canfd_msg->hdr.inf.echo  = 1;   // 发送回显

    canfd_msg->hdr.id  = id;        // ID
    canfd_msg->hdr.chn = chn_idx;   // 通道
    canfd_msg->hdr.len = 64;        // 数据长度

    // 队列发送
    if(pad > 0){
        canfd_msg->hdr.pad = 100;              //发送后延迟100ms
        canfd_msg->hdr.inf.qsend = 1;          //队列发送帧，仅判断首帧
        canfd_msg->hdr.inf.qsend_100us = 0;    //队列发送单位，0-ms，1-100us
    }

    for (int i = 0; i < canfd_msg->hdr.len; i++)
        canfd_msg->dat[i] = i;
}

// 构建 合并发送 帧
void construct_data_frame(ZCANDataObj *data_msg, UINT id, int chn_idx, int pad){
    memset(data_msg, 0, sizeof(data_msg));
    data_msg->dataType = ZCAN_DT_ZCAN_CAN_DATA;
    data_msg->chnl = chn_idx;
    construct_can_frame(& data_msg->data.zcanCANData, id, chn_idx, pad);
}

// 获取设备信息
void get_device_info(int DevType, int DevIdx) {
    ZCAN_DEV_INF info;
    memset(&info, 0, sizeof(info));
    VCI_ReadBoardInfo(DevType, DevIdx, &info);
    char sn[21];
    char id[41];
    memcpy(sn, info.sn, 20);
    memcpy(id, info.id, 40);
    sn[20] = '\0';
    id[40] = '\0';
    printf("HWV=0x%04x, FWV=0x%04x, DRV=0x%04x, API=0x%04x, IRQ=0x%04x, CHN=0x%02x, SN=%s, ID=%s\n",
        info.hwv, info.fwv, info.drv, info.api, info.irq, info.chn, sn, id);
}

// 设备 SN 号
void test_sn(int DevType, int DevIdx) {
    // // 设置 SN 号
    // char sn[128] = {0};
    // strcpy(sn, "ABC");
    // if (!VCI_SetReference(DevType, DevIdx, 0, CMD_SET_SN, (void *)sn)) {
    //     printf("CMD_SET_SN failed\n");
    // }
    // printf("set_sn:%s\n", sn);

    // 获取 SN 号
    char get_sn[128] = {0};
    if (!VCI_GetReference(DevType, DevIdx, 0, CMD_GET_SN, (void *)get_sn)) {
        printf("CMD_GET_SN failed\n");
    }
    printf("get_sn:%s\n", get_sn);
}

// 接收线程
void *rx_thread(void *data) {
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
            printf("[%u] ",can_data[i].hdr.ts);
            printf("chn: %d  ", chn_idx);
            printf("%s  ", can_data[i].hdr.inf.tx == 1 ? "Tx" : "Rx");    // 判断是否回显报文
            printf("CAN ID: 0x%X ", can_data[i].hdr.id & 0x1FFFFFFF);
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
            printf("[%u] ",canfd_data[i].hdr.ts);
            printf("chn: %d  ", chn_idx);
            printf("%s  ", canfd_data[i].hdr.inf.tx == 1 ? "Tx" : "Rx");  // 判断是否回显报文
            printf("CANFD%s  ", canfd_data[i].hdr.inf.brs == 1 ? "加速" : "");
            printf("ID: 0x%x ", canfd_data[i].hdr.id & 0x1FFFFFFF);
            printf("%s  ", canfd_data[i].hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

            printf("Data: ");
            for (int j = 0; j < canfd_data[i].hdr.len; ++j)
                printf("%02x ", canfd_data[i].dat[j]);
            printf("\n");
        }
        ctx->total += rcount;
        msleep(10);
    }
    printf("chn: %d receive %d\n", chn_idx, ctx->total);
    pthread_exit(0);
}

// 合并接收线程
void *rx_merge_thread(void *data) {
    THREAD_CTX *ctx = (THREAD_CTX *)data;
    int DevType = ctx->dev_type;
    int DevIdx  = ctx->dev_idx;
    //int chn_idx = ctx->chn_idx;

    ZCANDataObj obj_data[RX_BUFF_SIZE];
    ZCAN_20_MSG can_data;
    ZCAN_FD_MSG canfd_data;

    while (!ctx->stop)
    {
        memset(obj_data, 0, sizeof(obj_data));
        memset(&can_data, 0, sizeof(can_data));
        memset(&canfd_data, 0, sizeof(canfd_data));
        int rcount = VCI_ReceiveData(DevType, DevIdx, 0, obj_data, RX_BUFF_SIZE, RX_WAIT_TIME);     // 第三个参数固定为0就好
        for (int i = 0; i < rcount; ++i)
        {
            if(obj_data[i].dataType == ZCAN_DT_ZCAN_CAN_DATA){              // CAN
                can_data = obj_data[i].data.zcanCANData;
                printf("[%u] ",can_data.hdr.ts);
                printf("chn: %d  ", obj_data[i].chnl);
                printf("%s  ", can_data.hdr.inf.tx == 1 ? "Tx" : "Rx");     // 判断是否回显报文
                printf("ID: 0x%X CAN  ", can_data.hdr.id);
                printf("%s  ", can_data.hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

                printf("Data: ");
                if(can_data.hdr.inf.sdf == 0){       // 数据帧
                    for (int j = 0; j < can_data.hdr.len; ++j)
                        printf("%02x ", can_data.dat[j]);
                }
            }
            else if(obj_data[i].dataType == ZCAN_DT_ZCAN_CANFD_DATA){       // CANFD
                printf("[%u] ",obj_data[i].data.zcanCANFDData.hdr.ts);
                printf("chn: %d  ", obj_data[i].chnl);
                printf("%s  ", obj_data[i].data.zcanCANFDData.hdr.inf.tx == 1 ? "Tx" : "Rx");  // 判断是否回显报文
                printf("ID: 0x%x ", obj_data[i].data.zcanCANFDData.hdr.id);
                printf("CANFD%s  ", obj_data[i].data.zcanCANFDData.hdr.inf.brs == 1 ? "加速" : "");
                printf("%s  ", obj_data[i].data.zcanCANFDData.hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

                printf("Data: ");
                for (int j = 0; j < obj_data[i].data.zcanCANFDData.hdr.len; ++j)
                    printf("%02x ", obj_data[i].data.zcanCANFDData.dat[j]);
            }
            printf("\n");
        }
        ctx->total += rcount;
        msleep(10);
    }
    printf("merge receive %d\n", ctx->total);
    pthread_exit(0);
}

// 普通发送
void send_test(int DevType, int DevIdx, int ChIdx){
    // 测试发送
    const int send_num = 10;    // 发送数量
    int send_ret = 0;           // 发送返回值

    // CAN
    ZCAN_20_MSG can_msg[send_num];  // 数组不规范写法，仅供参考
    for (int i = 0; i < send_num; i++)
        construct_can_frame(&can_msg[i], i, ChIdx, 0);
    send_ret = VCI_Transmit(DevType, DevIdx, ChIdx, &can_msg[0], send_num);
    printf("send can frams %d\n", send_ret);

    // CANFD
    ZCAN_FD_MSG canfd_msg[send_num];        
    for (int i = 0; i < send_num; i++)
        construct_canfd_frame(&canfd_msg[i], i, ChIdx, 0);
    send_ret = VCI_TransmitFD(DevType, DevIdx, ChIdx, &canfd_msg[0], send_num);
    printf("send canfd frams %d\n", send_ret);

    // 合并发送
    ZCANDataObj data_msg[send_num];
    for (int i = 0; i < send_num; i++)
        construct_data_frame(&data_msg[i], i, ChIdx, 0);
    send_ret = VCI_TransmitData(DevType, DevIdx, ChIdx, &data_msg[0], send_num);
    printf("send frams %d\n", send_ret);
}

// 队列发送
void queue_send(int DevType, int DevIdx, int ChIdx) {
    unsigned int snd_queue_size = 0;    // 队列大小
    unsigned int snd_queue_remain = 0;  // 队列剩余空间

    // 开启队列发送（带LIN版本无需调用）
    VCI_SetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_SET_SEND_QUEUE_EN, (void *)1);
    
    // 获取队列大小，最多填充个数
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SIZE, &snd_queue_size);
    
    // 获取队列剩余可填充报文帧数
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
    printf("Chn%d send queue size:%u, remain space:%u\n", ChIdx, snd_queue_size, snd_queue_remain);

    // 发送 10 次100帧的报文
    int transmit_time = 10;
    int transmit_num = 100;
    ZCAN_20_MSG can_msg[100];
    ZCAN_FD_MSG canfd_msg[100];
    while(1){
        // 队列发送前需要判断可填充报文帧数!!
        VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);   
        if(snd_queue_remain >= transmit_num){                       
            for (int i = 0; i < 100; i++)
                construct_can_frame(&can_msg[i], i, ChIdx, 100);   // 10ms

            unsigned ret = VCI_Transmit(DevType, DevIdx, ChIdx, can_msg, 100);        // CAN
            printf("Chn%d remain space:%u, queue send canfd frame %u\n", ChIdx, snd_queue_remain, ret);
            
            transmit_time--;
            if(transmit_time == 0)
                break;
        }

        // VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);   
        // if(snd_queue_remain >= transmit_num){
        //     for (int i = 0; i < 100; i++)
        //         construct_canfd_frame(&canfd_msg[i], i, ChIdx, 100);   // 10ms
        //
        //     unsigned ret = VCI_TransmitFD(DevType, DevIdx, ChIdx, canfd_msg, 100);        // CANFD
        //     printf("Chn%d queue send canfd frame %u, ret=%u\n", ChIdx, 100, ret);
        //  
        //     transmit_time--;
        //     if(transmit_time == 0)
        //         break;
        // }
    }

    sleep(3);

    //发送后，队列空间变化
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
    printf("Chn%d remain space:%u\n", ChIdx, snd_queue_remain);

    //清空队列发送
    VCI_SetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_SET_SEND_QUEUE_CLR, (void *)1);
    printf("Chn%d send queue clear\n", ChIdx);

    //清空队列后，队列空间恢复初始值
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
    printf("Chn%d remain space:%u\n", ChIdx, snd_queue_remain);
}

// 定时发送
void auto_send(int DevType, int DevIdx, int ChIdx) {
    int autosend_num = 2;    // (最多8条)

    // 定时发送
    ZCAN_TTX_CFG autosend_cfg;
    autosend_cfg.size = sizeof(ZCAN_TTX) * autosend_num;  // 数量

    for (int i = 0; i< autosend_num; ++i) {
        autosend_cfg.table[i].interval = 5000;   // 500ms，单位 100 us
        autosend_cfg.table[i].repeat = 0;        // 0-一直发
        autosend_cfg.table[i].index = i;         // 索引
        autosend_cfg.table[i].flags = 1;         // 使能
        
        autosend_cfg.table[i].msg.hdr.id = 0x100 + i;
        autosend_cfg.table[i].msg.hdr.inf.txm = 0;   // 0为正常模式，2为自发自收
        autosend_cfg.table[i].msg.hdr.inf.fmt = 0;   // 0-CAN帧，1-CANFD帧
        autosend_cfg.table[i].msg.hdr.inf.sdf = 0;   // 0-数据帧，1-远程帧
        autosend_cfg.table[i].msg.hdr.inf.sef = 0;   // 0-标准帧，1-扩展帧
        autosend_cfg.table[i].msg.hdr.inf.brs = 0;   // 0-CANFD不加速，1-CANFD加速
        autosend_cfg.table[i].msg.hdr.pad = 0;
        autosend_cfg.table[i].msg.hdr.chn = 0;
        autosend_cfg.table[i].msg.hdr.len = 8;
        for (int j = 0; j< autosend_cfg.table[i].msg.hdr.len; ++j)
            autosend_cfg.table[i].msg.dat[j] = j;
    }

    int ret = VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TTX, &autosend_cfg);
    if(ret == 0)
        printf("设置定时发送 失败\n");
    else
        printf("设置定时发送 成功\n");
    
    int on = 1;
    ret = VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TTX_CTL, &on);
    if (ret == 0)
        printf("使能定时发送 失败\n");
    else
        printf("使能定时发送 成功\n");

    msleep(3000);

    on = 0;
    ret = VCI_SetReference(DevType, DevIdx, ChIdx, CMD_CAN_TTX_CTL, &on);
    if (ret == 0)
        printf("关闭定时发送 失败\n");
    else
        printf("关闭定时发送 成功\n");
}


int main(int argc, char* argv[])
{
    int DevType = USBCANFD;    // 设备类型号 33-usbcanfd
    int DevIdx = 0;                 // 设备索引号
    int IsMerge = 0;                // 是否合并接收

    THREAD_CTX rx_ctx[MAX_CHANNELS];        // 接收线程上下文
    pthread_t rx_threads[MAX_CHANNELS]; // 接收线程

    // 打开设备
    if (!VCI_OpenDevice(DevType, DevIdx, 0)) {
        printf("Open device fail\n");
        return 0;
    }
    printf("Open device success\n");

    // 初始化，启动通道
    for (int i = 0; i < MAX_CHANNELS; i++){
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

        if (!VCI_InitCAN(DevType, DevIdx, i, &init))    // 初始化通道
        {
            printf("InitCAN(%d) fail\n", i);
            return 0;
        }
        printf("InitCAN(%d) success\n", i);
        
        U32 on = 1;
        if (!VCI_SetReference(DevType, DevIdx, i, CMD_CAN_TRES, &on)) // 终端电阻
        {   
            printf("CMD_CAN_TRES fail\n");
        }

        if(i == 0){
            if (!VCI_SetReference(DevType, DevIdx, i, ZCAN_CMD_SET_CHNL_RECV_MERGE, &IsMerge)) // 合并接收
            {   
                printf("ZCAN_CMD_SET_CHNL_RECV_MERGE fail\n");
            }
        }

        if (!VCI_StartCAN(DevType, DevIdx, i))          // 启动通道
        {
            printf("StartCAN(%d) fail\n", i);
            return 0;
        }
        printf("StartCAN(%d) success\n", i);

        rx_ctx[i].dev_type = DevType;
        rx_ctx[i].dev_idx = DevIdx;
        rx_ctx[i].chn_idx = i;
        rx_ctx[i].total = 0;
        rx_ctx[i].stop = 0;
        
        if(IsMerge && i == 0){
            pthread_create(&rx_threads[i], NULL, rx_merge_thread, &rx_ctx[i]);  // 合并接收线程
        }
        else{
            pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);        // 创建接收线程
        }
    }

    // 发送测试
    send_test(DevType, DevIdx, 0);

    // // 队列发送
    // queue_send(DevType, DevIdx, 0);

    // // 定时发送
    // auto_send(DevType, DevIdx, 0);

    // 阻塞等待
    getchar();
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        rx_ctx[i].stop = 1;
        pthread_join(rx_threads[i], NULL);          // 等待线程退出

        if (!VCI_ResetCAN(DevType, DevIdx, i))      // 复位通道
            printf("ResetCAN(%d) fail\n", i);
        else
            printf("ResetCAN(%d) success!\n", i);
    }

    // 关闭设备
    if (!VCI_CloseDevice(DevType, DevIdx))
        printf("CloseDevice fail\n");
    else
        printf("CloseDevice success\n");
    return 0;
}

