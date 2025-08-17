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

#define USBCANFD 33
#define MAX_CHANNELS  2
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

// 接收线程
void* rx_thread(void *data)
{
    THREAD_CTX *ctx = (THREAD_CTX *)data;
    int DevType = ctx->dev_type;
    int DevIdx  = ctx->dev_idx;
    int chn_idx = ctx->chn_idx;

    ZCAN_LIN_MSG buff[RX_BUFF_SIZE]; // buffer

    while (!ctx->stop)
    {
        int rcount = VCI_ReceiveLIN(DevType, DevIdx, chn_idx, buff, RX_BUFF_SIZE, RX_WAIT_TIME);
        for (int i = 0; i < rcount; ++i)
        {
            if (0 == buff[i].dataType)
            { // 只显示LIN数据
                ZCANLINData ld = buff[i].data.zcanLINData;
                printf("[%llu] ",ld.RxData.timeStamp);
                printf("LIN%d ",(int)buff[i].chnl);      
                printf("%s ",ld.RxData.dir == 1 ? "TX" : "RX");
                printf(" ID: 0x%02X  ", ld.PID.unionVal.ID);
                printf("len:%d ", ld.RxData.dataLen);
                printf("Data: ");

                for (int j = 0; j < 8; ++j)
                    printf("%X ", ld.RxData.data[j]);
                printf("\n");
            }
        }
    }
    pthread_exit(0);
}

// 演示 USBCANFD-200U LIN 通道对接通讯
int main(int argc, char* argv[])
{
    int DevType = USBCANFD;    // 设备类型号 33-usbcanfd
    int DevIdx = 0;                 // 设备索引号

    THREAD_CTX rx_ctx[MAX_CHANNELS];
    pthread_t rx_threads[MAX_CHANNELS]; // 接收线程

    // 打开设备
    if (!VCI_OpenDevice(DevType, DevIdx, 0)) {
        printf("Open device fail\n");
        return 0;
    }
    printf("Open device success\n");

    // 初始化并打开LIN通道，LIN0设置为主，LIN1设置为从，波特率设置为9600
	ZCAN_LIN_INIT_CONFIG LinCfg[2];
	LinCfg[0].linMode = 1;      // 0-从机 1-主机
	LinCfg[0].linBaud = 9600;   // 波特率 取值1000~20000
	LinCfg[0].chkSumMode = 1;	// 1-经典校验 2-增强校验 3-自动
	LinCfg[1].linMode = 0;
	LinCfg[1].linBaud = 9600;
	LinCfg[1].chkSumMode = 1;
    
	for (int i = 0; i < MAX_CHANNELS; ++i)
	{
		if (!VCI_InitLIN(DevType, DevIdx, i, &LinCfg[i]))     // 初始化 LIN 通道
		{
            printf("init LIN %d fail\n",i);
            return 0;
		}
        printf("init LIN %d success\n",i);

		if (!VCI_StartLIN(DevType, DevIdx, i))                // 启动 LIN 通道
		{
			printf("start LIN %d fail\n",i);
		}
		printf("start LIN %d success\n",i);

        rx_ctx[i].dev_type = DevType;
        rx_ctx[i].dev_idx = 0;
        rx_ctx[i].chn_idx = i;
        rx_ctx[i].total = 0;
        rx_ctx[i].stop = 0;
        pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);
	}

    // LIN0 作为主站时，需要使能 LIN 终端电阻，以下设备的销售版本才支持函数调用设置电阻
    // USBCANFD-200U 销售版本 V1.04
    // USBCANFD-100U 销售版本 V1.03
    // USBCANFD-400U 销售版本 V1.00
    // U32 on = 1;
    // if (!VCI_SetReference(DevType, DevIdx, 0, CMD_LIN_INTERNAL_RESISTANCE, &on))   // LIN 通道终端电阻
    // {
    //     printf("CMD_LIN_INTERNAL_RESISTANCE failed\n");
    // }

    // 
	msleep(1000);

    // 设置 LIN1（从）ID 0 - 2 的响应，LIN0（主）ID 3 - 4 的响应
	ZCAN_LIN_PUBLISH_CFG lpc[5];
	for (int i = 0; i < 5; ++i)
	{
		lpc[i].ID = i;
		lpc[i].chkSumMode = 0;          // 校验
		lpc[i].dataLen = 8;
		for (int j = 0; j < 8; ++j)
			lpc[i].data[j] = i;
	}
	if (!VCI_SetLINPublish(DevType, DevIdx, 0, &lpc[0], 3))     // 主站响应，就是对应ZXDOC软件中的“头部和响应”
		printf("set LIN1 publish failed!\n");

	if (!VCI_SetLINPublish(DevType, DevIdx, 1, &lpc[3], 2))     // 从站响应
		printf("set LIN1 publish failed!\n");

	// LIN0(主)发送 ID 0 - 4 的头部
	ZCAN_LIN_MSG  send_data[5] = {};
	for (int i = 0; i < 5; i++)
	{
		send_data[i].dataType = 0;
		send_data[i].data.zcanLINData.PID.rawVal = i;
		send_data[i].chnl = 0;  //只有主站才能发数据
	}
	int scount = VCI_TransmitLIN(DevType, DevIdx, 0, send_data, 5); //count真实发送帧数
	printf("Send LIN count : %d\n", scount);
	msleep(1000);

    // 阻塞等待
    getchar();
    for (int i = 0; i < MAX_CHANNELS; i++) {
        rx_ctx[i].stop = 1;
        pthread_join(rx_threads[i], NULL);
        if (!VCI_ResetLIN(DevType, DevIdx, i))
            printf("ResetLIN(%d) fail\n", i);
        else
            printf("ResetLIN(%d) success!\n", i);
    }

    // 关闭设备
    if (!VCI_CloseDevice(DevType, DevIdx))
        printf("CloseDevice fail\n");
    else
        printf("CloseDevice success\n");
    return 0;
}

