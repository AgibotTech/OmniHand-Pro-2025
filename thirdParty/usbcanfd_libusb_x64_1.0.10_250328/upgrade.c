#include "zcan.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "unistd.h"

int main(int argc, char* argv[]) {
    int dev_type = 33;
    if (argc < 1) {
        printf("no firmware!\n");
        return -1;
    }

    int dev_idx = 0;
    if (argc > 2) {
        dev_idx = atoi(argv[2]);
    }

    const char *fw_path = argv[1];
    printf("firmware:%s, device_idx=%d\n", fw_path, dev_idx);

    if (!VCI_OpenDevice(dev_type, 0, 0)) {
        printf("VCI_OpenDevice failed\n");
        return -1;
    }

    printf("VCI_OpenDevice succeeded\n");

    int ret = VCI_SetReference(dev_type, dev_idx, 0, 0xFF, (void *)fw_path);
    if (!ret) {
        printf("upgrade failed!\n");
    } else {
        printf("upgrade success, wait device restart!\n");
    }

    VCI_CloseDevice(dev_type, dev_idx);

    printf("VCI_CloseDevice\n");

    return 0;
}
