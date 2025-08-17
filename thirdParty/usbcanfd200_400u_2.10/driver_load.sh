#!/bin/bash
driver_subcanfd="usbcanfd"
driver_candev="can_dev"
CAN_INTERFACES=(can0 can1 can2 can3)
bitrate="bitrate 500000"
dbitrate="dbitrate 2000000"
# 1是开启终端电阻
Terminal_resistance=1


pid_cangen=$(ps -ef | grep "cangen" | grep -v grep |awk '{print $2}')
if [[ -z "$pid_cangen" ]]; then
  echo "没有正在运行的cangen"
else
  sudo kill $pid_cangen
  echo "已杀杀死cangen $pid_cangen"
fi

pid_dump=$(ps -ef | grep "candump" | grep -v grep | awk '{print $2}')
if [[ -z "$pid_dump" ]]; then
  echo "没有正在运行的candunmp"
else
  sudo kill $pid_dump
  echo "已杀杀死candump $pid_dump"
fi

pid_send=$(ps -ef | grep "cansend" | grep -v grep | awk '{print $2}')
if [[ -z "$pid_send" ]]; then
  echo "没有正在运行的candsend"
else
  sudo kill $pid_send
  echo "已杀杀死cansend $pid_send"
fi

# 获取系统中所有的 CAN 设备名称
CAN_DEVICES=$(ifconfig | grep -o "can[0-9]\+" | sed 's/://g')

# 遍历所有的 CAN 设备，并将其关闭
for DEV in $CAN_DEVICES; do
    # 如果设备处于运行状态，就将其关闭
    if [[ $(ifconfig $DEV | grep RUNNING) ]]; then
        sudo ifconfig $DEV down
        echo "关闭 CAN 设备 $DEV"
    fi
done

echo "CAN 设备关闭完毕"

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# 切换工作目录
cd "$SCRIPT_DIR"

# 检查驱动是否已经加载
if lsmod | grep "$driver_subcanfd" &> /dev/null ; then
    echo "Unloading $driver_subcanfd"
    # 卸载驱动
    sudo rmmod "$driver_subcanfd.ko"
fi


if lsmod | grep "$driver_candev" &> /dev/null ; then
    echo "Unloading $driver_candev"
    # 卸载驱动
    sudo rmmod "$driver_candev"
fi

# 加载驱动
echo "Loading $driver_candev"
sudo modprobe "$driver_candev"

echo "Loading $driver_subcanfd"
sudo insmod "$driver_subcanfd.ko"  cfg_term_res=$Terminal_resistance dbg=1

for interface in "${CAN_INTERFACES[@]}"
do
    if [ -e "/sys/class/net/$interface" ]; then
        echo "CAN network $interface found."
        sudo ip link set $interface up type can fd on $bitrate $dbitrate sample-point 0.8 dsample-point 0.8
        echo "CAN network $interface bitrate set $bitrate $dbitrate"
        sudo chmod 777 /sys/class/net/$interface/tx_queue_len
        sudo echo 66536 > /sys/class/net/$interface/tx_queue_len
    else
        echo "CAN network $interface not found."
    fi
done

