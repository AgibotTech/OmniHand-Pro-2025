#!/bin/bash

#########################CAN的配置##############################
driver_subcanfd="usbcanfd"
driver_candev="can_dev"
bitrate="bitrate 500000"
dbitrate="dbitrate 2000000"
# 1是开启终端电阻
Terminal_resistance=1

#########################LIN的配置##############################
#如果插入多张卡，每一张卡的配置需要一致，如果需要不一样的配置就需要手动修改#

IsMaster_lin0=0  #0是从机，1是主机
IsMaster_lin1=0  #0是从机，1是主机

Baudrate_lin0=19200 #LIN波特率
Baudrate_lin1=19200

Feature_lin0=2  #0为经典校验，1为增强校验，2为自动校验
Feature_lin1=2

TrEnable_lin0=0 #0为关闭LIN终端电阻，1为开启Lin终端电阻
TrEnable_lin1=0 


configure_can_networks(){
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
}

configure_lin_networks() {
    ifconfig_output=$(ifconfig -a)
    lin_count=$(echo "$ifconfig_output" | grep -o '^lin[0-9]:' | wc -l)

    if [ "$lin_count" -gt 0 ]; then
        for ((i=0; i<lin_count; i++)); do
            sudo ip link set lin$i up type can fd on bitrate 1000000 dbitrate 1000000
        done
    else
        echo "LIN network not found"
    fi
}

CAN_INTERFACES=()  # 创建一个空数组


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
sudo insmod "$driver_subcanfd.ko"  cfg_term_res=$Terminal_resistance  dbg=1

lsusb_output=$(lsusb)
json_file="bind_config.json"
device_name=""
usbcanfd_dir=""
index=()
can_ports=""
j=0
k=0

if [[ $lsusb_output == *"USBCANFD-200U"* ]]; then
    device_name="usbcanfd200u"
    usbcanfd_dir="/sys/bus/usb/drivers/usbcanfd200u"
    j=1
fi

if [[ $lsusb_output == *"USBCANFD-400U"* ]]; then
    device_name="usbcanfd400u"
    usbcanfd_dir="/sys/bus/usb/drivers/usbcanfd"
    j=3
fi


if [[ $lsusb_output == *"USBCANFD-800U"* ]]; then
    device_name="usbcanfd800u"
    usbcanfd_dir="/sys/bus/usb/drivers/usbcanfd800u"
    j=7
fi

if [ -z "$device_name" ]; then
    echo "ZLGUSBCAN没有设备"
    exit 1  
fi

count=$(echo "$lsusb_output" | grep -c "USBCANFD*")
echo "$device_name 设备的数量为：$count"

# 使用 for 循环生成 can0 到 can16 并添加到数组中
for ((i=0; i<((j+1)*count); i++)); do
    CAN_INTERFACES+=("can$i")
done



k=$j
for ((i=0; i<$count; i++)); do
    sn_list[i]=$(cat "/sys/bus/usb/devices/$(readlink /sys/class/net/can$k/device | sed 's@.*/\([^/]\+\):.*@\1@g')/serial")
    k=$((k + j))
    echo "Serial 值为: ${sn_list[i]}"
done

files_and_dirs=$(ls "$usbcanfd_dir")
bus_list=()
for item in $files_and_dirs; do
    if [[ "$item" =~ ^[0-9] ]]; then
        bus_list+=("$item")
    fi
done

for bus in "${bus_list[@]}"; do
    echo "$bus"
done

file_list=()
for item in $files_and_dirs; do
    if [[ "$item" =~ ^[0-9] ]]; then
        file_list+=("$item")
    fi
done

k=$j

set_lin_config() {
    for ((i=0; i<$count; i++)); do
        sudo chmod 777 $usbcanfd_dir/${file_list[i]}/lin0_cfg
        sudo chmod 777 $usbcanfd_dir/${file_list[i]}/lin1_cfg   
        echo "{\"LIN0\":{\"Enable\":1,\"IsMaster\":${IsMaster_lin0},\"Baudrate\":${Baudrate_lin0},\"Feature\":${Feature_lin0},\"TrEnable\":${TrEnable_lin0},\"DataLen\":8}}" > $usbcanfd_dir/${file_list[i]}/lin0_cfg
        echo "{\"LIN1\":{\"Enable\":1,\"IsMaster\":${IsMaster_lin1},\"Baudrate\":${Baudrate_lin1},\"Feature\":${Feature_lin1},\"TrEnable\":${TrEnable_lin1},\"DataLen\":8}}" > $usbcanfd_dir/${file_list[i]}/lin1_cfg
    done

}

if [ "$count" -lt 2 ]; then
    echo "设备数量只有一个，不需要绑定功能"
    configure_can_networks
    set_lin_config
    configure_lin_networks
    exit 1  
fi


# 定义一个二维数组
declare -A can_dev

 for ((i=0; i<$count; i++)); do
    # 0:index, 1:bus,2:SN
    can_dev[$i,0]="$i"
    for ((w=0; w<$count; w++)); do
        file_path=$usbcanfd_dir/${file_list[w]}
        result=$(awk -F ',' -v col=$((j + 1)) '{print $col}' "${file_path}/can_ports" | head -n 1)
        result=$(echo "$result" | sed 's/[^0-9]//g')
         echo "The result is: $result"
        if [ "$result" = "$k" ]; then
        can_dev[$i,1]="${file_list[w]}"
        can_ports[i]=$(cat ${file_path}/can_ports)
        fi
    done
    can_dev[$i,2]="$(cat "/sys/bus/usb/devices/$(readlink /sys/class/net/can$k/device | sed 's@.*/\([^/]\+\):.*@\1@g')/serial")"
    k=$((k + j+1))
 done



# 检查JSON文件是否存在
if [ ! -f "$json_file" ]; then
  echo "JSON文件不存在，将创建一个新的文件。"
  echo "{}" > "$json_file"
  
  json_data="["

for ((i=0; i<$count; i++)); do
    json_data+="{"
    for ((j=0; j<4; j++)); do
    
    case "$j" in
    "0")
        json_data+="\"index\":\"${can_dev[$i,$j]}\","
        ;;
    "1")
        json_data+="\"bus\":\"${can_dev[$i,$j]}\","
        ;;
    "2")
        json_data+="\"SN\":\"${can_dev[$i,$j]}\","
        ;;
    "3")
        json_data+="\"can_ports\":\"${can_ports[$i]}\","
        ;;    
    *)
        echo "未知选项"
        ;;
    esac
    
    done
  json_data="${json_data%,}"
  json_data+="},"
done

    json_data="${json_data%,}"
    json_data+="]"
    echo "$json_data" | jq '.' > "$json_file"
    echo "JSON数据已写入$json_file文件。"
    configure_can_networks
    set_lin_config
    configure_lin_networks
    exit 1
else
    echo "配置文件已经存在"
fi

declare -A can_dev_read
for ((i=0; i<$count; i++)); do
    can_dev_read[$i,0]=$(jq -r --arg i "$i" '.['"$i"'].index' "$json_file")
    can_dev_read[$i,1]=$(jq -r --arg i "$i" '.['"$i"'].bus' "$json_file")
    can_dev_read[$i,2]=$(jq -r --arg i "$i" '.['"$i"'].SN' "$json_file")
done


for ((i=0; i<$count; i++)); do
    if [[ "${can_dev[$i,2]}" != "${can_dev_read[$i,2]}" ]]; then
       echo "索引号与配置不相同，重新调整"
       break 
    else
        echo "索引号与配置相同,不需要调整" 
        configure_can_networks
        set_lin_config
        configure_lin_networks
        exit 1
    fi 
done

unbind="$usbcanfd_dir/unbind"
bind="$usbcanfd_dir/bind"
sudo chmod  777  $unbind
sudo chmod  777  $bind


for ((i=0; i<$count; i++)); do
    echo "${can_dev[$i,1]}" > $unbind
done

    for ((w=0; w< $count; w++)); do
        for ((j=0; j<$count; j++)); do
         if [[ "${can_dev[$j,2]}" == "${can_dev_read[$w,2]}" ]]; then
            can_dev_change[$w]=${can_dev[$j,1]}
         fi
        done
    done


for ((i=0; i<$count; i++)); do
    echo "${can_dev_change[$i]}" > $bind
done


sleep 1
configure_can_networks
set_lin_config
configure_lin_networks

















