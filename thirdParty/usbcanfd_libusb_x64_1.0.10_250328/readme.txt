demo示例
1. USBCANFD驱动基于libusb实现，请确保运行环境中有libusb-1.0的库。
  	如果是ubuntu，可连网在线安装，命令如下：
  	# apt-get install libusb-1.0-0

2. 将libusbcanfd.so拷到/lib目录。
	# sudo cp libusbcanfd.so /lib

3. 进入test目录，运行基本收发例程：
	# sudo ./test

4. 编译
	# make

5. 如果运行 sudo ./test，遇到 No such file or directory。
	# cd /lib
	# sudo ln -s libusbcanfd.so libusbcanfd.so.1.0.8


例程说明：
	test.c 		例程中默认启动两个通道，波特率500k+2M。通道0会发送报文，可以将两个通道对接起来，会看到报文打印。
	testLin.c 	例程中两个lin通道，一个做主站，一个做从站，主站和从站都有设置响应。
	test_uds.c	例程演示UDS请求


设备调试常用命令：
1、查看系统是否正常枚举到usb设备，打印它们的VID/PID（USBCANFD为3068:0009）：
	# lsusb

2、查看系统内所有USB设备节点及其访问权限：
	# ls /dev/bus/usb/ -lR

3、修改usb设备的访问权限使普通用户可以操作，其中xxx对应lsusb输出信息中的bus序号，yyy对应device序号：
	# chmod 666 /dev/bus/usb/xxx/yyy

4、如果要永久赋予普通用户操作USBCANFD设备的权限，需要修改udev配置，增加文件：/etc/udev/rules.d/50-usbcanfd.rules，内容如下：
	SUBSYSTEMS=="usb", ATTRS{idVendor}=="3068", ATTRS{idProduct}=="0009", GROUP="users", MODE="0666"

	重新加载udev规则后插拔设备即可应用新权限：
	# udevadm control --reload

5、获取库版本号示例：
	# ./version.sh ./libusbcanfd.so
	输出：
	# 1.0.3

6、固件升级，升级文件 https://manual.zlg.cn/web/#/316/12495 ：
	# ./upgrade ./usbcanfd_1_200u_upgrade_2.43.bin 0
