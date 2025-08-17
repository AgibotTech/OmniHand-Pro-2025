1. load kernel modules.
	# modprobe can-dev
	# insmod usbcanfd.ko

2. install can-utils.
	# apt-get update
	# apt-get install can-utils

3. check hardware.
	# lsusb
	...04cc:1240...
	...3068:0009...

4. check device nodes.
	# ls /sys/class/net | grep can
	# ls /sys/class/net | grep lin

5. save lin config.
	# echo '{"LIN0":{"Enable":1,"IsMaster":1,"Baudrate":19200,"Feature":1,"TrEnable":1,"DataLen":8}}' > `ls /sys/bus/usb/drivers/usbcanfd/*/lin0_cfg`
	# echo '{"LIN1":{"Enable":1,"IsMaster":0,"Baudrate":19200,"Feature":1,"TrEnable":0,"DataLen":8}}' > `ls /sys/bus/usb/drivers/usbcanfd/*/lin1_cfg`

6. bring up can/lin interfaces.
	# ip link set can0 up type can help
	# ip link set can0 up type can fd on bitrate 1000000 dbitrate 4000000
	# ip link set can1 up type can fd on bitrate 1000000 dbitrate 4000000
	# ip link set lin0 up type can fd on bitrate 1000000 dbitrate 1000000
	# ip link set lin1 up type can fd on bitrate 1000000 dbitrate 1000000
	# ifconfig can0 up
	# ifconfig can1 up
	# ifconfig lin0 up
	# ifconfig lin1 up
	# ip -details link show can0

7. receive.
	# candump any

8. send.
	# cansend can0 123#00.11.22.33.44.55.66.77
	# cansend can0 12345678##f00.11.22.33.44.55.66.77.88.99.aa.bb.cc.dd.ee.ff
	# cansend lin0 b12#00.11.22.33.44.55.66.77
