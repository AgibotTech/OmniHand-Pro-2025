sudo ip link set can0 type can fd on bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.8
sudo ifconfig can0 up