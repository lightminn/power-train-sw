#!/bin/bash
sudo ip link set can0 down
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
sudo ip link set can0 up type can bitrate 1000000
echo "CAN0 포트 1Mbps 세팅 완료!"
