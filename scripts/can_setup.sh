#!/bin/bash
sudo ip link set can0 down
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
# restart-ms 100: bus-off 시 100ms 후 자동 복구 (다중 모터 1Mbps 마진 버스 대비).
# txqueuelen 1000: 일시적 TX 적체(ENOBUFS) 흡수 (기본 10은 너무 작음).
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
echo "CAN0 포트 1Mbps 세팅 완료! (restart-ms 100, txqueuelen 1000)"
