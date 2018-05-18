service network-manager stop
ip link set wlp3s0 down
iwconfig wlp3s0 mode ad-hoc
iwconfig wlp3s0 channel 4
iwconfig wlp3s0 essid SPHAF
ip link set wlp3s0 up
ip addr add 10.42.43.101 dev wlp3s0
ifconfig wlp3s0
