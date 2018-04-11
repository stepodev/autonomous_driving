service network-manager stop
ip link set wlan0 down
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 channel 4
iwconfig wlan0 essid SPHAF
ip link set wlan0 up
ip addr add 10.42.43.100 dev wlan0
ifconfig wlan0
