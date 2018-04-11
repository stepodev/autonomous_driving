service network-manager stop
ip link set wlan0 down
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 channel 4
iwconfig wlan0 essid SPHAF
ip link set wlan0 up
dhclient wlan0 &
ifconfig wlan0
