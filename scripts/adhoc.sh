# Addresses:

# Odriod 10.49.43.1

#



if [ -z "$1" ]

  then

    echo Missing last digit of IP-Adress

    exit 1

fi



echo start



echo sudo service network-manager stop

sudo service network-manager stop

sleep 0.5



echo sudo ifconfig wlan0 down

sudo ifconfig wlan0 down

sleep 0.5



echo sudo iw wlan0 set type ibss

sudo iw wlan0 set type ibss

sleep 0.5



echo sudo ifconfig wlan0 up

sudo ifconfig wlan0 up

sleep 0.5



echo sudo iw wlan0 ibss join SPHAF 2432

sudo iw wlan0 ibss join SPHAF 2432

sleep 0.5



echo sudo ifconfig wlan0 10.42.43.$1

sudo ifconfig wlan0 10.42.43.$1

sleep 0.5



echo done

