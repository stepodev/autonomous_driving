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



echo sudo ifconfig wlxc404157aa1b4 down

sudo ifconfig wlxc404157aa1b4 down

sleep 0.5



echo sudo iw wlxc404157aa1b4 set type ibss

sudo iw wlxc404157aa1b4 set type ibss

sleep 0.5



echo sudo ifconfig wlxc404157aa1b4 up

sudo ifconfig wlxc404157aa1b4 up

sleep 0.5



echo sudo iw wlxc404157aa1b4 ibss join SPHAF 2432

sudo iw wlxc404157aa1b4 ibss join SPHAF 2432

sleep 0.5



echo sudo ifconfig wlxc404157aa1b4 10.42.43.$1

sudo ifconfig wlxc404157aa1b4 10.42.43.$1

sleep 0.5

ifconfig wlxc404157aa1b4 netmask 255.0.0.0

echo done

