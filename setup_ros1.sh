#!/bin/bash

set -e
set -x

echo -e "\n\t\tUnd ab geht die wilde Fahrt!\n"

sleep 2

#Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

set +e
sudo apt-get update
set -e
sudo apt-get install ros-lunar-desktop-full

sudo rosdep init
rosdep update

#Dependencies for building packages
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

#environment variables
echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
source ~/.bashrc
set +x
echo -e "\t...läuft...\n"

sleep 5

echo -e "\t...jetzt...\n"

sleep 3

echo -e "\n\n\n\n\t."

