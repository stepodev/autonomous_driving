#!/bin/bash

set -x

printf '\n\t\tUnd ab geht die wilde Fahrt!\n'

#Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&

#Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 &&

#Updating sources and upgrading packages
sudo apt-get update &&
sudo apt-get upgrade &&

#Installing ros lunar
sudo apt-get install ros-lunar-desktop-full &&
sudo rosdep init &&
rosdep update &&

#Dependencies for building packages
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential &&

#Environment variables
echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
source ~/.bashrc

{ set +x; } 2>/dev/null

printf '\t...läuft...\n'

sleep 5

print '\t...jetzt...\n'

sleep 3

printf '\n\n\n\n\t\t\t\t\t\t.'
