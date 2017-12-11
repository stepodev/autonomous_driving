#!/bin/bash

if rospack find turtlesim;then
    echo 'Go go go!'
else
    rosdep install turtlesim &&
    echo "turtlesim installed"
fi

catkin_make &&
source devel/setup.bash &&
source ~/.bashrc &&

if rospack find platooning;then
    echo 'GREAT SUCCESS! platooning is installed'
else
    echo 'something went wrong. platooning package is not installed'
fi
