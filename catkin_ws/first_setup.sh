#!/bin/bash

catkin_make &&
source devel/setup.bash &&
source ~/.bashrc &&

if rospack find platooning;then
    echo 'GREAT SUCCESS! platooning is installed'
else
    echo 'something went wrong. platooning package is not installed'
fi