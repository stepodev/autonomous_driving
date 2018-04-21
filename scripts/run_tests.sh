#!/bin/bash

set -e #fail on error

#build workspace
source /opt/ros/lunar/setup.bash
scriptdir=$(pwd)
cd ../modules/platooning_ws && catkin_make
source devel/setup.bash

#run tests
logfilepath=$scriptdir"/testlog.txt"
rm -f $logfilepath

echo "#####################################################################\n" > $logfilepath
echo "MODULETESTS\n\n" >> $logfilepath

echo "#####################################################################\n" >> $logfilepath
echo "moduletest_longitudinalprocessing\n" >> $logfilepath
roslaunch platooning moduletest_longitudinalprocessing.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "moduletest_messagedistribution\n" >> $logfilepath

roslaunch platooning moduletest_messagedistribution.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "moduletest_messagetypes\n" >> $logfilepath
roslaunch platooning moduletest_messagetypes.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "moduletest_platooning\n" >> $logfilepath
roslaunch platooning moduletest_platooning.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "moduletest_prioritization\n" >> $logfilepath
roslaunch platooning moduletest_prioritization.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "moduletest_radiointerface\n" >> $logfilepath
roslaunch platooning moduletest_radiointerface.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "moduletest_udpserver\n" >> $logfilepath
roslaunch platooning moduletest_udpserver.test logfile:="$logfilepath"

echo "#####################################################################\n" > $logfilepath
echo "INTEGRATION TESTS \n\n" >> $logfilepath