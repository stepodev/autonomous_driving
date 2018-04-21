#!/bin/bash

set +e #fail on error

#build workspace
source /opt/ros/lunar/setup.bash
scriptdir=$(pwd)
cd ../modules/platooning_ws && catkin_make
source devel/setup.bash

#run tests
logfilepath=$scriptdir"/testlog.txt"
rm -f $logfilepath

echo "#####################################################################" > $logfilepath
echo "MODULETESTS" >> $logfilepath

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_longitudinalprocessing" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_longitudinalprocessing.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_messagedistribution" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_messagedistribution.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_messagetypes" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_messagetypes.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_platooning" >> $logfilepath
echo "" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_platooning.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_prioritization" >> $logfilepath
echo "" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_prioritization.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_radiointerface" >> $logfilepath
echo "" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_radiointerface.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "moduletest_udpserver" >> $logfilepath
echo "" >> $logfilepath
echo "" >> $logfilepath

roslaunch platooning moduletest_udpserver.test logfile:="$logfilepath"

echo "" >> $logfilepath
echo "" >> $logfilepath
echo "#####################################################################" >> $logfilepath
echo "INTEGRATION TESTS" >> $logfilepath
echo "" >> $logfilepath
echo "" >> $logfilepath

