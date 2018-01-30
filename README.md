INSTRUCTIONS

To set-up:
1. git clone git@gitlab.informatik.hu-berlin.de:adapt/ws17-HF/Gruppe-C.git
2. switch to branch develop (necessary for now)
3. cd catkin_ws && catkin_make
4. source devel/setup.bash (also put into your PATH)

To start:
1. `roslaunch src/platooning/launch/{node_name}.launch`
2. `rostopic list` and `rosnode list` should show new node_name and topic_name
3. `rostopic echo {topic_name}` should show messages by the node on the topic

To test:
1. `roslaunch platooning test/moduletest_node_name.test`
2. read test1.txt in folder TBD


