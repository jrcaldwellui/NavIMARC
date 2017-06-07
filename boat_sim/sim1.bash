#starts gazebo world and kills any rosbag recording nodes if any still exist
gnome-terminal -e "sh -c \"roslaunch imarc_gazebo test.launch;killall roscore;exec bash\""
sleep 3


#starts keyboard input terminal and ros node
if [ -z "$1" ]
	then
		gnome-terminal -e "sh -c \"rosrun ros_sim keyboard_input;exec bash\""
	else
		#if a bag name is given start rosbag record
		gnome-terminal -e "sh -c \"$(rosbag record -O ~/bags/bag-$(date +\'%s\') /imarc/laser/scan /tf) &;rosrun ros_sim keyboard_input;exec bash\""
fi
sleep 2

#launches imarc model into gazebo sim
gnome-terminal -e "sh -c \"roslaunch imarc_description imarc.launch;exec bash\""
sleep 2

#publishes tf from static point to boat body
#rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map body 20 &

