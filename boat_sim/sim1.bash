#starts gazebo world and kills any rosbag recording nodes if any still exist
gnome-terminal -e "sh -c \"roslaunch imarc_gazebo test.launch;exec bash\""
sleep 3

#launches imarc model into gazebo sim
gnome-terminal -e "sh -c \"roslaunch imarc_description imarc.launch;exec bash\""

