catkin_make
gnome-terminal -x bash -c "roslaunch dashgo_driver driver_imu.launch; exec bash" --tab
sleep 2
rosrun vision_nav vision_nav_node
