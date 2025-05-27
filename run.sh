sh install/setup.sh
source /opt/ros/jazzy/setup.sh
colcon build
source install/local_setup.sh
sudo ros2 run servo main