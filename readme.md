Harpun leci ostro

Niewiadów na minusie

--- Command hell ---

cd ~/ros2_ws
rm -rf build/ install/ log/

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

ros2 launch roboracer_sim simulation.launch.py

# ros2 run roboracer_sim roboracer_pid
# ros2 run roboracer_sim control_stanley