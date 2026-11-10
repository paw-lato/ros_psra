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

#### ros2 run roboracer_sim roboracer_pid
#### ros2 run roboracer_sim control_stanley



Bandycka jazda w pigułce:

# Bez Gazebo GUI i bez RViz, max RTF — szybka ewaluacja
ros2 launch roboracer_sim experiment.launch.py controller:=pid gui:=false rviz:=false rtf:=10

# 1. Normalnie, po Bożemu — PID, RViz, czas rzeczywisty
ros2 launch roboracer_sim experiment.launch.py controller:=pid

# 2. Szybko — ~3× RTF, nadal z RViz
ros2 launch roboracer_sim experiment.launch.py controller:=pid rtf:=3.0

# 3. Max szybkość — RTF bez limitu, bez RViz (lżejsze)
ros2 launch roboracer_sim experiment.launch.py controller:=stanley rtf:=0 rviz:=false

# 4. Wolniej / spokojniej — RTF 0.5 (pół tempa), z RViz
ros2 launch roboracer_sim experiment.launch.py controller:=pid rtf:=0.5

# 5. Stanley + otwarte podwozie + bez RViz, RTF 2×



ros2 launch roboracer_sim experiment.launch.py controller:=stanley body:=open rviz:=false rtf:=2.0
#### ros2 run roboracer_sim pure_pursuit
