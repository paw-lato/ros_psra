## --- Haiku ---

Harpun leci ostro

Niewiadów na minusie

Dron na złomowisku

## --- Command hell ---

cd ~/ros2_ws
rm -rf build/ install/ log/

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

ros2 launch roboracer_sim simulation.launch.py

#### ros2 run roboracer_sim roboracer_pid
#### ros2 run roboracer_sim control_stanley

## --- Bandycka jazda w pigułce: ---

### 0. Bez Gazebo GUI i bez RViz, max RTF — szybka ewaluacja
ros2 launch roboracer_sim experiment.launch.py controller:=pid gui:=false rviz:=false rtf:=10

### 1. Normalnie, po Bożemu — PID, RViz, czas rzeczywisty
ros2 launch roboracer_sim experiment.launch.py controller:=pid

### 2. Szybko — ~3× RTF, nadal z RViz
ros2 launch roboracer_sim experiment.launch.py controller:=pid rtf:=3.0

### 3. Max szybkość — RTF bez limitu, bez RViz (lżejsze)
ros2 launch roboracer_sim experiment.launch.py controller:=stanley rtf:=0 rviz:=false

### 4. Wolniej / spokojniej — RTF 0.5 (pół tempa), z RViz
ros2 launch roboracer_sim experiment.launch.py controller:=pid rtf:=0.5

### 5. Stanley + otwarte podwozie + bez RViz, RTF 2×
ros2 launch roboracer_sim experiment.launch.py controller:=stanley body:=open rviz:=false rtf:=2.0

### 6. Pure Pursuit na torze łatwym (domyślnie easy:=true)
ros2 launch roboracer_sim experiment.launch.py controller:=pure_pursuit

### 7. Pure Pursuit na torze trudnym
ros2 launch roboracer_sim experiment.launch.py controller:=pure_pursuit easy:=false

### 8. NEW - PERCEPCJA Z KAMERY
ros2 run roboracer_sim line_perception
PID z kamerą:
ros2 run roboracer_sim roboracer_pid --ros-args -p use_camera:=true
PID z csv:
ros2 run roboracer_sim roboracer_pid

## --- Wybór toru ---

Eksperyment (`easy`):
- `easy:=true` → `linefollower_track_easy` (domyślne w experiment)
- `easy:=false` → `linefollower_track`

Symulacja bezpośrednio (`track`):
```
ros2 launch roboracer_sim simulation.launch.py track:=linefollower_track
```

Tory:
- `linefollower_track_easy`
- `linefollower_track` (domyślny w simulation)

Dodanie autmatycznego pobierania waypointów

W `__init__` robimy zapytnaie o ścieżkę do `/track/get_path` podajemy do waypoints i otrzymujemy pełną listę waypointów w metrach:

((0.0, 0.0),
 
 (0.0779806, 0.0),
 
 (0.1559611, 0.0),
 
 ...)

```python
from nav_msgs.srv import GetPlan

# w __init__:
client = self.create_client(GetPlan, '/track/get_path')
while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Czekam na serwis /track/get_path...')

future = client.call_async(GetPlan.Request())
rclpy.spin_until_future_complete(self, future)
self.waypoints = tuple(
    (p.pose.position.x, p.pose.position.y) for p in future.result().plan.poses
)
```
