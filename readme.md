## --- Haiku ---

Harpun leci ostro

Niewiadów na minusie

Dron na złomowisku

## --- Command hell ---

cd ~/ros2_ws
rm -rf build/ install/ log/

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --merge-install

source install/setup.bash

ros2 launch roboracer_sim simulation.launch.py

#### ros2 run roboracer_sim roboracer_pid
#### ros2 run roboracer_sim control_stanley
#### ros2 run roboracer_sim pure_pursuit

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

### 8. Kamera zamiast CSV (line_perception)
ros2 launch roboracer_sim experiment.launch.py controller:=pid path_source:=camera
ros2 launch roboracer_sim experiment.launch.py controller:=pure_pursuit path_source:=camera
ros2 launch roboracer_sim experiment.launch.py controller:=stanley path_source:=camera

Debug obrazu: `/perception/debug_image`  
Ścieżka lokalna: `/perception/local_path`  
Uchyb (PID): `/perception/line_error`


## --- Źródło ścieżki (`path_source`) ---

- `path_source:=gt` (domyślne) → `/track/get_path` (CSV toru)
- `path_source:=camera` → startuje `line_perception`; PID dostaje `use_camera:=true` (`/perception/line_error`), Stanley/PP biorą `/perception/local_path`


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
