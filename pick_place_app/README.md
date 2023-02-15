# Pick place application ros2

### 1. follow waypoints defined in `config/waypoints.yaml`

```
# start with fake driver
ros2 launch pick_place_app ur5e_driver.launch.py

ros2 launch pick_place_app point_to_point_ur5e.launch.py
```

```
# start with real robot
ros2 launch pick_place_app ur5e_driver.launch.py use_fake_hardware:=false robot_ip:="robot ip"

ros2 launch pick_place_app point_to_point_ur5e.launch.py
```

### 2. pick n place with points defined in the parameter file
e.g. object pose and place pose defined in `config/pick_place_static_ur5e.yaml`

It adds an object to the planning scene. Therefore, it needs to launch move_group

```
# start with fake driver
ros2 launch ur5e_cell_moveit_config demo.launch.py

ros2 launch pick_place_app pick_place_static_demo_ur5e.launch.py
```

### 3. pick n place with marker detection
e.g. object pose and place pose defined in `config/pick_place_task_ur5e.yaml`

It adds an object to the planning scene. Therefore, it needs to launch move_group

```
# start with fake driver
ros2 launch ur5e_cell_moveit_config robot.launch.py robot_ip:="robot ip"

ros2 launch pick_place_app aruco_detection_with_realsense.launch.py

ros2 launch pick_place_app pick_place_demo_ur5e.launch.py
```
