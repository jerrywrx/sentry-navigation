# sentry-navigation

This is the development repository for an autonomous omnidirectional robot.

Note: fastlio_ws is no longer used.

This project contains three main workspaces:

```bash
sentry-navigation
├── livox_ws
├── slam_ws
└── navigation_ws
```

The livox_ws contains the [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) package and the slam_ws contains the [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN/tree/master) and [FAST-LIO-Localization-QN](https://github.com/engcang/FAST-LIO-Localization-QN/) packages. The navigation_ws is the main workspace for navigation.

## Dependencies

### 1. Python dependencies

Create a conda environment with `python>=3.10` and install the following packages using `python3 -m pip install`.

```bash
empy==3.3.4
catkin_package
pyyaml
rospkg
pyserial
crc
numpy
opencv-python
pynput
```

### 2. ROS dependencies

Make sure you have ROS Noetic installed. Install the following with `sudo apt-get`.

```bash
ros-noetic-map-server
ros-noetic-robot-localization
ros-noetic-move-base
ros-noetic-teb-local-planner
```

### 3. Livox SDK

See [here](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md) for instructions.

### 4. FAST-LIO-SAM-QN & FAST-LIO-Localization-QN

Install the dependencies shown in [here](https://github.com/engcang/FAST-LIO-SAM-QN/tree/master) and [here](https://github.com/engcang/FAST-LIO-Localization-QN/).

## Build

Clone the repository:

```bash
git clone --recursive https://github.com/jerrywrx/sentry-navigation.git
```

Follow the EXACT order below for correct workspace overlaying:

```bash
source /opt/ros/noetic/setup.bash
```

### 1. Build livox_ros_driver2 in livox_ws

```bash
cd ~/sentry-navigation/livox_ws/src/livox_ros_driver2
./build.sh ROS1
source ../../devel/setup.bash
```

### 2. Build FAST-LIO-SAM-QN and FAST-LIO-Localization-QN in slam_ws

```bash
cd ~/sentry-navigation/slam_ws
catkin_make
source devel/setup.bash
```

### 3. Build navigation_ws

```bash
cd ~/sentry-navigation/navigation_ws
catkin_make
source devel/setup.bash
```

## Preparation

### MID360
See the connection section of [this](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) document.

### Livox driver config

In the config file below, change `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, and `imu_data_ip` under MID360/host_net_info to the ip of the host. Change `ip` under lidar_configs to the ip of the lidar. For MID360, its of the form 192.168.1.1XX, where XX are the last two digits of the lidar's serial number.

```bash
livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

## Run

```bash
source /opt/ros/noetic/setup.bash
source ~/sentry-navigation/navigation_ws/devel/setup.bash
```

### 1. Map building

Launch livox driver
```bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

Launch FAST-LIO SAM module (set rviz to false if using ssh)
```bash
roslaunch fast_lio_sam_qn run.launch lidar:=livox rviz:=false
```

After you finished building the map, press `CTRL-C` once to save the map. Move the saved map to `slam_ws/src/results`, otherwise it will be overwritten.

### 2. Map processing

The `map_server` requires a 2D occupancy grid map in PNG format. You can convert a 3D PCD file into a 2D PNG file using `pcd2png.py`.

After converting, put the 2D map in `navigation_ws/src/base_controller/params`. Then, change `image` in `map_server.yaml` to the name of the 2D map.

### 3. Run Localization with pre-built map

Launch livox driver
```bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

Launch FAST-LIO Localization module (set rviz to false if using ssh). Before launching, set `saved_map` in 

`slam_ws/src/mid360_localization/FAST-LIO-Localization-QN/fast_lio_localization_qn/config/config.yaml` 

to the directory of the map you built using FAST-LIO SAM.
```bash
roslaunch fast_lio_localization_qn run.launch lidar:=livox rviz:=false
```
Launch the rest of the packages (pcl_processing, localization, laser_projection, base_controller, and move_base)
```bash
roslaunch start_all start_all.launch
```