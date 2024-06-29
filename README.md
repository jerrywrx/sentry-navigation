# sentry-navigation

This is the development repository for an autonomous omnidirectional robot.

Note: slam_ws is no longer used.

This project contains three main workspaces:

```bash
sentry-navigation
├── livox_ws
├── fastlio_ws
└── navigation_ws
```

The livox_ws contains the [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) package and the fastlio_ws contains the [FAST-LIO](https://github.com/hku-mars/FAST_LIO) package. The navigation_ws is the main workspace for navigation.

## Dependencies

### 1. Python dependencies

Create a conda environment with `python=3.10` and install the following packages using `python3 -m pip install`.

```bash
empy==3.3.4
catkin_package
pyyaml
rospkg
pyserial
crc
numpy
opencv-python
```

### 2. ROS dependencies

Make sure you have ROS Noetic installed.

```bash
sudo apt-get update

# Install the following packages with sudo apt-get
ros-noetic-map-server
ros-noetic-robot-localization
ros-noetic-move-base
ros-noetic-teb-local-planner
```

### 3. Livox SDK

See [here](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md) for instructions

## Build

Clone the repository:

```bash
git clone --recursive https://github.com/jerrywrx/sentry-navigation.git
```

Follow the EXACT order below for correct workspace overlaying:

```bash
source /opt/ros/noetic/setup.bash
```

### Build livox_ros_driver2 in livox_ws

```bash
cd ~/sentry-navigation/livox_ws/src/livox_ros_driver2
./build.sh ROS1
source ../../devel/setup.bash
```

### Build FAST-LIO in fastlio_ws

```bash
cd ~/sentry-navigation/fastlio_ws
catkin_make
source devel/setup.bash
```

### Build navigation_ws

```bash
cd ~/sentry-navigation/navigation_ws
catkin_make
source devel/setup.bash
```

## Preparation

In the config file below, change `cmd_data_ip`, `push_msg_ip`, `point_data_ip`, and `imu_data_ip` under MID360/host_net_info to the IP of your machine. Change `ip` under lidar_configs to the ip of the lidar. For MID360, its of the form 192.168.1.1XX, where XX are the last two digits of the lidar's serial number.

```bash
livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

## Run

```bash
source /opt/ros/noetic/setup.bash
source ~/sentry-navigation/navigation_ws/devel/setup.bash

# launch livox driver
roslaunch livox_ros_driver2 msg_MID360.launch

# launch fast-lio
roslaunch fast_lio mapping_mid360.launch

# launch pcl_processing, localization, laser_projection, base_controller, and move_base
roslaunch start_all start_all.launch
```