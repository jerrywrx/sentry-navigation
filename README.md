# sentry-navigation

This is the development repository for an autonomous omnidirectional robot.

## Dependencies

### Python dependencies
```bash
yaml
```

## Build

Clone the repository:

```bash
git clone --recursive https://github.com/jerrywrx/sentry-navigation.git
```

Follow the exact order below:

```bash
source <noetic>
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

## Run

```bash
source ~/sentry-navigation/navigation_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch
roslaunch start_all start_all.launch
```