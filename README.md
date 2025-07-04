# CANOPEN-ROS-PKG

## Version
- Ubuntu 22.04
- ROS2 humble

## Requirements
- canopen_sdk==0.0.1

## Build
```
colcon build --packages-select canopen_msgs canopen_ros_pkg canopen_rqt_plugin_pkg
```

## Launch
```
ros2 launch canopen_ros_pkg gui_canopen_ros_pkg.launch.py
```
or
```
ros2 launch canopen_ros_pkg canopen_ros_pkg.launch.py
```


