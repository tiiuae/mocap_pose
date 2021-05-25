## Install dependencies

```
$ sudo apt update
$ sudo apt install ros-foxy-geodesy
```

## Build

```
$ colcon build
```

## Run

```
$ ros2 run mocap_pose mocap_pose_node --ros-args -p home_lat:=<latitude value> -p home_lon:=<longitude value> -p home_alt:=<altitude value> -p frequency:=<update freq>
```
example:
```
$ ros2 run mocap_pose mocap_pose_node --ros-args -p home_lat:=63.125562 -p home_lon:=21.572674 -p home_alt:=98.4 -p frequency:=10
```

