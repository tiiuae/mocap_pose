Indoor GPS synthesizer for ROS from Qualisys QTM (a motion capture system)

More documentation in [fog_hyper documentation](https://github.com/tiiuae/fog_hyper/blob/main/docs/mocap.md).


## Development, debug

[See documentation in container base image](https://github.com/tiiuae/fog-ros-baseimage/tree/main#development--debug-for-concrete-projects)

## Outdated documentation

**WARNING**: Rest of the docs are outdated regarding container-specific build/running.


## Install dependencies

```
$ sudo apt update
$ sudo apt install ros-galactic-geodesy
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

