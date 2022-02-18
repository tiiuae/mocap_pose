## Install dependencies

```
$ sudo apt update
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

## Docker

Use the following instructions if you want to build docker image and run docker container in your local machine for development purposes.

In this case you will neeed to have available fog-ros-base-image in your local docker environment.

### Build

```
$ docker build -t tii-mocap-pose-runner .
```

### Run

For running the node in your local machine, you will need to have a valid enclave.

```
$ docker run --rm -v /enclave:/enclave -it tii-mocap-pose-runner
```
