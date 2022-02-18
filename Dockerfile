# Use one base image for all the components.
# First tests can be done using debian packages of release 6.0.1.
FROM ghcr.io/tiiuae/fog-ros-baseimage:devel

RUN apt install -y \
	ros-${ROS_DISTRO}-mocap-pose=0.1.1-47~git20220120.d624ade

USER runner

# Indoor server settings should be available via ros-with-env wrapper.
# Possible solution to add /enclave/indoor_settings?
ENTRYPOINT exec ros-with-env ros2 launch mocap_pose mocap_pose.launch \
	address:=$INDOOR_SERVER_IP_ADDRESS \
	lat:=$INDOOR_ORIGO_LATITUDE \
	lon:=$INDOOR_ORIGO_LONGITUDE \
	alt:=$INDOOR_ORIGO_ALTITUDE
