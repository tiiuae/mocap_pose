# Use one base image for all the components.
# First tests can be done using debian packages of release 6.0.1.
FROM ghcr.io/tiiuae/fog-ros-baseimage:devel

# Finland site default settings.
# Site settings are set when starting the container.
ENV INDOOR_SERVER_IP_ADDRESS=172.18.32.20
ENV INDOOR_ORIGO_LATITUDE=61.50341
ENV INDOOR_ORIGO_LONGITUDE=23.77509
ENV INDOOR_ORIGO_ALTITUDE=110.0

RUN apt install -y \
	ros-${ROS_DISTRO}-mocap-pose=0.1.1-47~git20220120.d624ade

USER runner

ENTRYPOINT exec ros-with-env ros2 launch mocap_pose mocap_pose.launch \
	address:=$INDOOR_SERVER_IP_ADDRESS \
	lat:=$INDOOR_ORIGO_LATITUDE \
	lon:=$INDOOR_ORIGO_LONGITUDE \
	alt:=$INDOOR_ORIGO_ALTITUDE
