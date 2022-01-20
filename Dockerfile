FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-f70aceb

# Tampere. TODO: make configurable from somewhere else. /enclave ?
ENV INDOOR_SERVER_IP_ADDRESS=172.18.32.20
ENV INDOOR_ORIGO_LATITUDE=61.50341
ENV INDOOR_ORIGO_LONGITUDE=23.77509
ENV INDOOR_ORIGO_ALTITUDE=110.0

# entrypoint previously defined in https://github.com/tiiuae/fogsw_systemd/blob/main/system/mocap_pose.service

ENTRYPOINT ros-with-env ros2 launch mocap_pose mocap_pose.launch address:=$INDOOR_SERVER_IP_ADDRESS lat:=$INDOOR_ORIGO_LATITUDE lon:=$INDOOR_ORIGO_LONGITUDE alt:=$INDOOR_ORIGO_ALTITUDE

COPY bin/ros-galactic-mocap-pose_*.deb /tmp/mocap-pose.deb

RUN dpkg -i /tmp/mocap-pose.deb && rm /tmp/mocap-pose.deb
