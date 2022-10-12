FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-a2b1348 AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-a2b1348

HEALTHCHECK --interval=5s \
	CMD fog_health check --metric=location_update_count --diff-gte=5.0 --metrics-from=http://localhost:9090/metrics

ENTRYPOINT exec ros-with-env ros2 launch mocap_pose mocap_pose.launch \
	address:=$INDOOR_SERVER_IP_ADDRESS \
	lat:=$INDOOR_ORIGO_LATITUDE \
	lon:=$INDOOR_ORIGO_LONGITUDE \
	alt:=$INDOOR_ORIGO_ALTITUDE

COPY --from=builder /main_ws/ros-*-mocap-pose_*_amd64.deb /mocap-pose.deb

RUN dpkg -i /mocap-pose.deb && rm /mocap-pose.deb
