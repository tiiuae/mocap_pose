FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v2.1.0 AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN dpkg -i /main_ws/src/ros-humble-px4-msgs_6.0.0-0~git20231005.d0a163b_amd64.deb && rm -f /main_ws/src/ros-humble-px4-msgs_6.0.0-0~git20231005.d0a163b_amd64.deb
RUN sed -i 's/ros-humble-px4-msgs=5.0.0\*/ros-humble-px4-msgs/g' /packaging/rosdep.yaml

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v2.1.0

HEALTHCHECK --interval=5s \
	CMD fog-health check --metric=location_update_count --diff-gte=5.0 \
		--metrics-from=http://localhost:${METRICS_PORT}/metrics --only-if-nonempty=${METRICS_PORT}

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-mocap-pose_*_amd64.deb /mocap-pose.deb

RUN dpkg -i /mocap-pose.deb && rm /mocap-pose.deb
