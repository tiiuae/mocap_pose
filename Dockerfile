FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v2.0.0 AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v2.0.0

HEALTHCHECK --interval=5s \
	CMD fog_health check --metric=location_update_count --diff-gte=5.0 \
		--metrics-from=http://localhost:${METRICS_PORT}/metrics --only-if-nonempty=${METRICS_PORT}

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-mocap-pose_*_amd64.deb /mocap-pose.deb

RUN dpkg -i /mocap-pose.deb && rm /mocap-pose.deb
