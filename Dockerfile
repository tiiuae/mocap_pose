FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-2f516bb AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-2f516bb

ENV LD_PRELOAD=libkeepalive.so
ENV KEEPIDLE=5
ENV KEEPINTVL=5
ENV KEEPCNT=3

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-mocap-pose_*_amd64.deb /mocap-pose.deb

RUN dpkg -i /mocap-pose.deb && rm /mocap-pose.deb \
	&& apt update && apt install -y libkeepalive0
