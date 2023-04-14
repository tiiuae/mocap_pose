FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v1.0.0 AS builder

# DIRTY HOTFIX TO STILL KEEP THE OLD BASEIMAGE WITH NEW REPO - DO NOT KEEP THIS
RUN rm -f /etc/apt/sources.list.d/fogsw.list && rm -f /etc/apt/sources.list.d/fogsw-sros.list && \
    echo "deb [trusted=yes] https://ssrc.jfrog.io/artifactory/ssrc-deb-public-local focal fog-sw" >> /etc/apt/sources.list.d/fogsw-latest.list

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/

RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v1.0.0

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-mocap-pose_*_amd64.deb /mocap-pose.deb

RUN dpkg -i /mocap-pose.deb && rm /mocap-pose.deb
