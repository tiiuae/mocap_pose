FROM ghcr.io/tiiuae/tii-px4-msgs:sha-e2978b9-${TARGETARCH:-amd64}-humble AS px4msgs

FROM ros:humble-ros-core-jammy AS base

RUN --mount=type=bind,from=px4msgs,source=/artifacts,target=/artifacts \
    apt-get update \
    && apt-get install -y --no-install-recommends /artifacts/*.deb \
        ros-humble-geodesy \
        ros-humble-launch-xml \
        ros-humble-ros2launch \
    && rm -rf /var/lib/apt/lists/*

FROM base AS builder

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libcurl4-openssl-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Jammy does not package prometheus-cpp; link the metrics libraries statically.
RUN git clone --branch v1.1.0 --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/jupp0r/prometheus-cpp.git /prometheus-cpp \
    && cmake -S /prometheus-cpp -B /prometheus-cpp/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=OFF \
        -DENABLE_PUSH=OFF \
        -DENABLE_TESTING=OFF \
    && cmake --build /prometheus-cpp/build --parallel 2 \
    && cmake --install /prometheus-cpp/build

WORKDIR /src
COPY . .

RUN . /opt/ros/humble/setup.sh \
    && cmake -S . -B /build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/mocap_pose \
        -DBUILD_TESTING=OFF \
    && cmake --build /build --parallel 2 \
    && cmake --install /build

FROM ghcr.io/tiiuae/pkcs11-closer:sha-7bec028 AS closer

FROM base AS runtime

WORKDIR /opt/mocap_pose

COPY --from=closer /pkcs11-closer /
COPY --from=builder /opt/mocap_pose/ /opt/mocap_pose/
COPY --from=builder /prometheus-cpp/LICENSE /usr/share/doc/prometheus-cpp/LICENSE
COPY --from=builder /prometheus-cpp/3rdparty/civetweb/LICENSE.md /usr/share/doc/civetweb/LICENSE.md
COPY LICENSE /usr/share/doc/mocap_pose/LICENSE
COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh", "/entrypoint.sh"]
CMD []
