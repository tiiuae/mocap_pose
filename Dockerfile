# Given dynamically from CI job.
FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/tii-px4-msgs:sha-77f972d-${TARGETARCH:-amd64}-humble AS px4msgs
FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:v3.4.0-${TARGETARCH:-amd64} AS builder

# Must be defined another time after "FROM" keyword.
ARG TARGETARCH

COPY --from=px4msgs /artifacts /artifacts

RUN apt-get update \
    && apt-get install -y --no-install-recommends /artifacts/*.deb \
    && rm -rf /var/lib/apt/lists/*

# SRC_DIR environment variable is defined in the fog-ros-sdk image.
# The same workspace path is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile.sdk_builder
COPY . $SRC_DIR/mocap_pose

# Tar directories so they are easier to handle when doing installation.
RUN /packaging/build_colcon_sdk.sh ${TARGETARCH:-amd64}
# Even though it is possible to tar the install directory for retrieving it later in runtime image,
# the tar extraction in arm64 emulated on arm64 is still slow. So, we copy the install directory instead

FROM ghcr.io/tiiuae/pkcs11-closer:sha-7bec028 AS closer

FROM ghcr.io/tiiuae/fog-ros-baseimage:v3.4.0

HEALTHCHECK --interval=5s \
	CMD fog-health check --metric=location_update_count --diff-gte=5.0 \
		--metrics-from=http://localhost:${METRICS_PORT}/metrics --only-if-nonempty=${METRICS_PORT}

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

# WORKSPACE_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
WORKDIR $WORKSPACE_DIR

COPY --from=closer /pkcs11-closer /
COPY --from=builder $WORKSPACE_DIR/install install
COPY --from=builder /artifacts /artifacts

RUN apt-get update \
    && apt-get install -y --no-install-recommends /artifacts/*.deb \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
