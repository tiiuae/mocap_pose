#!/bin/bash -e

_term() {
    # FILL UP PROCESS SEARCH PATTERN HERE TO FIND PROPER PROCESS FOR SIGINT:
    pattern="mocap_pose/mocap_pose_node"

    pid_value="$(ps -e | grep $pattern | grep -v grep | awk '{ print $1 }')"
    if [ "$pid_value" != "" ]; then
        pid=$pid_value
        echo "Send SIGINT to pid $pid"
    else
        pid=1
        echo "Pattern not found, send SIGINT to pid $pid"
    fi
    kill -s SIGINT $pid
}
# Use SIGTERM or TERM, does not seem to make any difference.
trap _term TERM

ros-with-env ros2 launch mocap_pose mocap_pose.launch \
    address:=$INDOOR_SERVER_IP_ADDRESS \
    lat:=$INDOOR_ORIGO_LATITUDE \
    lon:=$INDOOR_ORIGO_LONGITUDE \
    alt:=$INDOOR_ORIGO_ALTITUDE &
child=$!

echo "Waiting for pid $child"
# * Calling "wait" will then wait for the job with the specified by $child to finish, or for any signals to be fired.
#   Due to "or for any signals to be fired", "wait" will also handle SIGTERM and it will shutdown before
#   the node ends gracefully.
#   The solution is to add a second "wait" call and remove the trap between the two calls.
# * Do not use -e flag in the first wait call because wait will exit with error after catching SIGTERM.
set +e
wait $child
set -e
trap - TERM
wait $child
RESULT=$?

/pkcs11-closer --label mocap-pose

if [ $RESULT -ne 0 ]; then
    echo "ERROR: Mocap pose node failed with code $RESULT" >&2
    exit $RESULT
else
    echo "INFO: Mocap pose node finished successfully, but returning 125 code for docker to restart properly." >&2
    exit 125
fi
