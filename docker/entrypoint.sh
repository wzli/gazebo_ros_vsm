#!/bin/bash

export VSM_HOST=${VSM_HOST:-$(hostname -i)}
export VSM_PORT=${VSM_PORT:-11511}
export VSM_BOOTSTRAP_PEER=${VSM_BOOTSTRAP_PEER:-${VSM_HOST%.*}.2:$VSM_PORT}

VSM_ID=${VSM_ID:-$((${VSM_HOST##*.} - 2))}
export VSM_NAME=${VSM_NAME:-robot_$VSM_ID}
export SPAWN_X=${SPAWN_X:-$VSM_ID}
export SPAWN_Y=${SPAWN_Y:-"-$VSM_ID"}
export SPAWN_YAW=${SPAWN_YAW:-1.57}

(cd ${GZWEB_WS}; npm start) &
source ${CATKIN_WS}/devel/setup.bash
roslaunch gazebo_vsm_demo gazebo_vsm_demo.launch
