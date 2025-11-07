#!/bin/bash
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]:-${(%):-%x}}" )" >/dev/null 2>&1 && pwd )"
source ${CURRENT_DIR}/install/setup.bash

ros2 launch plan_manager run_desert.launch.py & sleep 1;
wait;
