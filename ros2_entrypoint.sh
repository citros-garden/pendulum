#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/pendulum/install/setup.bash

exec "$@"