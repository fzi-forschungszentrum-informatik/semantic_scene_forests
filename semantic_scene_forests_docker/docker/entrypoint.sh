#!/usr/bin/bash
source /opt/ros/kilted/setup.bash

if [[ -f /home/ubuntu/ws/install/setup.bash ]]; then
  source /home/ubuntu/ws/install/setup.bash
fi


# Execute the command passed into this entrypoint.
exec "$@"
