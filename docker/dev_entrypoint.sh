#!/bin/bash

# This file defines the raw entrypoint for the container when in a development environment

# Adjust permissions if necessary
if [ "$(id -u)" != "1000" ] || [ "$(id -g)" != "1000" ]; then
    echo "Adjusting permissions..."
    for item in /home/developer/.* /home/developer/*; do
        if [ "$item" != "/home/developer/repo" ] && [ "$item" != "/home/developer/." ] && [ "$item" != "/home/developer/.." ]; then
            sudo chown -R $(id -u):$(id -g) "$item"
            sudo chmod -R u+rw "$item"
        fi
    done
fi

# Dump root shell environment to file such that we can access them anywhere later
printenv | sed 's/^\(.*\)$/export \1/' > /home/developer/project_env.sh
echo "source /home/developer/project_env.sh" >> /home/developer/.bashrc

# # Source shell setup, which gives sources ROS2 underlay (and if available, overlay / workspace), and environmental variables
# source /home/developer/repo/.vscode/shell_setup.sh

# # Install workspace dependencies, build workspace
# source /home/developer/repo/docker/prebuild.sh
# prebuild # In the case that we are launching rbc_dev_ros_prebuild instead of rbc_dev_ros, some dependenices are already installed / some packages are already built whichs saves time

sudo /usr/sbin/sshd
xpra start --bind-tcp=0.0.0.0:8000
tail -f /dev/null