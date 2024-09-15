# This script is for initializing the development shell

source ~/.bashrc

if [ "$IS_DEV_CONTAINER" = "true" ]; then
    # This only runs when inside the container

    # Source overlay and underlay (if available)
    source /home/developer/repo/docker/ros_environment.sh
    
    alias gui="xdg-open http://${SELF_HOSTNAME}:${SELF_WEB_PORT}" # Alias to open the GUI

    # Wrap colcon such that it always executes in the workspace: /home/developer/repo
    # And that after it finishes, it will source the workspace setup.bash
    colcon() {
        (
            cd /home/developer/repo
            /usr/bin/colcon "$@"
        )
        source_workspace
    }
fi
