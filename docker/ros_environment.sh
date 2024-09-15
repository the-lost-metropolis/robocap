# This script defines the environment that provides access to the underlay and overlay

source /opt/ros/humble/setup.sh # Source underlay
source /usr/share/colcon_cd/function/colcon_cd.sh # Source colcon_cd
export _colcon_cd_root=/opt/ros/humble/ # Set colcon_cd root
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash # Source colcon_argcomplete

source_workspace() { # Function to source workspace if available
    if [ -f /home/developer/repo/install/setup.bash ]; then
        source /home/developer/repo/install/setup.bash
    fi
}

clear_workspace() { # Function to clear workspace (build, install, log)
    rm -rf /home/developer/repo/build /home/developer/repo/install /home/developer/repo/log
}

source_workspace # Actually source the workspace