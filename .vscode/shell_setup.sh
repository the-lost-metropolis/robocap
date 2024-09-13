source ~/.bashrc

if [ "$IS_DEV_CONTAINER" = "true" ]; then
    # This only runs when inside the container
    source /opt/ros/humble/setup.sh # Source underlay
    alias gui="xdg-open http://${SELF_HOSTNAME}:${SELF_WEB_PORT}" # Alias to open the GUI
    source /usr/share/colcon_cd/function/colcon_cd.sh # Source colcon_cd
    export _colcon_cd_root=/opt/ros/humble/ # Set colcon_cd root
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash # Source colcon_argcomplete

    source_workspace() {
        if [ -f /home/developer/repo/install/setup.bash ]; then
            source /home/developer/repo/install/setup.bash
        fi
    }

    # Wrap colcon such that it always executes in the workspace: /home/developer/repo
    # And that after it finishes, it will source the workspace setup.bash
    colcon() {
        cd /home/developer/repo
        /usr/bin/colcon "$@"
        source_workspace
    }

    clear_workspace() {
        rm -rf /home/developer/repo/build /home/developer/repo/install /home/developer/repo/log
    }

    source_workspace
fi
