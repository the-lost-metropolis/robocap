prebuild() {
    ( # subshell such that it does not affect working directory of shell
        sudo apt update
        cd /home/developer/repo
        sudo rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        colcon build --symlink-install --event-handlers console_direct+
    )
}