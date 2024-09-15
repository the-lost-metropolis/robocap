# This script does all the dependency installation and building of the workspace. It is called when building rbc_dev_ros_prebuild and also during launch in start.sh (the second pass will cover whatever's missed!)

prebuild() {
    ( # subshell such that it does not affect working directory of shell
        sudo apt update
        cd /home/developer/repo
        sudo rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        # Print user, cwd
        echo "User: $(whoami)"
        echo "CWD: $(pwd)"
        colcon build --symlink-install --event-handlers console_direct+
    )
}