if [ "$IS_DEV_CONTAINER" = "true" ]; then
    # This only runs when inside the container
    source /opt/ros/humble/setup.sh # Source underlay
fi