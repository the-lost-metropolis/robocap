source ~/.bashrc

if [ "$IS_DEV_CONTAINER" = "true" ]; then
    # This only runs when inside the container
    source /opt/ros/humble/setup.sh # Source underlay
    alias gui="xdg-open http://${SELF_HOSTNAME}:${SELF_WEB_PORT}"
fi
