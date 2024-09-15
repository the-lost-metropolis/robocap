if [ "$IS_DEV_CONTAINER" = "true" ]; then
    echo You are already in the docker container.
else
    SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
    cd $SCRIPT_DIR
    cd docker
    docker build -f Dockerfile.rbc_dev_base -t rbc_dev_base:latest .
    docker build -f Dockerfile.rbc_dev_ros -t rbc_dev_ros:latest .
fi