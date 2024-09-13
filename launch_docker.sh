#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to get the hostname or fallback IP address
get_hostname() {
    hostname=$(hostname)
    if ping -c 1 -W 1 "${hostname}.local" &> /dev/null; then
        echo "${hostname}.local"
    else
        hostname -I | awk '{print $1}'
    fi
}

# Set up ports
SSH_PORT=2222
WEB_PORT=8000

# Find available ports if the defaults are in use
while nc -z localhost $SSH_PORT 2>/dev/null; do
    SSH_PORT=$((SSH_PORT + 1))
done
while nc -z localhost $WEB_PORT 2>/dev/null; do
    WEB_PORT=$((WEB_PORT + 1))
done

# Get the hostname or IP
HOSTNAME=$(get_hostname)

# Get current user's UID and GID
HOST_UID=$(id -u)
HOST_GID=$(id -g)

# Print information
echo -e "${BLUE}Launching Docker container on:${NC}"
echo -e "${GREEN}SSH port: ${SSH_PORT}${NC}"
echo -e "${GREEN}Web port: ${WEB_PORT}${NC}"
echo -e "${GREEN}Access URL: http://${HOSTNAME}:${WEB_PORT}${NC}"
echo -e "${GREEN}Using UID: ${HOST_UID}, GID: ${HOST_GID}${NC}"

# Create a named volume for XDG_RUNTIME_DIR
docker volume create xdg_runtime_dir

# Launch the Docker container
docker run -it --rm \
  --user ${HOST_UID}:${HOST_GID} \
  -p ${SSH_PORT}:22 \
  -p ${WEB_PORT}:8000 \
  -v $(realpath ./):/home/developer/repo \
  -v xdg_runtime_dir:/run/user/${HOST_UID} \
  -e DISPLAY=:0 \
  -e HOME=/home/developer \
  rbc_dev_ros:latest