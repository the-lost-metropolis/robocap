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

# Check if SSH public key exists
SSH_KEY_MOUNT=""
if [ -f "$HOME/.ssh/id_rsa.pub" ]; then
    echo -e "${BLUE}SSH public key found. Mounting into container.${NC}"
    SSH_KEY_MOUNT="-v $HOME/.ssh/id_rsa.pub:/home/developer/.ssh/authorized_keys"
else
    echo -e "${BLUE}No SSH public key found.${NC}"
fi

# Print information
echo -e "${BLUE}Launching Docker container on:${NC}"
echo -e "${GREEN}SSH port: ${SSH_PORT}${NC}"
echo -e "${GREEN}Web port: ${WEB_PORT}${NC}"
echo -e "${GREEN}GUI URL: http://${HOSTNAME}:${WEB_PORT}${NC}"
echo -e "${GREEN}Use ./remote_vscode.sh ${SSH_PORT} ${HOSTNAME} to launch VSCode in the container${NC}"
echo -e "${GREEN}Using UID: ${HOST_UID}, GID: ${HOST_GID}${NC}"

# Launch the Docker container in the background
docker run -it --rm \
    --user ${HOST_UID}:${HOST_GID} \
    -p ${SSH_PORT}:22 \
    -p ${WEB_PORT}:8000 \
    -v $(realpath ./):/home/developer/repo \
    -v $(realpath ./docker/start.sh):/home/developer/start.sh \
    -v $(realpath ~/.gitconfig):/home/developer/.gitconfig \
    $SSH_KEY_MOUNT \
    -e DISPLAY=:0 \
    rbc_dev_ros:latest