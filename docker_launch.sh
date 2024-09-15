#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to display usage instructions
usage() {
    echo "Usage: $0 [--headless]"
    echo "  --headless : Run container without launching VSCode"
    exit 1
}

# Function to get the hostname or fallback IP address
get_hostname() {
    hostname=$(hostname)
    if ping -c 1 -W 1 "${hostname}.local" &> /dev/null; then
        echo "${hostname}.local"
    else
        hostname -I | awk '{print $1}'
    fi
}

# Parse command-line arguments
HEADLESS=false
if [ "$1" == "--headless" ]; then
    HEADLESS=true
elif [ $# -gt 0 ]; then
    usage
fi

if [ "$IS_DEV_CONTAINER" = "true" ]; then
    echo "You are already in the Docker container."
else

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
        -e IS_DEV_CONTAINER=true \
        -e SELF_SSH_PORT=${SSH_PORT} \
        -e SELF_WEB_PORT=${WEB_PORT} \
        -e SELF_HOSTNAME=${HOSTNAME} \
        rbc_dev_ros:latest

    # Launch VSCode remote if not in headless mode
    if [ "$HEADLESS" = false ]; then
        # Remove existing SSH key entry
        if ssh-keygen -R "[$HOSTNAME]:$SSH_PORT"; then
            echo "Removed old SSH key entry for [$HOSTNAME]:$SSH_PORT."
        else
            echo "Failed to remove old SSH key entry. Please check if the hostname and port are correct."
            exit 1
        fi

        # Launch VSCode remote
        if code --folder-uri "vscode-remote://ssh-remote+developer@$HOSTNAME:$SSH_PORT/home/developer/repo"; then
            echo "VSCode successfully launched with SSH remote at $HOSTNAME:$SSH_PORT."
        else
            echo "Failed to launch VSCode with SSH remote. Please check your configuration."
            exit 1
        fi
    else
        echo -e "${GREEN}Container running in headless mode. VSCode not launched.${NC}"
    fi
fi
