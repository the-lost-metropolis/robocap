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

# Function to wait for SSH server to be fully ready
wait_for_ssh() {
    echo -e "${BLUE}Waiting for SSH server on port ${SSH_PORT} to respond...${NC}"
    until ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o BatchMode=yes -o ConnectTimeout=5 developer@localhost -p ${SSH_PORT} exit &> /dev/null; do
        echo "SSH server not ready yet. Retrying..."
        sleep 1
    done
}

# Function to handle Ctrl+C (SIGINT)
cleanup() {
    echo -e "${BLUE}Caught SIGINT, cleaning up...${NC}"
    # Kill any background jobs (e.g., polling process)
    pkill -P $$
    exit 1
}

# Set the trap for SIGINT (Ctrl+C)
trap cleanup SIGINT

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

    # We first build the docker images
    (
        SCRIPT_DIR=$(dirname "$(readlink -f "$0")") # Get the directory of this script
        docker build -f docker/Dockerfile.rbc_dev_base -t rbc_dev_base:latest .
        docker build -f docker/Dockerfile.rbc_dev_ros -t rbc_dev_ros:latest .
        docker build -f docker/Dockerfile.rbc_dev_ros_prebuild -t rbc_dev_ros_prebuild:latest .
    )

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
    echo Launching Docker with the following configuration:
    echo -e "Hostname: ${HOSTNAME}, SSH port: ${SSH_PORT}, Web port: ${WEB_PORT}"
    echo -e "UID: ${HOST_UID}, GID: ${HOST_GID}"

    # Start polling for SSH server availability in the background
    ( 
        wait_for_ssh

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
        sleep 2
        echo -e "✅ ${GREEN}Development environment is ready. To manually start a remote session into the container, run \"./remote_vscode.sh ${SSH_PORT} ${HOSTNAME}\", or to open the remote GUI, visit http://${HOSTNAME}:${WEB_PORT}.${NC}"
    ) &

    # Capture the PID of the background process
    BG_PID=$!

    echo $(pwd)

    # Launch the Docker container in the foreground
    docker run -it --rm \
        --user ${HOST_UID}:${HOST_GID} \
        -p ${SSH_PORT}:22 \
        -p ${WEB_PORT}:8000 \
        -v $(realpath ./):/home/developer/repo \
        -v $(realpath ./docker/entrypoint.sh):/home/developer/entrypoint.sh \
        -v $(realpath ~/.gitconfig):/home/developer/.gitconfig \
        $SSH_KEY_MOUNT \
        -e DISPLAY=:0 \
        -e IS_DEV_CONTAINER=true \
        -e SELF_SSH_PORT=${SSH_PORT} \
        -e SELF_WEB_PORT=${WEB_PORT} \
        -e SELF_HOSTNAME=${HOSTNAME} \
        rbc_dev_ros_prebuild:latest

    # Kill the background process if it exists
    if [ -n "$BG_PID" ]; then
        kill $BG_PID
    fi
fi
