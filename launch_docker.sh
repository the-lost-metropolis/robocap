#!/bin/bash

# Colors and styles for polished output
BOLD='\033[1m'
GREEN='\033[32m'
BLUE='\033[34m'
RED='\033[31m'
CYAN='\033[36m'
RESET='\033[0m'

# Function to get the hostname or fallback IP address
function get_hostname {
    # Try to get the hostname for .local mDNS (multicast DNS)
    HOSTNAME=$(hostname)
    LOCAL_ADDRESS="${HOSTNAME}.local"

    # Check if .local hostname resolves (this requires avahi-daemon or similar service)
    if ping -c 1 -W 1 "$LOCAL_ADDRESS" &> /dev/null; then
        echo "$LOCAL_ADDRESS"
    else
        # Fallback to using the machine's IP address if mDNS is not available
        IP_ADDRESS=$(hostname -I | awk '{print $1}')
        if [[ -n $IP_ADDRESS ]]; then
            echo "$IP_ADDRESS"
        else
            echo ""
        fi
    fi
}

# Function to print the access URL
function print_http_link {
    local port=${1:-8000}  # Default port is 8000 if not provided
    local hostname=$2      # Hostname to use in the URL

    if [[ -n $hostname ]]; then
        echo -e "${GREEN}${BOLD}Click here to remote into the development environment: ${CYAN}http://${hostname}:${port}${RESET}"
    else
        echo -e "${RED}${BOLD}Could not determine local network address.${RESET}"
    fi
}

# Function to prompt the user to launch VSCode
function prompt_vscode_launch {
    local hostname=$1  # Pass the hostname for the SSH connection
    local ssh_port=$2  # Pass the SSH port for the connection

    # Remove old SSH keys for the hostname to prevent conflicts
    echo -e "${BLUE}${BOLD}Clearing old SSH keys for ${hostname}...${RESET}"
    ssh-keygen -R "[${hostname}]:${ssh_port}" > /dev/null 2>&1

    read -p "$(echo -e "${BLUE}${BOLD}Would you like to open this environment in VSCode via SSH? (y/n): ${RESET}")" -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${GREEN}${BOLD}Launching VSCode with remote SSH on port ${ssh_port}...${RESET}"
        code --folder-uri "vscode-remote://ssh-remote+root@${hostname}:${ssh_port}/root/repo"
    else
        echo -e "${BLUE}${BOLD}Skipping VSCode launch.${RESET}"
    fi
}

# Function to check if a port is in use using netcat (nc) or ss
function is_port_in_use {
    local port=$1
    # Check if the port is in use using netcat or ss (socket statistics)
    if nc -z localhost $port 2>/dev/null || ss -tuln | grep ":$port" > /dev/null; then
        return 0  # Port is in use
    else
        return 1  # Port is free
    fi
}

# Initial ports
SSH_PORT=2222
WEB_PORT=8000

# Check and increment SSH port if necessary
while is_port_in_use $SSH_PORT; do
    SSH_PORT=$((SSH_PORT + 1))
done

# Check and increment web port if necessary
while is_port_in_use $WEB_PORT; do
    WEB_PORT=$((WEB_PORT + 1))
done

# Get the hostname or IP address
HOSTNAME=$(get_hostname)

# Notify the user of the ports to be used
echo -e "${BLUE}${BOLD}Launching Docker container on:${RESET}"
echo -e "${GREEN} - SSH port: ${BOLD}${SSH_PORT}${RESET}"
echo -e "${GREEN} - Web port: ${BOLD}${WEB_PORT}${RESET}"

# Print the access URL
print_http_link $WEB_PORT $HOSTNAME

# Prompt the user to launch VSCode via SSH
prompt_vscode_launch $HOSTNAME $SSH_PORT

# Launch the Docker container with the available ports and SSH key (if present)
docker run -it --rm \
  -p ${SSH_PORT}:22 \
  -p ${WEB_PORT}:8000 \
  -v $(realpath ./):/root/repo \
  -e IS_DEV_CONTAINER=true \
  rbc_dev_ros:latest