#!/bin/bash

# Function to display usage instructions
usage() {
    echo "Usage: $0 <ssh_port> [hostname]"
    echo "  <ssh_port> : SSH port number"
    echo "  [hostname] : Optional, defaults to current hostname"
    exit 1
}

# Check if at least one argument is provided
if [ $# -lt 1 ]; then
    usage
fi

# Assign the provided arguments to variables
SSH_PORT=$1
HOSTNAME=${2:-$(hostname)}  # Use current hostname if not provided

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
