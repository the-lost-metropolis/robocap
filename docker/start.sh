#!/bin/bash

# Adjust permissions if necessary
if [ "$(id -u)" != "1000" ] || [ "$(id -g)" != "1000" ]; then
    echo "Adjusting permissions..."
    for item in /home/developer/.* /home/developer/*; do
        if [ "$item" != "/home/developer/repo" ] && [ "$item" != "/home/developer/." ] && [ "$item" != "/home/developer/.." ]; then
            sudo chown -R $(id -u):$(id -g) "$item"
            sudo chmod -R u+rw "$item"
        fi
    done
fi

# Dump root shell environment to file
printenv | sed 's/^\(.*\)$/export \1/' > /home/developer/project_env.sh
echo "source /home/developer/project_env.sh" >> /home/developer/.bashrc

sudo apt-get update

sudo /usr/sbin/sshd
xpra start --bind-tcp=0.0.0.0:8000
tail -f /dev/null

