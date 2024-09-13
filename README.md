# Installation
## Prerequisites
- Linux host
- Docker
## Launching the development environment
- 1. Build the docker images: `./build_docker.sh`
- 2. Launch the container: `./launch_docker.sh` -> This will be assigned a port for ssh and a web interface
- 3. Remote with VSCode into the container using SSH with the provided port: `./remote_vscode.sh <port> <optional: hostname (defaults to own)>`
- 4. If you run any GUI applications, launch the web interface (URL provided in step 2) and the application will be displayed there