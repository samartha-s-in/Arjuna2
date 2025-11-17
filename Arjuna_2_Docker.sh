#!/bin/bash

set -e
set -o pipefail

cd ~


git clone https://github.com/NewrroTechLLP/arjuna_R2_docker.git

cd arjuna_R2_docker/ && chmod +x *

./arjuna_docker_install.sh

echo "Setup complete! Your Docker and Arjuna ROS2 Workspace is ready."
