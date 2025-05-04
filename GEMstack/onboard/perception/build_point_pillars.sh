#!/bin/bash

# Check if nvidia-docker is installed
if ! command -v nvidia-container-toolkit &> /dev/null; then
    echo "NVIDIA Container Toolkit not found. Do you want to install it? (y/n)"
    read -p ">" choice
    if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
        echo "Installing NVIDIA Container Toolkit..."
        ./setup/get_nvidia_container.sh
    else
        echo "NVIDIA Container Toolkit is required for GPU support."
        echo "You can install it later by running ./get_nvidia_container.sh"
    fi
fi

# Check if the point_pillars_node.py and model weights exist
if [ ! -f "point_pillars_node.py" ]; then
    echo "ERROR: point_pillars_node.py not found in the current directory!"
    echo "Please place your point_pillars_node.py file in the same directory as this script."
    exit 1
fi

if [ ! -f "epoch_160.pth" ]; then
    echo "WARNING: epoch_160.pth model weights not found in the current directory!"
    echo "Please place your model weights file in the same directory as this script."
    echo "Continue anyway? (y/n)"
    read -p ">" choice
    if [ "$choice" != "y" ] && [ "$choice" != "Y" ]; then
        exit 1
    fi
fi

echo "Building Point Pillars Docker container..."
export DOCKERFILE=setup/Dockerfile.cuda111

# Using sudo to handle permissions
MY_UID=$(id -u)
MY_GID=$(id -g)

# Attempt to use docker-compose directly, then with sudo if needed (if you uncomment it)
if ! docker compose -f setup/docker-compose.yaml build; then
    echo "Uncomment these lines if you wish to use sudo to build container as backup"
    # echo "Using sudo to build the container..."
    # sudo -E docker compose -f setup/docker-compose.yaml build
fi

echo "Build complete. To start the container, run:"
echo "docker compose -f setup/docker-compose.yaml up"
echo ""
echo "Or with sudo if you have permission issues:"
echo "sudo docker compose -f setup/docker-compose.yaml up"
echo ""
echo "To run in detached mode (background):"
echo "docker compose -f setup/docker-compose.yaml up -d"
echo "Or: sudo docker compose -f setup/docker-compose.yaml up -d"
