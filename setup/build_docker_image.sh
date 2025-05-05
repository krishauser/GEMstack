#!/bin/bash

echo "Select CUDA version:"
echo "1) CUDA 11.8"
echo "2) CUDA 12+"
read -p "Enter choice [1-2]: " choice

case $choice in
    1)
        DOCKERFILE=setup/Dockerfile.cuda11.8
        ;;
    2)
        DOCKERFILE=setup/Dockerfile.cuda12
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

export DOCKERFILE
UID=$(id -u) GID=$(id -g) docker compose -f setup/docker-compose.yaml build