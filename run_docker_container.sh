#!/bin/bash
# Check if container is already running
if [ "$(docker ps -q -f name=gem_stack-container)" ]; then
    docker exec -it gem_stack-container bash
else
<<<<<<< HEAD
    docker compose -f setup/docker-compose.yaml up -d
=======
    UID=$(id -u) GID=$(id -g) docker compose -f setup/docker-compose.yaml up -d
>>>>>>> s2025_teamB
    docker exec -it gem_stack-container bash
fi