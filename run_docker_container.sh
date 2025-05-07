#!/bin/bash
# Check if container is already running
if [ "$(docker ps -q -f name=gem_stack-container)" ]; then
    docker exec -it gem_stack-container bash
else
    UID=$(id -u) GID=$(id -g) docker compose -f setup/docker-compose.yaml up -d
    docker exec -it gem_stack-container bash
fi