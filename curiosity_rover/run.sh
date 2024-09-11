#!/bin/bash

docker-compose down

if [ "$1" == "--isaacsim" ]; then
    docker compose up -d curiosity_demo_isaacsim
else
    docker compose up -d curiosity_demo curiosity_gui
fi
