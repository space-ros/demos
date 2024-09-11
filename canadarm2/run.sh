#!/bin/bash

docker compose down

if [ "$1" == "--isaacsim" ]; then
    docker compose up -d canadarm_demo_isaacsim
else
    docker compose up -d canadarm_demo canadarm_gui
fi
