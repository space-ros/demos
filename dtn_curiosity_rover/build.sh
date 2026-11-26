#!/bin/bash
# Build the demo images: the two cFS/BPNode DTN nodes, the RACS2 rover image from
# ../racs2_demos_on_spaceros and the Curiosity Gazebo image from ../curiosity_rover.
set -e
cd "$(dirname "$0")"
docker compose build
