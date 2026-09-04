#!/usr/bin/env bash

ORG=baroloteam
IMAGE=freisa_on_mars
TAG=latest

VCS_REF=""
VERSION=preview

# Exit script with failure if build fails
set -eo pipefail

echo ""
echo "##### Building freisa_on_mars Demo Docker Image #####"
echo ""

docker build -t $ORG/$IMAGE:$TAG \
    --build-arg VCS_REF="$VCS_REF" \
    --build-arg VERSION="$VERSION" .

echo ""
echo "##### Done! #####"

# EOF
