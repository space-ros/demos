#!/bin/bash
echo "Starting docker container..."
docker run -it \
  --network host \
  -e DISPLAY \
  -e TERM \
  -e QT_X11_NO_MITSHM=1 \
  --device /dev/dri \
  spaceros_gz_demos:latest
