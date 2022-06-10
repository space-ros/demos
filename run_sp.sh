SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
sudo docker run --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --volume="$SCRIPTPATH:/root/src/spaceros_demo_ws/src/demo:rw" -it openrobotics/spaceros-demo
