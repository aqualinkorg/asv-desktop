#!/usr/bin/env bash
set -e

# Default settings
NETWORK_CONFIG="bridge"
IMAGE_NAME="missionrobotics/ros2:latest"
USERNAME="user"

# X11/Display
XAUTH=$HOME/.Xauthority
touch $XAUTH
DISPLAY_ARGS="--env=DISPLAY"
X11_ARGS="--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --env=XAUTHORITY=$XAUTH --volume=$XAUTH:$XAUTH"

# Handle options
for i in "$@"
do
case $i in
    --nvidia)
    NVIDIA_DOCKER_ARGS="--gpus all -e NVIDIA_VISIBLE_DEVICES=\"all\" -e \"QT_X11_NO_MITSHM=1\""
    shift
    ;;
    --host)
    NETWORK_CONFIG="host"
    shift
    ;;
    --priv)
    PRIVILIGED_ARGS="--privileged"
    shift
    ;;
    --ipc)
    IPC_ARGS="--ipc=host"
    shift
    ;;
    *)
        # unknown option
    ;;
esac
done

NETWORK_ARGS="--network=${NETWORK_CONFIG}"

# Share current directory as the workspace in the container
WORKSPACE_DIR="${PWD}"
WORKSPACE_ARG="--volume=${WORKSPACE_DIR}:/home/${USERNAME}/workspace/"

# Local development files (caches, config, etc) get stored here
DEV_CACHE_DIR="${HOME}/.local/mr/"
mkdir -p ${DEV_CACHE_DIR}

DATA_DIR="${DEV_CACHE_DIR}/data"
mkdir -p "${DATA_DIR}"
DATA_DIR_ARG="--volume=${DATA_DIR}:/home/${USERNAME}/data"

# ==================================
# Conan

# Share conan cache (make directories if they don't exist)
CONAN_DATA_DIR="${DEV_CACHE_DIR}/conan/data"
mkdir -p "${CONAN_DATA_DIR}"
CONAN_CACHE_ARG="--volume=${CONAN_DATA_DIR}:/opt/mr/.conan/data"

# ===================================

docker run -it \
    ${NETWORK_ARGS} \
    ${PRIVILIGED_ARGS} \
    ${IPC_ARGS} \
    ${NVIDIA_DOCKER_ARGS} \
    ${X11_ARGS} \
    ${DISPLAY_ARGS} \
    ${WORKSPACE_ARG} \
    ${DATA_DIR_ARG} \
    ${CONAN_CACHE_ARG} \
    --workdir "/home/${USERNAME}/workspace/" \
    --rm \
    "${IMAGE_NAME}" \
    bash