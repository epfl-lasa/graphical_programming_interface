#!/bin/bash
NAME=$(echo "${PWD##*/}" | tr _ -)
TAG=$(echo "$1" | tr _/ -)

ISISOLATED=true # change to  false to use host network

NETWORK=host
if [ "${ISISOLATED}" = true ]; then
    docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
    NETWORK=isolated
fi

if [ -z "$TAG" ]; then
	TAG="latest"
fi

docker volume create --driver local \
    --opt type=none \
    --opt device=$PWD/backend \
    --opt o=bind \
   "backend_vol"

docker volume create --driver local \
    --opt type=none \
    --opt device=$PWD/module_library \
    --opt o=bind \
   "module_library_vol"

xhost +
docker run \
    --privileged \
	--net="${NETWORK}" \
    --volume="backend_vol:/home/ros2/backend:rw" \
    --volume="module_library_vol:/home/ros2/module_library:rw" \
	-it \
    -p 5000:5000 \
    -p 8081:8080 \
    --rm \
	"${NAME}:${TAG}" \
    "supervisord"