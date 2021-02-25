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
    --opt device=$PWD/src \
    --opt o=bind \
   "src_vol"

docker volume create --driver local \
    --opt type=none \
    --opt device=$PWD/userdata \
    --opt o=bind \
   "userdata_vol"

xhost +
docker run \
    --privileged \
	--net="${NETWORK}" \
    --volume="src_vol:/home/ros2/src:rw" \
    --volume="userdata_vol:/home/ros2/userdata:rw" \
	-it \
    -p 5000:5000 \
    -p 8081:8080 \
    --rm \
	"${NAME}:${TAG}"