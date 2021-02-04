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

xhost +
docker run \
    --privileged \
	--net="${NETWORK}" \
	-it \
    -p 5001:5000 \
    -p 8081:8080 \
    --rm \
	"${NAME}:${TAG}" \
    "supervisord"