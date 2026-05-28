#!/bin/bash

xhost +local:root
xhost +local:docker

docker compose up -d
docker exec -it fre_noetic bash
