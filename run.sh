#!/bin/bash

xhost +local:docker

docker compose up -d

docker exec -it fre_noetic bash
