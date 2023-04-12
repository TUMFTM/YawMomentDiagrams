#!/bin/bash

echo "Build docker image for development."
docker build -f ./docker/Dockerfile -t ymd:dev .
