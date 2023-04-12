#!/bin/bash

echo "Launch docker container for development."
docker run --name ymd_test --mount type=bind,src=$(pwd),target=/home/test_user/dev_ws -it ymd:dev
