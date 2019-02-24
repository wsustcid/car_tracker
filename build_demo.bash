#!/usr/bin/env bash

# get this script's absolute path to define the "Context" path
# for the Docker Engine
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo docker build -t osrf/car_demo $DIR
# sudo docker build -t osrf/car_demo .