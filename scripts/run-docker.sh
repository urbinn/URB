#!/bin/bash
DIRECTORY=$1

if [ -d "$1" ]; then
  docker build -t urbinn -f $1/Dockerfile .
  # Base a development image on the base image
  docker run --name urbinn-dev -v $DIRECTORY:/urbinn -it urbinn /bin/bash
else
  echo $1
  echo "Excepting second argument: host project directory."
fi
