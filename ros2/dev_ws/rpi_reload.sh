#!/bin/bash

RPI_USER=pi

# Hostname set in /etc/hosts
RPI_IP=robomower

# Remotely restarts the docker compose instance with the latest config
# Normally, run this in its own terminal

if [ "$1" = "build" ]; then
  bash dev_build.sh
  sleep 3
fi

# Setup pub key auth
scp docker/docker-compose.yml $RPI_USER@$RPI_IP:~/
ssh $RPI_USER@$RPI_IP "date && docker compose down && docker-compose pull && docker-compose up"
