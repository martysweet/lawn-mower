#!/bin/bash

# Hostname set in /etc/hosts
RPI_USER=pi
RPI_IP=robomower


# Setup pub key auth
ssh $RPI_USER@$RPI_IP -t "watch -n1 docker ps"
