#!/bin/bash

# WGET this file on the RPI and execute it
# $ curl https://raw.githubusercontent.com/martysweet/lawn-mower/main/ros2/dev_ws/rpi_fetch_and_run.sh --output rpi_fetch_and_run.sh
# $ chmod +x rpi_fetch_and_run.sh && bash rpi_fetch_and_run.sh install
# bash rpi_fetch_and_run.sh install

# Ensure docker is installed
if [ $1 = "install" ]; then
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh get-docker.sh
  sudo usermod -aG docker pi
  sudo apt-get install libffi-dev libssl-dev python3 python3-pip python3-dev -y
  sudo pip3 install docker-compose
  sudo systemctl enable docker
  echo "DONE! Now run bash to reinitialise your terminal (logout). Then bash rpi_fetch_and_run.sh"
  exit
fi

# Pulls the latest docker-compose
curl https://raw.githubusercontent.com/martysweet/lawn-mower/main/ros2/dev_ws/docker/docker-compose.yml --output docker-compose.yml
docker-compose down
docker-compose pull
docker-compose up -d
