#!/bin/bash

# Building locally to push onto the RPI will be faster than CI

cat .ghsecret | docker login ghcr.io -u martysweet --password-stdin
docker buildx build --platform linux/aarch64 -t ghcr.io/martysweet/lawn-mower:dev --push .