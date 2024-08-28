#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "SCRIPT_DIR: $SCRIPT_DIR"

docker build -f $SCRIPT_DIR/docker/Dockerfile -t vdynse:v1 $SCRIPT_DIR/docker/..