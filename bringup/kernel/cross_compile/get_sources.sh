#!/bin/bash
FOLDER=/home/taylor/Software/kernel_acorn
TAG_NAME=1.20230405
echo $FOLDER
mkdir -p $FOLDER
echo "git clone --depth=1 --branch $TAG_NAME https://github.com/raspberrypi/linux $FOLDER"
