#!/bin.bash

gnome-terminal -- sudo bash pigpiod-roscore-local.sh
gnome-terminal -- sudo bash program-local.sh
gnome-terminal -- bash position-echo-local.sh