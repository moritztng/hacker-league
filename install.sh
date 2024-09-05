#!/bin/bash

BINARY_NAME="hacker-league"
sudo apt-get update
sudo apt-get install -y libglfw3
if ! lspci | grep -iqE 'vga.*(nvidia|radeon)'; then
    sudo apt-get install -y mesa-vulkan-drivers
else
    echo "GPU detected. Make sure that GPU drivers are installed."
fi
mkdir hacker-league
cd hacker-league
curl -L -o "$BINARY_NAME" "https://github.com/moritztng/hacker-league/releases/latest/download/$BINARY_NAME"
chmod +x "$BINARY_NAME"
curl -L -o "gamepad.txt" https://raw.githubusercontent.com/mdqinc/SDL_GameControllerDB/master/gamecontrollerdb.txt
