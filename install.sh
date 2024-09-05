#!/bin/bash

BINARY_NAME="universe"
sudo apt-get update
sudo apt-get install -y libglfw3
if ! lspci | grep -iqE 'vga.*(nvidia|radeon)'; then
    sudo apt-get install -y mesa-vulkan-drivers
else
    echo "GPU detected. Make sure that GPU drivers are installed."
fi
mkdir universe
cd universe
curl -L -o "$BINARY_NAME" "https://github.com/moritztng/universe/releases/latest/download/$BINARY_NAME"
chmod +x "$BINARY_NAME"
curl -L -o "gamepad.txt" https://raw.githubusercontent.com/mdqinc/SDL_GameControllerDB/master/gamecontrollerdb.txt
