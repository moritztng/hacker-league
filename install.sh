#!/bin/bash

BINARY_URL="https://github.com/moritztng/universe/releases/latest/download/universe"
BINARY_NAME="universe"
sudo apt-get update
sudo apt-get install -y libglfw3 wget
if ! lspci | grep -iqE 'vga.*(nvidia|radeon)'; then
    sudo apt-get install -y mesa-vulkan-drivers
else
    echo "GPU detected. Make sure that GPU drivers are installed."
fi
wget -O "$BINARY_NAME" "$BINARY_URL"
chmod +x "$BINARY_NAME"
