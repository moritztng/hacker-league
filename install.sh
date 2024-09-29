#!/bin/bash

if [[ "$BINARIES" == "game" || "$BINARIES" == "both" ]]; then
    BINARY_NAME="hacker-league"
    sudo apt-get update
    sudo apt-get install -y libglfw3 libcurl4-openssl-dev
    if ! lspci | grep -iqE 'vga.*(nvidia|radeon)'; then
        echo "Integrated graphics detected"
        sudo apt-get install -y mesa-vulkan-drivers
    else
        echo "GPU detected. Make sure that GPU drivers are installed"
    fi
    mkdir hacker-league
    cd hacker-league
    curl -L -o "$BINARY_NAME" "https://github.com/moritztng/hacker-league/releases/latest/download/$BINARY_NAME"
    chmod +x "$BINARY_NAME"
    curl -L -O "https://raw.githubusercontent.com/moritztng/hacker-league/refs/heads/main/{gamepad.txt,font.png}"
fi
if [[ "$BINARIES" == "server" || "$BINARIES" == "both" ]]; then
    BINARY_NAME="server"
    curl -L -o "$BINARY_NAME" "https://github.com/moritztng/hacker-league/releases/latest/download/$BINARY_NAME"
    chmod +x "$BINARY_NAME"
fi
