#!/bin/bash

if [ -f /etc/debian_version ]; then
    DISTRIBUTION="debian"
elif [ -f /etc/arch-release ]; then
    DISTRIBUTION="arch"
else
    echo "No pre-built binary for your operating system. But you can compile from source."
    exit 1
fi

if [[ "$(uname -m)" != "x86_64" ]]; then
    echo "No pre-built binary for instruction set x86_64. But you can compile from source."
    exit 1
fi

if [[ "$BINARIES" == "game" || "$BINARIES" == "both" ]]; then
    BINARY_NAME="hacker-league"
    
    if [ "$DISTRIBUTION" == "debian" ]; then
        sudo apt-get update
        sudo apt-get install -y libglfw3 libcurl4-openssl-dev
    elif [ "$DISTRIBUTION" == "arch" ]; then
        sudo pacman -Syu --noconfirm glfw curl
    fi

    mkdir -p hacker-league
    cd hacker-league
    curl -L -o "$BINARY_NAME" "https://github.com/moritztng/hacker-league/releases/latest/download/${BINARY_NAME}_x86_64_${DISTRIBUTION}"
    chmod +x "$BINARY_NAME"
    curl -L -O "https://raw.githubusercontent.com/moritztng/hacker-league/refs/heads/main/{gamepad.txt,font.png}"
fi

if [[ "$BINARIES" == "server" || "$BINARIES" == "both" ]]; then
    BINARY_NAME="server"
    curl -L -o "$BINARY_NAME" "https://github.com/moritztng/hacker-league/releases/latest/download/${BINARY_NAME}_x86_64_${DISTRIBUTION}"
    chmod +x "$BINARY_NAME"
fi
