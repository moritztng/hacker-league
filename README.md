https://github.com/user-attachments/assets/3a630d46-ec17-4da8-8879-76320ea563fe
# Install
Currently only debian based distros with x86_64. Please help me build it on other platforms
```bash
sudo apt install curl && curl -sL https://raw.githubusercontent.com/moritztng/hacker-league/main/install.sh | bash
```

## Play
Use a gamepad for maximum fun
```bash
cd hacker-league
./hacker-league
```

## Build from source
```bash
sudo apt install libvulkan-dev vulkan-validationlayers-dev spirv-tools libglfw3-dev libglm-dev libeigen3-dev vim-common xxd g++ make
curl -L -o ./shaders/glslc https://github.com/moritztng/hacker-league/releases/download/glslc/glslc
chmod +x ./shaders/glslc
make debug
curl -L -o "gamepad.txt" https://raw.githubusercontent.com/mdqinc/SDL_GameControllerDB/master/gamecontrollerdb.txt
```
