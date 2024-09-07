https://github.com/user-attachments/assets/3a630d46-ec17-4da8-8879-76320ea563fe
# Install
Currently debian and arch based distros with x86_64. Please help me build it on other platforms. If you have an external GPU, make sure the drivers are installed
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

Debian based:
```bash
git clone https://github.com/moritztng/hacker-league.git
cd hacker-league
sudo apt install libvulkan-dev vulkan-validationlayers-dev spirv-tools libglfw3-dev libglm-dev libeigen3-dev vim-common xxd g++ make
curl -L -o ./shaders/glslc https://github.com/moritztng/hacker-league/releases/download/glslc/glslc
chmod +x ./shaders/glslc
make debug
curl -L -o "gamepad.txt" https://raw.githubusercontent.com/mdqinc/SDL_GameControllerDB/master/gamecontrollerdb.txt
```

Arch based / Manjaro etc.:
```bash
pacman -Syu (for updating your system first)
pacman -S curl git vim xxd make base-devel vulkan-validation-layers vulkan-headers spirv-tools glfw glm eigen vim make
git clone https://github.com/moritztng/hacker-league.git
cd hacker-league
make
curl -L -o ./shaders/glslc https://github.com/moritztng/hacker-league/releases/download/glslc/glslc
chmod +x ./shaders/glslc
make debug
or make -j32 debug if you want to build 32 jobs at once (when you have a cpu with multiple cores which can do 2 threads at a time)
curl -L -o "gamepad.txt" https://raw.githubusercontent.com/mdqinc/SDL_GameControllerDB/master/gamecontrollerdb.txt
```
Then you find the binary hacker-league that you can just run with:
./hacker-league

Have fun!

## Community
- Discord Server: https://discord.gg/BbNH27st
- I build in public on X: https://x.com/moritzthuening
