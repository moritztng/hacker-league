https://github.com/user-attachments/assets/3a630d46-ec17-4da8-8879-76320ea563fe
# Install
Currently only debian based distros with x86_64. Please help me build it on other platforms. If you have an external GPU, make sure the drivers are installed
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
git clone https://github.com/moritztng/hacker-league.git
cd hacker-league
sudo apt install libvulkan-dev vulkan-validationlayers-dev spirv-tools libglfw3-dev libglm-dev libeigen3-dev vim-common xxd g++ make libsqlite3-dev libcurl4-openssl-dev
curl -L -o ./shaders/glslc https://github.com/moritztng/hacker-league/releases/download/glslc/glslc
chmod +x ./shaders/glslc
make debug
```
## Community
- Discord Server: https://discord.gg/BbNH27st
- I build in public on X: https://x.com/moritzthuening
