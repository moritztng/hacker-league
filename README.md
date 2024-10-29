https://github.com/user-attachments/assets/e8a15f8f-6383-4ccb-a74a-a1911f3507eb
# Hacker League
## Install
Currently only debian and arch based distros with x86_64. On other platforms you have to build it from source. Please help me build it on other platforms. Make sure vulkan drivers are installed on your machine.
```bash
curl -sL https://raw.githubusercontent.com/moritztng/hacker-league/main/install.sh | BINARIES="game" bash
cd hacker-league
```
## Play
Use a gamepad for maximum fun
### Singleplayer
```bash
./hacker-league
```
### Multiplayer
Choose server from public server list
```bash
./hacker-league servers
```
Or connect to server with `server-ip` and `server-port`
```bash
./hacker-league <server-ip> <server-port>
```
# Server
## Install
```bash
curl -sL https://raw.githubusercontent.com/moritztng/hacker-league/main/install.sh | BINARIES="server" bash
```
## Run
Specify `public-ip` and `public-port` if the server should be added to the public server list
```bash
./server <local-port> [<public-ip>] [<public-port>]
```
# Python Reinforcement Learning Environment
## Install
Currently only debian and arch based distros with x86_64. Make sure vulkan drivers are installed on your machine.
### Debian Dependencies
```bash
sudo apt-get install libglfw3 libcurl4-openssl-dev
```
### Arch Dependencies
```bash
sudo pacman -S glfw curl
```
### Python Package
```bash
pip install git+https://github.com/moritztng/hacker-league.git
```
## Train
See [train_example.py](train_example.py)
```python
best_params = evolution_strategies(model.get_params(), fitness)
model.set_params(best_params)
def policy(observation):
    return model.forward(observation)
hacker_league.play(policy) # opens the game and shows the agent playing
```
# Build from source
## Install dependencies
### Debian
```bash
sudo apt update
sudo apt install libvulkan-dev vulkan-validationlayers-dev spirv-tools libglfw3-dev libglm-dev vim-common xxd g++ make libcurl4-openssl-dev git
```
### Arch
```bash
sudo pacman -Syu vulkan-headers vulkan-validation-layers spirv-tools glfw glm vim xxd gcc make curl git
```
### Slackware 15.0 
Required dependencies and where to acquire them (not complete):
 * glfw3 - https://slackbuilds.org/repository/15.0/libraries/glfw3/

## Build
```bash
git clone https://github.com/moritztng/hacker-league.git
cd hacker-league
curl -L -o ./shaders/glslc https://github.com/moritztng/hacker-league/releases/download/glslc/glslc
chmod +x ./shaders/glslc
make debug
```
## Community
- Discord Server: https://discord.gg/bD2saQqdTG
- I build in public on X: https://x.com/moritzthuening
