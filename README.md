https://github.com/user-attachments/assets/1ce9303e-a5ff-4ba6-9eac-6aa4ad94cb31
# Install
Currently only debian based distros with x86_64. Please help me build it on other platforms.
```bash
curl -sL https://raw.githubusercontent.com/moritztng/universe/main/install.sh | bash
```

## Play
```bash
./universe
```

## Build from source
```bash
sudo apt install libvulkan-dev vulkan-validationlayers-dev spirv-tools libglfw3-dev libglm-dev libeigen3-dev
make
curl -L -o ./shaders/glslc https://github.com/moritztng/universe/releases/download/v0.1/glslc
chmod +x ./shaders/glslc
./shaders/compile.sh
```
