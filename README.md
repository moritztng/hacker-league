## Install
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
