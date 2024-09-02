#!/bin/bash

for shader in vert frag; do
  ./shaders/glslc ./shaders/shader.$shader -o ./shaders/$shader.spv
done
