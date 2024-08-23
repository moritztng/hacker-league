#version 450

layout(binding = 0) uniform UniformBufferObject {
    mat4 model[2];
    mat4 view;
    mat4 proj;
} ubo;

layout(push_constant) uniform PushConstants {
    uint index;
};

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 fragColor;

void main() {
    gl_Position = ubo.proj * ubo.view * ubo.model[index] * vec4(inPosition, 1.0);
    fragColor = inColor;
}
