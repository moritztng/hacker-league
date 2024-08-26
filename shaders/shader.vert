#version 450

layout(binding = 0) uniform UniformBufferObject {
    mat4 model[3];
    mat4 view;
    mat4 proj;
} ubo;

layout(push_constant) uniform PushConstants {
    uint index;
};

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal; // Changed from inColor to inNormal

layout(location = 0) out vec3 fragPosition;
layout(location = 1) out vec3 fragNormal; // Changed from fragIndex to fragNormal
layout(location = 2) out uint fragIndex; // Changed from fragIndex to fragNormal

void main() {
    gl_Position = ubo.proj * ubo.view * ubo.model[index] * vec4(inPosition, 1.0);
    fragPosition = inPosition; // Transform to world space
    fragNormal = normalize(mat3(transpose(inverse(ubo.model[index]))) * inNormal); // Transform normal to world space
    fragIndex = index;
}
