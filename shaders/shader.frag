#version 450

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) flat in uint index;

layout(location = 0) out vec4 color;

void main() {
    uint stripe = 10;
    vec3 lightPosition = vec3(0.0, 20.0, 0.0);
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    vec3 ambient = vec3(0.5, 0.5, 0.5);
    float lightIntensity = 10.0;
    float attenuationConstant = 1.0;
    float attenuationLinear = 0.09;
    float attenuationQuadratic = 0.032;

    vec3 color3d;
    if (index == 0) {
        if (normal.y > 0.5) {
            if (int(floor(position.z / stripe)) % 2 == 0) {
                color3d = vec3(0.0, 0.4, 0.0);
            } else {
                color3d = vec3(0.0, 0.5, 0.0);
            }
        } else {
            color3d = vec3(0.0, 1.0, 1.0);
        }
    } else if (index == 1) {
        color3d = vec3(1.0, 1.0, 1.0);
    } else {
        if (position.z > 0.5) {
            color3d = vec3(0.0, 1.0, 1.0);
        } else {
            color3d = vec3(0.05, 0.05, 0.05);
        }
    } 

    float distance = length(lightPosition - position);
    color = vec4(color3d, 1.0) * vec4(ambient + max(dot(normalize(normal), normalize(lightPosition - position)), 0.0) * lightIntensity * (1.0 / (attenuationConstant + attenuationLinear * distance + attenuationQuadratic * distance * distance)) * lightColor, 1.0);
}
