#version 450

layout(location = 0) in vec3 fragPosition;
layout(location = 1) flat in uint fragIndex;

layout(location = 0) out vec4 outColor;

void main() {
    if (fragIndex == 0) {
        outColor = vec4(1.0, 0.0, 0.0, 1.0);
    } else if (fragIndex == 1) {
        outColor = vec4(0.0, 0.0, 1.0, 1.0);
    } else if (fragIndex == 2) {
        if (fragPosition.y < -9.9) {
            uint tileWidth = 10;
            bool isBlack = int(floor(fragPosition.z / tileWidth)) % 2 == 0;
    
            // Assign colors based on the pattern
            if (isBlack) {
                outColor = vec4(0.0, 0.392, 0.0, 1.0); // Black color
            } else {
                outColor = vec4(0.0, 0.5, 0.0, 1.0); // White color
            }
        } else {
            outColor = vec4(0.0, 1.0, 1.0, 1.0);
        }
    } else {
        outColor = vec4(1.0);
    }
}
