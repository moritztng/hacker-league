#version 450

layout(location = 0) in vec3 fragPosition;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) flat in uint fragIndex;

layout(location = 0) out vec4 outColor;

void main() {
    // Point light parameters (hardcoded)
    vec3 lightPos = vec3(0.0, 20.0, 0.0);  // Position of the point light
    vec3 lightColor = vec3(1.0, 1.0, 1.0); // White light
    float lightIntensity = 10.0;            // Light intensity
    float constant = 1.0;                  // Constant attenuation term
    float linear = 0.09;                   // Linear attenuation term
    float quadratic = 0.032;               // Quadratic attenuation term

    // Ambient light (uniform across the entire surface)
    vec3 ambient = vec3(0.5, 0.5, 0.5); // Low-intensity ambient light

    // Normalize the normal vector
    vec3 norm = normalize(fragNormal);

    // Calculate vector from fragment position to light source
    vec3 lightDir = normalize(lightPos - fragPosition);

    // Calculate diffuse lighting using Lambert's cosine law
    float diff = max(dot(norm, lightDir), 0.0);

    // Calculate distance and attenuation
    float distance = length(lightPos - fragPosition);
    float attenuation = 1.0 / (constant + linear * distance + quadratic * distance * distance);

    // Final light contribution
    vec3 diffuse = diff * lightColor * lightIntensity * attenuation;

    // Default color is cyan
    vec4 color;
    if (fragIndex == 0) {
        if (fragPosition.z > 0.5) {
            color = vec4(0.0, 1.0, 1.0, 1.0);
        } else {
            color = vec4(0.05, 0.05, 0.05, 1.0);
        }
    } else if (fragIndex == 1) {
        color = vec4(1.0, 1.0, 1.0, 1.0);
    } else if (fragIndex == 2) {
        // Apply checkerboard pattern
        if (fragPosition.y < -9.9) {
            uint tileWidth = 10;
            bool isBlack = int(floor(fragPosition.z / tileWidth)) % 2 == 0;

            // Assign colors based on the pattern
            if (isBlack) {
                color = vec4(0.0, 0.392, 0.0, 1.0); // Dark green
            } else {
                color = vec4(0.0, 0.5, 0.0, 1.0);   // Green
            }
        } else {
            color = vec4(0.0, 1.0, 1.0, 1.0);
        }
    }

    // Combine ambient and diffuse lighting
    vec3 finalColor = ambient + diffuse;

    // Apply final lighting to the base color
    outColor = color * vec4(finalColor, 1.0);
}