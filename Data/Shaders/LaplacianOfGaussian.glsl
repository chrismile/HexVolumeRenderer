-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Fragment

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform sampler2D imageTexture;
uniform ivec2 imageTextureSize;

uniform sampler2D weightTexture;
uniform ivec2 weightTextureSize;

void main() {
    vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0).x;
            ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2 + x, -weightTextureSize.y/2 + y);
            vec2 textureCoordinates = samplePos / vec2(imageTextureSize.xy);
            vec4 colorSample = texture(imageTexture, textureCoordinates);
            color += weight * colorSample;
        }
    }
    if (color.r > 0.001) {
        fragColor = vec4(color.rgb, 1.0);
    } else {
        fragColor = vec4(0.0);
    }
}
