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

uniform vec4 clearColor;

uniform sampler2D imageTexture;
uniform ivec2 imageTextureSize;

uniform sampler2D weightTexture;
uniform ivec2 weightTextureSize;

void main() {
    vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0).x * 1.0;
            ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2 + x, -weightTextureSize.y/2 + y);
            vec2 textureCoordinates = samplePos / vec2(imageTextureSize.xy);
            vec4 colorSample = texture(imageTexture, textureCoordinates);
            color += weight * colorSample;
        }
    }
    fragColor = vec4(vec3(1.0) - clearColor.rgb, smoothstep(0.01, 1.0, color.r));
    if (fragColor.r < 0.01) {
        discard;
    }
    /*if (color.r > 0.001) {
        fragColor = vec4(color.rgb, 1.0);
    } else {
        fragColor = vec4(0.0);
    }*/
}
