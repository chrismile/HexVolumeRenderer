-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;

out VertexData
{
    vec3 linePosition;
    vec4 lineColor;
};

void main()
{
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineColor = vertexColor;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Vertex

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform sampler2d imageTexture;
uniform ivec2 imageTextureSize;

uniform sampler2d weightTexture;
uniform ivec2 weightTextureSize;

void main() {
    vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2, -weightTextureSize.y/2);
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0);
            vec2 textureCoordinates = samplePos / vec2(imageTextureSize.xy);
            vec4 colorSample = texture(imageTexture, textureCoordinates);
            color += weight * colorSample;
            samplePos.x++;
        }
        samplePos.y++;
    }
    fragColor = color;
}
