-- Vertex

#version 430 core

in vec4 vertexPosition;
in vec2 vertexTexCoord;
out vec2 fragTexCoord;

void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment

#version 430 core

uniform sampler2DMS inputTexture;
uniform int numSamples;
in vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    ivec2 size = textureSize(inputTexture);
    ivec2 iCoords = ivec2(int(fragTexCoord.x*size.x), int(fragTexCoord.y*size.y));
    vec3 color = vec3(0, 0, 0); 
    for (int currSample = 0; currSample < numSamples; currSample++) {
        color += texelFetch(inputTexture, iCoords, currSample).rgb;
    }
    fragColor = vec4(color / numSamples, 1);
}
