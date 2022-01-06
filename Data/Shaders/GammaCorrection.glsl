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

uniform sampler2D inputTexture;
in vec2 fragTexCoord;
out vec4 fragColor;

// Converts linear RGB to sRGB
vec3 toSRGB(vec3 u) {
    return mix(1.055 * pow(u, vec3(1.0 / 2.4)) - 0.055, u * 12.92, lessThanEqual(u, vec3(0.0031308)));
}

void main() {
    vec4 linearColor = texture(inputTexture, fragTexCoord);
    fragColor = vec4(toSRGB(linearColor.rgb), linearColor.a);
}
