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

void main() {
    fragColor = texture(inputTexture, fragTexCoord);
}

-- Vertex.NoTexCoord

#version 430 core

in vec4 vertexPosition;
out vec2 fragTexCoord;

void main() {
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.NoTexCoord

#version 430 core

uniform sampler2D inputTexture;
uniform ivec2 viewportSize;
in vec4 gl_FragCoord;
out vec4 fragColor;

void main() {
    fragColor = texture(inputTexture, gl_FragCoord.xy / vec2(viewportSize - ivec2(1, 1)));
}
