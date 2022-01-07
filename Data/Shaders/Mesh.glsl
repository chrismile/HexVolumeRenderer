-- Vertex.Plain

#version 430 core

in vec4 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.Plain

#version 430 core

uniform vec4 color;
out vec4 fragColor;

void main() {
    fragColor = color;
}


-- Vertex.Textured

#version 430 core

in vec4 vertexPosition;
in vec2 vertexTexCoord;
out vec2 fragTexCoord;

void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.Textured

#version 430 core

uniform sampler2D albedoTexture;
uniform vec4 color;
in vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    fragColor = color * texture(albedoTexture, fragTexCoord);
}
