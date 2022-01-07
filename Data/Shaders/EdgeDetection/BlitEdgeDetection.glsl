-- Vertex

#version 430 core

in vec4 vertexPosition;
out vec2 fragTexCoord;

void main() {
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment

#version 430 core

uniform sampler2D textureIn;
uniform vec4 clearColor;
uniform ivec2 viewportSize;

in vec4 gl_FragCoord;
out vec4 fragColor;

void main() {
    vec4 color = texture(textureIn, gl_FragCoord.xy / vec2(viewportSize - ivec2(1, 1)));
    fragColor = vec4(vec3(1.0) - clearColor.rgb, color.r * 0.5);
}
