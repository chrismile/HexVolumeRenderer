-- Vertex

#version 430 core

in vec4 vertexPosition;
in vec2 vertexTextureCoordinates;
out vec2 fragmentTextureCoordinates;

void main()
{
    fragmentTextureCoordinates = vertexTextureCoordinates;
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment

#version 430 core

uniform sampler2D textureImage;
in vec2 fragmentTextureCoordinates;
out vec4 fragColor;

// Values for perfect weights and offsets that utilize bilinear texture filtering
// are from http://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/
uniform float offsets[3] = float[](0.0, 1.3846153846, 3.2307692308);
uniform float weights[3] = float[](0.2270270270, 0.3162162162, 0.0702702703);
uniform bool horizontalBlur;
uniform vec2 textureSize;

void main()
{
    vec4 color = texture(textureImage, fragmentTextureCoordinates) * weights[0];
    for (int i = 1; i < 3; i++) {
        vec2 offset;
        if (horizontalBlur) {
            offset = vec2(offsets[i] / textureSize.x, 0.0) ;
        } else {
            offset = vec2(0.0, offsets[i] / textureSize.y);
        }
        color += texture(textureImage, fragmentTextureCoordinates + offset) * weights[i];
        color += texture(textureImage, fragmentTextureCoordinates - offset) * weights[i];
    }
    fragColor = color;
}
