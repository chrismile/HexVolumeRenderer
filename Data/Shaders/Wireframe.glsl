-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;
out vec4 fragmentColor;

void main()
{
    fragmentColor = vertexColor;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Fragment

#version 430 core

in vec4 fragmentColor;
out vec4 fragColor;

void main()
{
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    fragColor = fragmentColor;
}

