-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main()
{
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "LinkedListHeader.glsl"

void main()
{
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    int index = viewportW*y + x;
    
    startOffset[index] = -1;
}