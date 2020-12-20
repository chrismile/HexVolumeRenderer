-- Vertex.Attribute

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;

out vec3 fragmentPositionWorld;
//out vec4 fragmentColor;
out float fragmentAttribute;

//#include "TransferFunction.glsl"

void main()
{
    //fragmentColor = transferFunction(vertexAttribute);
    fragmentAttribute = vertexAttribute;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
//in vec4 fragmentColor;
in float fragmentAttribute;

#include "TransferFunction.glsl"

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition; // in world space

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

void main()
{
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(fragmentColor.rgb, 1.0);
#else
    gatherFragment(fragmentColor);
#endif
}
