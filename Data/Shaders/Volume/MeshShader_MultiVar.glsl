-- Vertex.Attribute

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute0;
layout(location = 2) in float vertexAttribute1;

out vec3 fragmentPositionWorld;
out float fragmentAttribute0;
out float fragmentAttribute1;

//#include "TransferFunction.glsl"

void main()
{
    fragmentAttribute0 = vertexAttribute0;
    fragmentAttribute1 = vertexAttribute1;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in float fragmentAttribute0;
in float fragmentAttribute1;

uniform float opacity;

//#include "TransferFunction.glsl"

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition; // in world space

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

void main()
{
    //vec4 fragmentColor = transferFunction(fragmentAttribute);
    //vec4 fragmentColor = vec4(fragmentAttribute0, fragmentAttribute1, 0.0, max(fragmentAttribute0, fragmentAttribute1));
    //vec4 fragmentColor = vec4(fragmentAttribute0, fragmentAttribute1, 0.0, fragmentAttribute0);
    vec4 fragmentColor = vec4(fragmentAttribute0, fragmentAttribute1, 0.0, opacity * max(fragmentAttribute0, fragmentAttribute1));
#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(fragmentColor.rgb, 1.0);
#else
    gatherFragment(fragmentColor);
#endif
}
