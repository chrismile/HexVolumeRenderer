-- Vertex.Color

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 vertexColor;

out vec3 fragmentPositionWorld;
out vec3 fragmentNormal;
out vec4 fragmentColor;

void main()
{
    fragmentNormal = vertexNormal;
    fragmentColor = vertexColor;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Vertex.Attribute

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in float vertexAttribute;

out vec3 fragmentPositionWorld;
out vec3 fragmentNormal;
out vec4 fragmentColor;

#include "TransferFunction.glsl"

void main()
{
    fragmentNormal = vertexNormal;
    fragmentColor = transferFunction(vertexAttribute);
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment.FrontFace

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 lightDirection = vec3(1.0, 0.0, 0.0);
uniform vec3 cameraPosition; // in world space

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#include "Lighting.glsl"

void main()
{
    vec4 color = fragmentColor;

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
    #else
    gatherFragmentVolumeFrontFace(color);
    #endif
}


-- Fragment.BackFace

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 lightDirection = vec3(1.0, 0.0, 0.0);
uniform vec3 cameraPosition; // in world space

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#include "Lighting.glsl"

void main()
{
    vec4 color = fragmentColor;

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
    #else
    gatherFragmentVolumeBackFace(color);
    #endif
}


-- Fragment.ClearView.Context.FrontFace

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#include "Lighting.glsl"

void main()
{
    vec4 color = fragmentColor;
    color.a *= getClearViewContextFragmentOpacityFactor();

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
    #else
    gatherFragmentVolumeFrontFace(color);
    #endif
}


-- Fragment.ClearView.Context.BackFace

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#include "Lighting.glsl"

void main()
{
    vec4 color = fragmentColor;
    color.a *= getClearViewContextFragmentOpacityFactor();

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
    #else
    gatherFragmentVolumeBackFace(color);
    #endif
}

