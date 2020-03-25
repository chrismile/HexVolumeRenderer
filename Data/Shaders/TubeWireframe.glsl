-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec3 vertexTangent;
layout(location = 3) in vec4 vertexColor;

out vec3 fragmentPositionWorld;
out vec3 fragmentNormal;
out vec3 fragmentTangent;
out vec4 fragmentColor;

void main()
{
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    fragmentNormal = vertexNormal;
    fragmentTangent = vertexTangent;
    fragmentColor = vertexColor;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec3 fragmentTangent;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;

/*
uniform float minCriterionValue = 0.0;
uniform float maxCriterionValue = 1.0;

// Transfer function color lookup table
uniform sampler1D transferFunctionTexture;

vec4 transferFunction(float attr) {
    // Transfer to range [0,1]
    float posFloat = clamp((attr - minCriterionValue) / (maxCriterionValue - minCriterionValue), 0.0, 1.0);
    // Look up the color value
    return texture(transferFunctionTexture, posFloat);
}
*/

#define TUBE_HALO_LIGHTING
#include "Lighting.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

void main()
{
    //vec4 color = transferFunction(fragmentAttribute);
    vec4 color = fragmentColor;
    //color = blinnPhongShadingHalo(color);
    color = flatShadingHalo(color);

    #if defined(DIRECT_BLIT_GATHER)
    fragColor = color;
    #else
    gatherFragment(color);
    #endif
}


-- Fragment.ClearView.Focus

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec3 fragmentTangent;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

/*
uniform float minCriterionValue = 0.0;
uniform float maxCriterionValue = 1.0;

// Transfer function color lookup table
uniform sampler1D transferFunctionTexture;

vec4 transferFunction(float attr) {
    // Transfer to range [0,1]
    float posFloat = clamp((attr - minCriterionValue) / (maxCriterionValue - minCriterionValue), 0.0, 1.0);
    // Look up the color value
    return texture(transferFunctionTexture, posFloat);
}
*/

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define TUBE_HALO_LIGHTING
#include "Lighting.glsl"

void main()
{
    //vec4 color = transferFunction(fragmentAttribute);
    vec4 color = fragmentColor;
    float fragmentDepth;
    //color = blinnPhongShadingHalo(color);
    color = flatShadingHalo(color, fragmentDepth);
    color.a *= getClearViewFocusFragmentOpacityFactor();

    #if defined(DIRECT_BLIT_GATHER)
    fragColor = color;
    #else
    gatherFragmentCustomDepth(color, fragmentDepth);
    #endif
}

