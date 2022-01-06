-- Vertex

#version 430 core

struct SphereInstancingData {
    vec3 position;
    float padding;
    vec4 color;
};

layout (std430, binding = 6) readonly buffer SphereInstancingDataBuffer {
    SphereInstancingData sphereInstancingDataBuffer[];
};

layout(location = 0) in vec3 vertexPosition;
//layout(location = 1) in vec3 vertexNormal;

out vec3 fragmentPositionWorld;
//out vec3 fragmentNormal;
out vec4 fragmentColor;

void main() {
    SphereInstancingData sphereInstancingData = sphereInstancingDataBuffer[gl_InstanceID];
    vec3 translatedPosition = vertexPosition + sphereInstancingData.position;
    //fragmentNormal = vertexNormal;
    fragmentColor = sphereInstancingData.color;
    fragmentPositionWorld = (mMatrix * vec4(translatedPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(translatedPosition, 1.0);
}

-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
//in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

void main() {
    vec4 color = fragmentColor;
    color.a *= getClearViewFocusFragmentOpacityFactor();

#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
#else
    gatherFragment(color);
#endif
}
