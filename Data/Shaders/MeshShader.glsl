-- Vertex

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


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 lightDirection = vec3(1.0, 0.0, 0.0);
uniform vec3 cameraPosition; // in world space
uniform int useShading = 1;

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#include "Lighting.glsl"

void main()
{
    vec4 color;
    if (useShading == 1) {
        color = blinnPhongShading(fragmentColor);
    } else {
        color = fragmentColor;
    }

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
    #else
    gatherFragment(color);
    #endif
}


-- Fragment.ClearView.Context

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform int useShading = 1;

// Camera data
uniform vec3 cameraPosition;
uniform vec3 lookingDirection;

// Focus region data
uniform vec3 sphereCenter;
uniform float sphereRadius;

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#include "Lighting.glsl"
#include "RayIntersection.glsl"

void main()
{
    vec4 color;
    if (useShading == 1) {
        color = blinnPhongShading(fragmentColor);
    } else {
        color = fragmentColor;
    }

    vec3 rayOrigin = cameraPosition;
    vec3 rayDirection = normalize(fragmentPositionWorld - cameraPosition);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);

    float t0, t1;
    vec3 intersectionPosition;
    bool intersectsSphere = raySphereIntersection(
            rayOrigin, rayDirection, sphereCenter, sphereRadius, t0, t1, intersectionPosition);
    bool fragmentInSphere = SQR(fragmentPositionWorld.x - sphereCenter.x) + SQR(fragmentPositionWorld.y - sphereCenter.y)
            + SQR(fragmentPositionWorld.z - sphereCenter.z) <= SQR(sphereRadius);

    // Add opacity multiplication factor for fragments in front of or in focus region.
    float opacityFactor = 1.0f;
    if (intersectsSphere && (fragmentInSphere || fragmentDepth < t1)) {
        // Intersect view ray with plane parallel to camera looking direction containing the sphere center.
        vec3 negativeLookingDirection = -lookingDirection; // Assuming right-handed coordinate system.
        vec3 projectedPoint;
        rayPlaneIntersection(rayOrigin, rayDirection, sphereCenter, negativeLookingDirection, projectedPoint);
        float sphereDistance = length(projectedPoint - sphereCenter) / sphereRadius;
        //if (fragmentInSphere && length(fragmentPositionWorld - cameraPosition) > length(projectedPoint - cameraPosition)) {
        //    sphereDistance = length(fragmentPositionWorld - sphereCenter) / sphereRadius;
        //}
        opacityFactor = pow(sphereDistance, 4.0); // linear increase
    }
    color.a *= opacityFactor;

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(color.rgb, 1.0);
    #else
    gatherFragment(color);
    #endif
}


-- Vertex.Plain

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

out vec3 fragmentPositionWorld;
out vec3 fragmentNormal;
out vec4 fragmentColor;

void main()
{
    fragmentNormal = vertexNormal;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment.Plain

#version 430 core

uniform vec4 color;

in vec3 fragmentPositionWorld;
in vec3 fragmentNormal;

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
    vec4 phongColor = blinnPhongShading(color);

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(phongColor.rgb, 1.0);
    #else
    gatherFragment(phongColor);
    #endif
}
