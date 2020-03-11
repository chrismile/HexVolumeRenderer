-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 vertexColor;

out vec3 fragmentPositonWorld;
out vec3 fragmentNormal;
out vec4 fragmentColor;

void main()
{
    fragmentNormal = vertexNormal;
    fragmentColor = vertexColor;
    fragmentPositonWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

in vec3 fragmentPositonWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 lightDirection = vec3(1.0, 0.0, 0.0);
uniform vec3 cameraPosition; // in world space

#include "Lighting.glsl"

void main()
{
    vec4 colorPhong = blinnPhongShading(fragmentColor);

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(colorPhong.rgb, 1.0);
    #else
    gatherFragment(colorPhong);
    #endif
}


-- Fragment.ClearView.Context

#version 430 core

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

in vec3 fragmentPositonWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

// Camera data
uniform vec3 cameraPosition;
uniform vec3 lookingDirection;

// Focus region data
uniform vec3 sphereCenter;
uniform float sphereRadius;

#include "Lighting.glsl"

#define SQR(x) ((x)*(x))

/**
 * Implementation of ray-sphere intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter1.htm
 */
bool raySphereIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 sphereCenter, float sphereRadius,
        out float t0, out float t1, out vec3 intersectionPosition)
{
    float A = SQR(rayDirection.x) + SQR(rayDirection.y) + SQR(rayDirection.z);
    float B = 2.0 * (rayDirection.x * (rayOrigin.x - sphereCenter.x) + rayDirection.y * (rayOrigin.y - sphereCenter.y)
            + rayDirection.z * (rayOrigin.z - sphereCenter.z));
    float C = SQR(rayOrigin.x - sphereCenter.x) + SQR(rayOrigin.y - sphereCenter.y) + SQR(rayOrigin.z - sphereCenter.z)
            - SQR(sphereRadius);

    float discriminant = SQR(B) - 4.0*A*C;
    if (discriminant < 0.0) {
        return false; // No intersection
    }

    float discriminantSqrt = sqrt(discriminant);
    t0 = (-B - discriminantSqrt) / (2.0 * A);
    t1 = (-B + discriminantSqrt) / (2.0 * A);

    intersectionPosition = rayOrigin + t0 * rayDirection;
    // Intersection(s) behind the ray origin?
    /*if (t0 >= 0.0) {
        return true;
    } else if (t1 >= 0) {
        intersectionPosition = rayOrigin + t1 * rayDirection;
        return true;
    }
    return false;*/

    return true;
}

bool rayPlaneIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 planePoint, vec3 planeNormal, out vec3 intersectionPosition) {
    float ln = dot(planeNormal, rayDirection);
    if (abs(ln) < 1e-4) {
        // Plane and ray are (almost) parallel.
        return false;
    } else {
        float pos = dot(planeNormal, rayOrigin) + dot(planeNormal, planePoint);
        float t = -pos / ln;
        intersectionPosition = rayOrigin + t * rayDirection;
        return true;
    }
}

void main()
{
    vec4 colorPhong = blinnPhongShading(fragmentColor);

    vec3 rayOrigin = cameraPosition;
    vec3 rayDirection = normalize(fragmentPositonWorld - cameraPosition);
    float fragmentDepth = length(fragmentPositonWorld - cameraPosition);

    float t0, t1;
    vec3 intersectionPosition;
    bool intersectsSphere = raySphereIntersection(
            rayOrigin, rayDirection, sphereCenter, sphereRadius, t0, t1, intersectionPosition);
    bool fragmentInSphere = SQR(fragmentPositonWorld.x - sphereCenter.x) + SQR(fragmentPositonWorld.y - sphereCenter.y)
            + SQR(fragmentPositonWorld.z - sphereCenter.z) <= SQR(sphereRadius);
    float intersectionDepth = length(intersectionPosition - cameraPosition);

    // Add opacity multiplication factor for fragments in front of or in focus region.
    float opacityFactor = 1.0f;
    if (intersectsSphere && (fragmentInSphere || fragmentDepth < intersectionDepth)) {
        // Intersect view ray with plane parallel to camera looking direction containing the sphere center.
        vec3 negativeLookingDirection = -lookingDirection; // Assuming right-handed coordinate system.
        vec3 projectedPoint;
        rayPlaneIntersection(rayOrigin, rayDirection, sphereCenter, negativeLookingDirection, projectedPoint);
        opacityFactor = length(projectedPoint - sphereCenter) / sphereRadius; // linear increase
    }
    colorPhong.a *= opacityFactor;

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(colorPhong.rgb, 1.0);
    #else
    gatherFragment(colorPhong);
    #endif
}


-- Vertex.Plain

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

out vec3 fragmentPositonWorld;
out vec3 fragmentNormal;
out vec4 fragmentColor;

void main()
{
    fragmentNormal = vertexNormal;
    fragmentPositonWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment.Plain

#version 430 core

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

uniform vec4 color;

in vec3 fragmentPositonWorld;
in vec3 fragmentNormal;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 lightDirection = vec3(1.0, 0.0, 0.0);
uniform vec3 cameraPosition; // in world space

#include "Lighting.glsl"

void main()
{
    vec4 colorPhong = blinnPhongShading(color);

    #if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(colorPhong.rgb, 1.0);
    #else
    gatherFragment(colorPhong);
    #endif
}
