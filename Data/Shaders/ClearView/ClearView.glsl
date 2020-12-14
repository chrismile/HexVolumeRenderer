// Camera data
uniform vec3 cameraPosition;
uniform vec3 lookingDirection;

// Focus region data
uniform vec3 sphereCenter;
uniform float sphereRadius;

#define USE_CLEAR_VIEW

#include "RayIntersection.glsl"

float getClearViewContextFragmentOpacityFactor() {
    vec3 rayOrigin = cameraPosition;
    vec3 rayDirection = normalize(fragmentPositionWorld - cameraPosition);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);

    float t0, t1;
    vec3 intersectionPosition;
    bool intersectsSphere = raySphereIntersection(
            rayOrigin, rayDirection, sphereCenter, sphereRadius, t0, t1, intersectionPosition);
    bool fragmentInSphere = SQR(fragmentPositionWorld.x - sphereCenter.x)
            + SQR(fragmentPositionWorld.y - sphereCenter.y)
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

    return opacityFactor;
}

float getClearViewFocusFragmentOpacityFactor() {
    vec3 rayOrigin = cameraPosition;
    vec3 rayDirection = normalize(fragmentPositionWorld - cameraPosition);

    float t0, t1;
    vec3 intersectionPosition;
    bool intersectsSphere = raySphereIntersection(
            rayOrigin, rayDirection, sphereCenter, sphereRadius, t0, t1, intersectionPosition);
    bool fragmentInSphere = SQR(fragmentPositionWorld.x - sphereCenter.x)
            + SQR(fragmentPositionWorld.y - sphereCenter.y)
            + SQR(fragmentPositionWorld.z - sphereCenter.z) <= SQR(sphereRadius);

    // Add opacity multiplication factor for fragments in front of or in focus region.
    float opacityFactor = 0.0f;
    if (intersectsSphere && fragmentInSphere) {
        float sphereDistance = length(fragmentPositionWorld - sphereCenter) / sphereRadius;
        opacityFactor = 1.0 - pow(sphereDistance, 10.0); // linear decrease
    }

    return opacityFactor;
}