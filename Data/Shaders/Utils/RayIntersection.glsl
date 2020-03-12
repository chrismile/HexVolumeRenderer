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
        float pos = dot(planeNormal, rayOrigin) - dot(planeNormal, planePoint);
        float t = -pos / ln;
        intersectionPosition = rayOrigin + t * rayDirection;
        return true;
    }
}
