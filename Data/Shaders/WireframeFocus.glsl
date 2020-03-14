-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;

out VertexData
{
    vec3 linePosition;
    vec4 lineColor;
};

void main()
{
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineColor = vertexColor;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float lineWidth;

out vec3 fragmentPositionWorld;
out float quadCoords; // Between -1 and 1
out vec4 fragmentColor;

in VertexData
{
    vec3 linePosition;
    vec4 lineColor;
} v_in[];

void main()
{
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    vec4 lineColor0 = v_in[0].lineColor;
    vec4 lineColor1 = v_in[1].lineColor;
    vec4 linePositionNDC0 = pMatrix * vMatrix * vec4(linePosition0, 1.0);
    vec4 linePositionNDC1 = pMatrix * vMatrix * vec4(linePosition1, 1.0);
    linePositionNDC0.xyz /= linePositionNDC0.w;
    linePositionNDC1.xyz /= linePositionNDC1.w;
    vec2 linePositionScreen0 = linePositionNDC0.xy;
    vec2 linePositionScreen1 = linePositionNDC0.xy;
    vec2 dir = linePositionScreen1 - linePositionScreen0;
    vec2 normalDir = vec2(-dir.y, dir.x);

    vec3 right = normalize(v_in[1].linePosition - v_in[0].linePosition);
    vec3 quadNormal0 = normalize(cameraPosition - linePosition0);
    vec3 quadNormal1 = normalize(cameraPosition - linePosition1);
    vec3 vertexPosition;

    vec3 up0 = normalize(cross(quadNormal0, right));
    vec3 up1 = normalize(cross(quadNormal1, right));

    vertexPosition = linePosition0 - (lineWidth / 2.0) * up0;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor0;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - (lineWidth / 2.0) * up1;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor1;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + (lineWidth / 2.0) * up0;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor0;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + (lineWidth / 2.0) * up1;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor1;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

in vec3 fragmentPositionWorld;
in vec4 fragmentColor;
in float quadCoords; // Between -1 and 1

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

// Camera data
uniform vec3 cameraPosition;
uniform vec3 lookingDirection;

// Focus region data
uniform vec3 sphereCenter;
uniform float sphereRadius;

#include "RayIntersection.glsl"

void main()
{
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    float absCoords = abs(quadCoords);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, absCoords);
    vec4 color = vec4(mix(fragmentColor.rgb, vec3(1.0, 1.0, 1.0),
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, absCoords)), fragmentColor.a * coverage);

    vec3 rayOrigin = cameraPosition;
    vec3 rayDirection = normalize(fragmentPositionWorld - cameraPosition);

    float t0, t1;
    vec3 intersectionPosition;
    bool intersectsSphere = raySphereIntersection(
    rayOrigin, rayDirection, sphereCenter, sphereRadius, t0, t1, intersectionPosition);
    bool fragmentInSphere = SQR(fragmentPositionWorld.x - sphereCenter.x) + SQR(fragmentPositionWorld.y - sphereCenter.y)
    + SQR(fragmentPositionWorld.z - sphereCenter.z) <= SQR(sphereRadius);

    // Add opacity multiplication factor for fragments in front of or in focus region.
    float opacityFactor = 0.0f;
    if (intersectsSphere && fragmentInSphere) {
        // Intersect view ray with plane parallel to camera looking direction containing the sphere center.
        /*vec3 negativeLookingDirection = -lookingDirection; // Assuming right-handed coordinate system.
        vec3 projectedPoint;
        rayPlaneIntersection(rayOrigin, rayDirection, sphereCenter, negativeLookingDirection, projectedPoint);
        float sphereDistance = length(projectedPoint - sphereCenter) / sphereRadius;
        opacityFactor = 1.0 - sphereDistance*sphereDistance; // linear decrease
        */
        float sphereDistance = length(fragmentPositionWorld - sphereCenter) / sphereRadius;
        opacityFactor = 1.0 - pow(sphereDistance, 10.0); // linear decrease
    }
    color.a *= opacityFactor;

    #if defined(DIRECT_BLIT_GATHER)
    fragColor = color;
    #else
    gatherFragment(color);
    #endif
}

