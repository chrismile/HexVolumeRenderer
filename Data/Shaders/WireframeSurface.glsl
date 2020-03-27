-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexBarycentricCoordinates;

out vec3 fragmentPositionWorld;
out vec4 fragmentColor;
out vec3 fragmentBarycentricCoordinates;

void main()
{
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    fragmentColor = vertexColor;
    fragmentBarycentricCoordinates = vertexBarycentricCoordinates;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in vec4 fragmentColor;
in vec3 fragmentBarycentricCoordinates;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define CONSTANT_LINE_THICKNESS

void main()
{
    /*const float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    float minCoordinate = min(fragmentBarycentricCoordinates.y, fragmentBarycentricCoordinates.z);
    #ifdef CONSTANT_LINE_THICKNESS
    float delta = fwidth(minCoordinate) * 2;
    #else
    float delta = fwidth(minCoordinate) / fragmentDepth * 2;
    #endif
    minCoordinate = smoothstep(lineOffset, lineOffset + delta, minCoordinate);*/

    /*const float THICKNESS = 2.0;
    const float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    float minCoordinate = min(fragmentBarycentricCoordinates.y, fragmentBarycentricCoordinates.z);
    float delta = fwidth(minCoordinate);
    #ifdef CONSTANT_LINE_THICKNESS
    float coverage = smoothstep(0.0, 2.0 * delta, minCoordinate);
    #else
    float lineOffset = delta / fragmentDepth;
    float coverage = smoothstep(lineOffset, lineOffset + delta, minCoordinate);
    #endif*/

    const float fragmentDepth = length(fragmentPositionWorld - cameraPosition);//gl_FragCoord.z;//length(fragmentPositionWorld - cameraPosition);

    /*vec3 dir1;
    vec3 dir2;
    if (fragmentBarycentricCoordinates.y < fragmentBarycentricCoordinates.z) {
        dir1 = vec3(1,0,0) - vec3(0,1,0);
        dir2 = fragmentBarycentricCoordinates - vec3(0,1,0);
    } else {
        dir1 = vec3(1,0,0) - vec3(0,0,1);
        dir2 = fragmentBarycentricCoordinates - vec3(0,0,1);
    }
    float cosAngle = dot(normalize(dir1), normalize(dir2));
    float sinAngle = sin(acos(cosAngle));
    float l = sinAngle * fragmentBarycentricCoordinates.y;

    float minCoordinate = l;//l < 0.1 ? 0.0 : 1.0;*/

    vec3 testCoords;
    testCoords.x = 0.0;
    testCoords.y = fragmentBarycentricCoordinates.y;
    testCoords.z = fragmentBarycentricCoordinates.z;
    float sum = testCoords.y + testCoords.z;
    testCoords.y /= sum;
    testCoords.z /= sum;

    vec3 deltas = fwidth(fragmentBarycentricCoordinates);

    const float WIREFRAME_SMOOTHING = 1.0;
    const float WIREFRAME_THICKNESS = 2.0 / fragmentDepth;
    vec3 smoothing = deltas * WIREFRAME_SMOOTHING;
    vec3 thickness = deltas * WIREFRAME_THICKNESS;
    vec3 barycentricCoordinates = smoothstep(thickness, thickness + smoothing, fragmentBarycentricCoordinates);


    //float minCoordinate = min(barycentricCoordinates.y, barycentricCoordinates.z);
    //float minCoordinate = min(barycentricCoordinates.y, barycentricCoordinates.z);
    float minCoordinate = min(barycentricCoordinates.y, barycentricCoordinates.z);
    float minCoordinateB = min(fragmentBarycentricCoordinates.y, fragmentBarycentricCoordinates.z);
    //float minCoordinateC = min(min(fragmentBarycentricCoordinates.x, fragmentBarycentricCoordinates.y), fragmentBarycentricCoordinates.z);
    float minCoordinateC = min(min(barycentricCoordinates.x, barycentricCoordinates.y), barycentricCoordinates.z);
    float coverage = minCoordinate;


    /*if (minCoordinate > 0.01 && minCoordinate < 0.1) {
        minCoordinate = 1.0;
    } else if (minCoordinate < 0.01) {
        minCoordinate = 0.5;
    } else {
        minCoordinate = 0.0;
    }*/

    //if (coverage > 0.999) {
    //    discard;
    //}

    vec3 lineColor = vec3(minCoordinate);

    vec4 color = vec4(lineColor, fragmentColor.a);

    #if defined(DIRECT_BLIT_GATHER)
    fragColor = color;
    #else
    gatherFragment(color);
    #endif
}
