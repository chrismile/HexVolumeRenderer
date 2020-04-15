-- Vertex

#version 430 core

/**
 * vertex 1     edge 1    vertex 2
 *          | - - - - - |
 *          | \         |
 *          |   \       |
 *   edge 0 |     \     | edge 2
 *          |       \   |
 *          |         \ |
 *          | - - - - - |
 * vertex 0     edge 3    vertex 3
*/
struct HexahedralCellFace {
    vec4 vertexPositions[4];
    vec4 edgeColors[4];
    vec4 cornerColors[4];
    float edgeLodValues[4];
    float cornerLodValues[4];
};

layout (std430, binding = 6) readonly buffer HexahedralCellFaces {
    HexahedralCellFace hexahedralCellFaces[];
};

out vec3 fragmentPositionWorld;
flat out vec3 vertexPositions[4];
flat out vec4 edgeColors[4];
flat out vec4 cornerColors[4];
flat out float edgeLodValues[4];
flat out float cornerLodValues[4];

void main()
{
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    HexahedralCellFace hexahedralCellFace = hexahedralCellFaces[faceId];
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
        edgeColors[i] = hexahedralCellFace.edgeColors[i];
        cornerColors[i] = hexahedralCellFace.cornerColors[i];
        edgeLodValues[i] = hexahedralCellFace.edgeLodValues[i];
        cornerLodValues[i] = hexahedralCellFace.cornerLodValues[i];
    }

    vec4 vertexPosition = hexahedralCellFace.vertexPositions[vertexId];
    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    gl_Position = mvpMatrix * vertexPosition;
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
flat in vec3 vertexPositions[4];
flat in vec4 edgeColors[4];
flat in vec4 cornerColors[4];
flat in float edgeLodValues[4];
flat in float cornerLodValues[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform float maxLod;

const float LOD_EPSILON = 0.001;

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define WIREFRAME_SURFACE_HALO_LIGHTING
#include "Lighting.glsl"
#include "PointToLineDistance.glsl"

void main()
{
    const float INF = 1e9;

    // Compute the distance to the edges and get the minimum distance.
    float minDistanceLines = INF;
    int minDistanceLinesIndex = 0;
    float currentDistance;
    for (int i = 0; i < 4; i++) {
        currentDistance = distanceToLineSegment(
                fragmentPositionWorld, vertexPositions[i], vertexPositions[(i + 1) % 4]);
        if (currentDistance < minDistanceLines && edgeLodValues[i] <= maxLod + LOD_EPSILON) {
            minDistanceLines = currentDistance;
            minDistanceLinesIndex = i;
        }
    }

    // Compute the distance to the corners.
    float minDistancePoints = INF;
    int minDistancePointsIndex = 0;
    for (int i = 0; i < 4; i++) {
        currentDistance = length(fragmentPositionWorld - vertexPositions[i]);
        if (currentDistance < minDistancePoints && cornerLodValues[i] <= maxLod + LOD_EPSILON) {
            minDistancePoints = currentDistance;
            minDistancePointsIndex = i;
        }
    }

    vec4 baseColor;
    float fragmentLodValue;
    float minDistance;
    if (minDistanceLines < minDistancePoints) {
        baseColor = edgeColors[minDistanceLinesIndex];
        fragmentLodValue = edgeLodValues[minDistanceLinesIndex];
        minDistance = minDistanceLines;
    } else {
        baseColor = cornerColors[minDistancePointsIndex];
        fragmentLodValue = cornerLodValues[minDistancePointsIndex];
        minDistance = minDistancePoints;
    }

    if (fragmentLodValue > maxLod + LOD_EPSILON) {
        discard;
    }
    float lineWidthPrime = lineWidth * (1.3 * (1.0 - fragmentLodValue) + 0.2);

    float lineCoordinates = clamp(minDistance / lineWidthPrime * 2.0, 0.0, 1.0);
    float fragmentDepth;
    //vec4 color = flatShadingWireframeSurfaceHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #if defined(LINE_RENDERING_STYLE_HALO)
    vec4 color = flatShadingWireframeSurfaceHalo(baseColor, fragmentDepth, lineCoordinates);
    #elif defined(LINE_RENDERING_STYLE_TRON)
    vec4 color = flatShadingWireframeSurfaceTronHalo(baseColor, fragmentDepth, lineCoordinates);
    #else //#elif defined(LINE_RENDERING_STYLE_SINGLE_COLOR)
    vec4 color = flatShadingWireframeSingleColor(baseColor, fragmentDepth, lineCoordinates);
    #endif
    //vec4 color = flatShadingWireframeSurfaceTronHalo(baseColor, fragmentDepth, lineCoordinates);

    #if defined(DIRECT_BLIT_GATHER)
    if (color.a < 0.01) {
        discard;
    }
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    //gl_FragDepth = gl_FragCoord.z + fragmentDepth - length(fragmentPositionWorld - cameraPosition);
    fragColor = color;
    #else
    gatherFragmentCustomDepth(color, fragmentDepth);
    #endif
}
