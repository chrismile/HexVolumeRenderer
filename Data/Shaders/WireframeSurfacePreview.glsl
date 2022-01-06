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
struct LodHexahedralCellFace {
    vec4 vertexPositions[4];
    vec4 edgeLodValues;
};

layout (std430, binding = 6) readonly buffer HexahedralCellFaces {
    LodHexahedralCellFace hexahedralCellFaces[];
};

out vec3 fragmentPositionWorld;
flat out vec3 vertexPositions[4];
flat out vec4 edgeColors[4];
flat out vec4 edgeLodValues;

#include "TransferFunction.glsl"

void main() {
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    LodHexahedralCellFace hexahedralCellFace = hexahedralCellFaces[faceId];
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
        edgeColors[i] = vec4(transferFunction(1.0 - hexahedralCellFace.edgeLodValues[i]).rgb, 1.0);
        edgeLodValues[i] = hexahedralCellFace.edgeLodValues[i];
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
flat in vec4 edgeLodValues;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform int showLodDifferences = 0;
uniform int maxLodValueInt;
uniform float lineWidth;
uniform float maxLod;

const float LOD_EPSILON = 0.001;

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define WIREFRAME_SURFACE_HALO_LIGHTING
#include "Lighting.glsl"
#include "PointToLineDistance.glsl"

void main() {
    const float INF = 1e9;
    int discreteSelectedLodValue = int(round(maxLod * (maxLodValueInt+1)));

    // Compute the distance to the edges and get the minimum distance.
    float minDistanceLines = INF;
    int minDistanceLinesIndex = 0;
    float currentDistance;
    for (int i = 0; i < 4; i++) {
        currentDistance = getDistanceToLineSegment(
                fragmentPositionWorld, vertexPositions[i], vertexPositions[(i + 1) % 4]);
        if (showLodDifferences == 0 && currentDistance < minDistanceLines && edgeLodValues[i] <= maxLod + LOD_EPSILON) {
            minDistanceLines = currentDistance;
            minDistanceLinesIndex = i;
        }
        int discreteFragmentLodValue = int(round(edgeLodValues[i] * (maxLodValueInt+1)));
        if (showLodDifferences == 1 && currentDistance < minDistanceLines && discreteFragmentLodValue == discreteSelectedLodValue) {
            minDistanceLines = currentDistance;
            minDistanceLinesIndex = i;
        }
    }

    vec4 baseColor = edgeColors[minDistanceLinesIndex];
    float fragmentLodValue = edgeLodValues[minDistanceLinesIndex];
    float minDistance = minDistanceLines;
    int discreteFragmentLodValue = int(round(fragmentLodValue * (maxLodValueInt+1)));

    if (showLodDifferences == 0 && fragmentLodValue > maxLod + LOD_EPSILON) {
        discard;
    }
    if (showLodDifferences == 1 && discreteFragmentLodValue != discreteSelectedLodValue) {
        discard;
    }
    float lineWidthPrime = lineWidth * (1.0 + (1.0 - fragmentLodValue) * 0.5);

    float lineCoordinates = clamp(minDistance / lineWidthPrime * 2.0, 0.0, 1.0);
    float fragmentDepth;
    //vec4 color = flatShadingWireframeSurfaceHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #if defined(LINE_RENDERING_STYLE_HALO)
    vec4 color = flatShadingWireframeSurfaceHalo(baseColor, fragmentDepth, lineCoordinates, lineWidthPrime / 2.0);
    #elif defined(LINE_RENDERING_STYLE_TRON)
    vec4 color = flatShadingWireframeSurfaceTronHalo(baseColor, fragmentDepth, lineCoordinates);
    #else //#elif defined(LINE_RENDERING_STYLE_SINGLE_COLOR)
    vec4 color = flatShadingWireframeSingleColor(baseColor, fragmentDepth, lineCoordinates);
    #endif
    //vec4 color = flatShadingWireframeSurfaceTronHalo(baseColor, fragmentDepth, lineCoordinates);

    //float dist = length(fragmentPositionWorld - cameraPosition) * 0.99 - 0.6;
    //color.rgb = mix(color.rgb, vec3(0.6), dist);

    #if defined(DIRECT_BLIT_GATHER)
    if (color.a < 0.01) {
        discard;
    }
    //gl_FragDepth = gl_FragCoord.z - 0.00001;
    gl_FragDepth = convertLinearDepthToDepthBufferValue(convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth - length(fragmentPositionWorld - cameraPosition) - 0.0001);
    fragColor = color;
    #else
    gatherFragmentCustomDepth(color, fragmentDepth);
    #endif
}
