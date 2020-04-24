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
struct HexahedralCellFaceLineDensityControl {
    vec4 vertexPositions[4];
    vec4 edgeAttributes;
    vec4 edgeLodValues;
    uvec4 edgeSingularityInformationList;
};

layout (std430, binding = 6) readonly buffer HexahedralCellFaces {
    HexahedralCellFaceLineDensityControl hexahedralCellFaces[];
};

out vec3 fragmentPositionWorld;
flat out vec3 vertexPositions[4];
flat out vec4 edgeAttributes;

void main()
{
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    HexahedralCellFaceLineDensityControl hexahedralCellFace = hexahedralCellFaces[faceId];
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
        edgeAttributes[i] = hexahedralCellFace.edgeAttributes[i];
    }

    vec4 vertexPosition = hexahedralCellFace.vertexPositions[vertexId];
    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    gl_Position = mvpMatrix * vertexPosition;
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
flat in vec3 vertexPositions[4];
flat in vec4 edgeAttributes;

uniform vec3 cameraPosition;
uniform float lineWidth;

#include "PointToLineDistance.glsl"
#include "CreateAttributeTextureHeader.glsl"
#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"

/**
 * Flat shading, but adds a constant-sized halo at the outline of the surface. Assumes the following global variables
 * are given: cameraPosition, fragmentPositionWorld.
*/
void flatShadingWireframeSurfaceHaloDepth(out float fragmentDepthFrag, in float lineCoordinates) {
    float fragmentDepth = gl_FragCoord.z;
    //float fragmentDepth = convertDepthBufferValueToLinearDepth(gl_FragCoord.z);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(convertDepthBufferValueToLinearDepth(fragmentDepth) / 2.0, 0.0, 0.49);

    if (lineCoordinates >= WHITE_THRESHOLD - EPSILON) {
        fragmentDepth = convertLinearDepthToDepthBufferValue(
                convertDepthBufferValueToLinearDepth(fragmentDepth) + 0.008);
    }
    fragmentDepthFrag = fragmentDepth;
}

void gatherFragment(float importanceAttribute, float depth, vec3 lineDirection) {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    uint packedLineDirection;
    packLineDirection(lineDirection, packedLineDirection);

    LinkedListFragmentNodeAttributeTextures fragmentNode;
    fragmentNode.importanceAttribute = importanceAttribute;
    fragmentNode.depth = depth;
    fragmentNode.directionQuantized = packedLineDirection;
    fragmentNode.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        fragmentNode.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = fragmentNode;
    }
}

void main()
{
    // Compute the distance to the edges and get the minimum distance.
    float minDistance = 1e9;
    int minDistanceIndex = 0;
    float currentDistance;
    for (int i = 0; i < 4; i++) {
        currentDistance = getDistanceToLineSegment(
                fragmentPositionWorld, vertexPositions[i], vertexPositions[(i + 1) % 4]);
        if (currentDistance < minDistance) {
            minDistance = currentDistance;
            minDistanceIndex = i;
        }
    }

    vec3 lineDirection = normalize(vertexPositions[(minDistanceIndex + 1) % 4] - vertexPositions[minDistanceIndex]);
    float lineAttribute = edgeAttributes[minDistanceIndex];
    float lineCoordinates = max(minDistance / lineWidth * 2.0, 0.0);
    if (lineCoordinates > 1.0) {
        discard;
    }

    float fragmentDepth;
    flatShadingWireframeSurfaceHaloDepth(fragmentDepth, lineCoordinates);

    gatherFragment(lineAttribute, fragmentDepth, lineDirection);
}

