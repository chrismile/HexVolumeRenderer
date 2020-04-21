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

#define WIREFRAME_SURFACE_HALO_LIGHTING
#include "Lighting.glsl"
#include "PointToLineDistance.glsl"

#include "CreateAttributeTextureHeader.glsl"

void gatherFragment(float importanceAttribute, float depth, vec3 lineDirection) {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNodeAttributeTextures fragmentNode;

    fragmentNode.importanceAttribute = importanceAttribute;

    //fragmentNode.depth = convertDepthBufferValueToLinearDepth(gl_FragCoord.z);
    //fragmentNode.depth = length(fragmentPositionWorld - cameraPosition);
    fragmentNode.depth = depth;

    uint packedLineDirection;
    packLineDirection(lineDirection, packedLineDirection);
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
        currentDistance = distanceToLineSegment(
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

    // TODO: Remove color.
    vec4 lineBaseColor = vec4(0.0);
    float fragmentDepth;
    #if defined(LINE_RENDERING_STYLE_HALO)
    vec4 color = flatShadingWireframeSurfaceHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #elif defined(LINE_RENDERING_STYLE_TRON)
    vec4 color = flatShadingWireframeSurfaceTronHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #else //#elif defined(LINE_RENDERING_STYLE_SINGLE_COLOR)
    vec4 color = flatShadingWireframeSingleColor(lineBaseColor, fragmentDepth, lineCoordinates);
    #endif

    gatherFragment(lineAttribute, fragmentDepth, lineDirection);
}

