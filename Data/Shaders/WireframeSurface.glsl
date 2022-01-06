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
    vec4 vertexPositions[4]; ///< Vertex positions
    vec4 lineColors[4]; ///< Colors of the edges
};

layout (std430, binding = 6) readonly buffer HexahedralCellFaces {
    HexahedralCellFace hexahedralCellFaces[];
};

out vec3 fragmentPositionWorld;
flat out vec3 vertexPositions[4];
flat out vec4 lineColors[4];

void main() {
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    HexahedralCellFace hexahedralCellFace = hexahedralCellFaces[faceId];
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
        lineColors[i] = hexahedralCellFace.lineColors[i];
    }

    vec4 vertexPosition = hexahedralCellFace.vertexPositions[vertexId];
    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    gl_Position = mvpMatrix * vertexPosition;
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
flat in vec3 vertexPositions[4];
flat in vec4 lineColors[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define WIREFRAME_SURFACE_HALO_LIGHTING
#include "Lighting.glsl"
#include "PointToLineDistance.glsl"
#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"

void main() {
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

    vec4 lineBaseColor = lineColors[minDistanceIndex];
    float lineCoordinates = clamp(minDistance / lineWidth * 2.0, 0.0, 1.0);
    float fragmentDepth;
    //vec4 color = flatShadingWireframeSurfaceHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #if defined(LINE_RENDERING_STYLE_HALO)
    vec4 color = flatShadingWireframeSurfaceHalo(lineBaseColor, fragmentDepth, lineCoordinates, lineWidth / 2.0);
    #elif defined(LINE_RENDERING_STYLE_TRON)
    vec4 color = flatShadingWireframeSurfaceTronHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #else //#elif defined(LINE_RENDERING_STYLE_SINGLE_COLOR)
    vec4 color = flatShadingWireframeSingleColor(lineBaseColor, fragmentDepth, lineCoordinates);
    #endif
    //vec4 color = flatShadingWireframeSurfaceTronHalo(lineBaseColor, fragmentDepth, lineCoordinates);

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


-- Fragment.ClearView.Focus

#version 430 core

in vec3 fragmentPositionWorld;
flat in vec3 vertexPositions[4];
flat in vec4 lineColors[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform float lineWidth;

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define WIREFRAME_SURFACE_HALO_LIGHTING
#include "Lighting.glsl"
#include "PointToLineDistance.glsl"

void main() {
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

    vec4 lineBaseColor = lineColors[minDistanceIndex];
    float lineCoordinates = max(minDistance / lineWidth * 2.0, 0.0);
    if (lineCoordinates > 1.0) {
        discard;
    }
    float fragmentDepth;
    #if defined(LINE_RENDERING_STYLE_HALO)
    vec4 color = flatShadingWireframeSurfaceHalo_DepthCue(lineBaseColor, fragmentDepth, lineCoordinates);
    #elif defined(LINE_RENDERING_STYLE_TRON)
    vec4 color = flatShadingWireframeSurfaceTronHalo(lineBaseColor, fragmentDepth, lineCoordinates);
    #else //#elif defined(LINE_RENDERING_STYLE_SINGLE_COLOR)
    vec4 color = flatShadingWireframeSingleColor(lineBaseColor, fragmentDepth, lineCoordinates);
    #endif
    color.a *= getClearViewFocusFragmentOpacityFactor();

    #if defined(DIRECT_BLIT_GATHER)
    if (color.a < 0.01) {
        discard;
    }
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    //gl_FragDepth = gl_FragCoord.z + fragmentDepth - length(fragmentPositionWorld - cameraPosition);
    fragColor = color;
    #else
    fragmentDepth += 0.00001;
    //fragmentDepth = fragmentDepth + 0.00004;
    gatherFragmentCustomDepth(color, fragmentDepth);
    #endif
}
