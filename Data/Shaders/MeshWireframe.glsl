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
struct HexahedralCellFaceUnified {
    vec4 vertexPositions[4]; ///< Vertex positions
    float vertexAttributes[4]; ///< Vertex attributes
    vec4 lineColors[4]; ///< Colors of the edges
};

layout (std430, binding = 6) readonly buffer HexahedralCellFaces {
    HexahedralCellFaceUnified hexahedralCellFaces[];
};

out vec3 fragmentPositionWorld;
out vec4 fragmentColor;
flat out vec3 vertexPositions[4];
flat out vec4 lineColors[4];

#include "TransferFunction.glsl"

void main()
{
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    HexahedralCellFaceUnified hexahedralCellFace = hexahedralCellFaces[faceId];
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
        lineColors[i] = hexahedralCellFace.lineColors[i];
    }

    vec4 vertexPosition = hexahedralCellFace.vertexPositions[vertexId];
    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    fragmentColor = transferFunction(hexahedralCellFace.vertexAttributes[vertexId]);
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.ClearView

#version 430 core

in vec3 fragmentPositionWorld;
in vec4 fragmentColor;
flat in vec3 vertexPositions[4];
flat in vec4 lineColors[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform float lineWidth;

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#define GATHER_NO_DISCARD // as we gather two fragments in one shader
#include OIT_GATHER_HEADER
#endif

#define WIREFRAME_SURFACE_HALO_LIGHTING
#include "Lighting.glsl"
#include "PointToLineDistance.glsl"

void main()
{
    // Add the focus fragment.
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

    vec4 lineBaseColor = lineColors[minDistanceIndex];
    float lineCoordinates = max(minDistance / lineWidth * 2.0, 0.0);
    if (lineCoordinates <= 1.0) {
        // Focus wireframe
        float fragmentDepth;
        #if defined(LINE_RENDERING_STYLE_HALO)
        vec4 color = flatShadingWireframeSurfaceHalo_DepthCue(lineBaseColor, fragmentDepth, lineCoordinates);
        #elif defined(LINE_RENDERING_STYLE_TRON)
        vec4 color = flatShadingWireframeSurfaceTronHalo(lineBaseColor, fragmentDepth, lineCoordinates);
        #else //#elif defined(LINE_RENDERING_STYLE_SINGLE_COLOR)
        vec4 color = flatShadingWireframeSingleColor(lineBaseColor, fragmentDepth, lineCoordinates);
        #endif
        color.a *= getClearViewFocusFragmentOpacityFactor();

        fragmentDepth += 0.00001;
        gatherFragmentCustomDepth(color, fragmentDepth);
    }

    // Add the context fragment.
    vec4 colorContext = fragmentColor;
    bool isSingularEdge = lineBaseColor.r > 0.8;
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    float expFactor = exp(-3.5 * fragmentDepth);
    float boostFactor = clamp(2.0 * expFactor + 1.0, 1.0, 1.5);
    const float EPSILON = 1e-5;
    float lineCoordinatesContext = max(minDistance / lineWidth * 2.0 / (isSingularEdge ? 1.0 : max(expFactor, EPSILON)) * 1.5, 0.0);
    if (lineCoordinatesContext <= 1.0) {
        if (isSingularEdge) {
            colorContext.rgb = vec3(1.0, 0.0, 0.0);
            //colorContext.a *= 0.5;
            colorContext.a = max(colorContext.a * 0.5, 0.2);
        } else {
            //float boostFactor = clamp(0.2 / fragmentDepth + 1.0, 1.0, 1.5);
            ///colorContext.rgb = vec3(0.0, 0.7, 1.0);
            //colorContext.a *= 0.1;
            colorContext.a = clamp(colorContext.a * boostFactor, 0.0, 1.0);
        }
    }
    colorContext.a *= getClearViewContextFragmentOpacityFactor();
    gatherFragment(colorContext);
}
