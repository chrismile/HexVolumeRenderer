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
    vec4 vertexPositions[4];
    float vertexAttributes[4];
    float edgeAttributes[4];
    float edgeLodValues[4];
    uint edgeSingularityInformationList[4];
};

layout (std430, binding = 6) readonly buffer HexahedralCellFaces {
    HexahedralCellFaceUnified hexahedralCellFaces[];
};

out vec3 fragmentPositionWorld;
out vec4 fragmentColor;
flat out vec3 vertexPositions[4];
flat out vec4 lineColors[4];
flat out float edgeLodValues[4];
flat out uint edgeSingularityInformationList[4];

#include "TransferFunction.glsl"

// Color lookup table for singular edges.
uniform sampler2D singularEdgeColorLookupTexture;

vec4 lookupSingularEdgeColor(uint edgeSingularityInformation) {
    // x position: The valence minus one. y position: 0 if it is an interior edge, 1 if it is a boundary edge.
    ivec2 samplingPosition = ivec2(int(edgeSingularityInformation >> 2u) - 1, int(edgeSingularityInformation >> 1u) & 1);
    return texelFetch(singularEdgeColorLookupTexture, samplingPosition, 0);
}

void main()
{
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    HexahedralCellFaceUnified hexahedralCellFace = hexahedralCellFaces[faceId];

    // Copy the edge data.
    for (int i = 0; i < 4; i++) {
        uint edgeSingularityInformation = hexahedralCellFace.edgeSingularityInformationList[i];
        vec4 lineColor;
        if ((edgeSingularityInformation & 1u) == 1u) {
            // Singular edge.
            lineColor = lookupSingularEdgeColor(edgeSingularityInformation);
        } else {
            // Regular edge.
            lineColor = vec4(transferFunction(hexahedralCellFace.edgeAttributes[i]).rgb, 1.0);
        }
        lineColors[i] = lineColor;
        edgeLodValues[i] = hexahedralCellFace.edgeLodValues[i];
        edgeSingularityInformationList[i] = edgeSingularityInformation;
    }

    // Copy the face data.
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
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
flat in float edgeLodValues[4];
flat in uint edgeSingularityInformationList[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform float lineWidth;
uniform float maxLodValue;

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

    float lodLineValue = edgeLodValues[minDistanceIndex];
    float discreteLodValue = lodLineValue * maxLodValue;
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);

    const float CUTOFF_EPSILON = 0.05;

    vec4 lineBaseColor = vec4(mix(lineColors[minDistanceIndex].rgb, vec3(0.0), 0.4), lineColors[minDistanceIndex].a);
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
        float expOpacityFactorFocus = exp(-4.0 * (fragmentDepth - 0.2) * discreteLodValue / lineWidth * 0.001) + 0.4;
        color.a *= clamp(expOpacityFactorFocus, 0.0, 1.0);
        color.a *= getClearViewFocusFragmentOpacityFactor();

        fragmentDepth += 0.00001;
        gatherFragmentCustomDepth(color, fragmentDepth);
    }

    float distanceToFocusRing = length(fragmentPositionWorld - sphereCenter) - sphereRadius;
    float expFactor = exp(-3.0 * min(fragmentDepth, max(distanceToFocusRing * 6.0, 0.01)) * discreteLodValue);
    float expFactorOpacity = exp(-6.0 * min(fragmentDepth, max(distanceToFocusRing * 6.0, 0.01)) * discreteLodValue);
    float boostFactor = clamp(2.5 * expFactorOpacity, 0.0, 2.0);

    // Add the context fragment.
    vec4 colorContext = fragmentColor;
    bool isSingularEdge = (edgeSingularityInformationList[minDistanceIndex] & 1u) == 1u;
    const float EPSILON = 1e-5;
    float lineWidthFactor;
    float lineWidthPrime = lineWidth * (
    #if defined(HIGHLIGHT_SINGULAR_EDGES)
        isSingularEdge ? 1.0 :
    #endif
        max(expFactor, EPSILON) / log2(discreteLodValue/4.0+1.75)) / 1.5;
    if (expFactor < 0.2) {
        lineWidthPrime = mix(lineWidthPrime, 1e-8, smoothstep(0.2, 0.2 + CUTOFF_EPSILON, expFactor));
    }
    float lineCoordinatesContext = max(minDistance / lineWidthPrime * 2.0, 0.0);
    #ifdef HIGHLIGHT_EDGES
    if (lineCoordinatesContext <= 1.0) {
        #ifdef HIGHLIGHT_LOW_LOD_EDGES
        if (discreteLodValue <= 1.001) {
            colorContext.a = max(colorContext.a, 0.5);
        } else if (true) {
            colorContext.a = max(colorContext.a, 0.5 / discreteLodValue);
        }
        #endif

        colorContext.rgb = lineBaseColor.rgb;
        if (isSingularEdge) {
            //colorContext.a *= 0.5;
            #if defined(TOO_MUCH_SINGULAR_EDGE_MODE) || !defined(HIGHLIGHT_SINGULAR_EDGES)
            colorContext.a = clamp(colorContext.a * max(boostFactor, 1.0), 0.0, 1.0);
            #else
            colorContext.a = max(colorContext.a * max(boostFactor, 1.0), 0.5);
            #endif
        } else {
            colorContext.rgb = mix(colorContext.rgb, vec3(1.0, 1.0, 1.0), 0.3);
            if (boostFactor >= 1.0) {
                colorContext.a = clamp(colorContext.a * boostFactor, 0.0, 1.0);
            } else {
                colorContext.rgb = mix(fragmentColor.rgb, colorContext.rgb, boostFactor);
            }
        }
    }
    #endif
    colorContext.a *= getClearViewContextFragmentOpacityFactor();
    gatherFragment(colorContext);
}
