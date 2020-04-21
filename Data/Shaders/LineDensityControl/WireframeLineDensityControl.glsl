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
flat out vec4 lineColors[4];
flat out uvec4 edgeSingularityInformationList;
flat out bvec4 shouldRenderEdge;

#include "TransferFunction.glsl"

// Color lookup table for singular edges.
uniform sampler2D singularEdgeColorLookupTexture;

vec4 lookupSingularEdgeColor(uint edgeSingularityInformation) {
    // x position: The valence minus one. y position: 0 if it is an interior edge, 1 if it is a boundary edge.
    ivec2 samplingPosition = ivec2(int(edgeSingularityInformation >> 2u) - 1, int(edgeSingularityInformation >> 1u) & 1);
    return texelFetch(singularEdgeColorLookupTexture, samplingPosition, 0);
}

uniform float lambda = 1.0f;
uniform float factor_m = 0.25f;
uniform float factor_c = 0.25f;
uniform float factor_v = 0.25f;
uniform float factor_d = 0.25f;

uniform int viewportW;
uniform int viewportH;

/**
 * Channel 0: Maximum importance M.
 * Channel 1: Maximum importance depth D.
 * Channel 2: Coverage C.
 * Channel 3: Directional variance V.
 * IMPORTANT: Disable blend, as the alpha channel is also used as normal data!
 */
uniform sampler2D attributeTexture;

/**
 * This function uses the algorithm uses line density control with pre-generated attribute textures.
 * For more details on line density control see: "Line density control in screen-space via balanced line hierarchies",
 * Mathias Kanzler, Florian Ferstl and Rüdiger Westermann (2016)
 * Computer Graphics and Visualization Group, Technical University Munich, Germany
 * https://www.in.tum.de/cg/research/publications/2016/line-density-control-in-screen-space-via-balanced-line-hierarchies/
 */
float computeVisibilityValue() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    vec4 attributeTextureEntry = texture(
    attributeTexture, vec2(float(x) + 0.5, float(y) + 0.5) / vec2(viewportW, viewportH));
    float maximumImportance = attributeTexture[0];
    float maximumImportanceDepth = attributeTexture[1];
    float coverage = attributeTexture[2];
    float directionalVariance = attributeTexture[3];

    // Compute the visibility value.
    float P = factor_m * maximumImportance + factor_c * coverage + factor_v * directionalVariance;
    if (depth < maximumImportanceDepth) {
        P += d;
    }
    float rho_i = 1.0 / (1.0 + (1.0 - pow(g_i, lambda)) * P);
    return rho_i;
}

const float LOD_EPSILON = 0.001;

void main()
{
    int globalId = gl_VertexID;
    int faceId = globalId / 4;
    int vertexId = globalId % 4;

    float rho_i = computeVisibilityValue();

    HexahedralCellFaceLineDensityControl hexahedralCellFace = hexahedralCellFaces[faceId];

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
        edgeSingularityInformationList[i] = edgeSingularityInformation;
        shouldRenderEdge[i] = hexahedralCellFace.edgeLodValues[i] <= rho_i + LOD_EPSILON;
    }

    // Copy the face data.
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;
    }

    vec4 vertexPosition = hexahedralCellFace.vertexPositions[vertexId];
    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    gl_Position = mvpMatrix * vertexPosition;
}


-- Fragment

#version 430 core

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

in vec3 fragmentPositionWorld;
flat in vec3 vertexPositions[4];
flat in vec4 lineColors[4];
flat in uvec4 edgeSingularityInformationList;
flat in bvec4 shouldRenderEdge;

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

void main()
{
    // Stop if all edges are discarded.
    if (!any(shouldRenderEdge)) {
        discard;
    }

    // Compute the distance to the edges and get the minimum distance.
    float minDistance = 1e9;
    int minDistanceIndex = 0;
    float currentDistance;
    for (int i = 0; i < 4; i++) {
        currentDistance = distanceToLineSegment(
                fragmentPositionWorld, vertexPositions[i], vertexPositions[(i + 1) % 4]);
        if (currentDistance < minDistance && shouldRenderEdge[i]) {
            minDistance = currentDistance;
            minDistanceIndex = i;
        }
    }

    //vec4 lineBaseColor = vec4(mix(lineColors[minDistanceIndex].rgb, vec3(0.0), 0.4), lineColors[minDistanceIndex].a);
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

    #if defined(DIRECT_BLIT_GATHER)
    if (color.a < 0.01) {
        discard;
    }
    //gl_FragDepth = gl_FragCoord.z - 0.00001;
    gl_FragDepth = convertLinearDepthToDepthBufferValue(fragmentDepth);
    fragColor = color;
    #else
    gatherFragmentCustomDepth(color, fragmentDepth);
    #endif
}
