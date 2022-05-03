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
/*struct HexahedralCellFaceUnified {
    vec4 vertexPositions[4];
    float vertexAttributes[4];
    float edgeAttributes[4];
    float edgeLodValues[4];
    uint edgeSingularityInformationList[4];
};
struct HexahedralCellFaceUnified {
    vec4 vertexPositions[4];
    float vertexAttributes[4];
    float edgeAttributes[4];
    float edgeLodValues[4];
    //uint edgeSingularityInformationList[4];
};*/

struct HexahedralCellFaceUnified {
    uint vertexIdx[4];
    uint edgeIdx[4];
};

struct HexahedralCellVertexUnified {
    vec3 vertexPosition;
    float vertexAttribute;
};

struct HexahedralCellEdgeUnified {
    float edgeAttribute;
    float edgeLodValue;
};

layout (std430, binding = 6) readonly buffer FaceBuffer {
    HexahedralCellFaceUnified hexahedralCellFaces[];
};

layout (std430, binding = 7) readonly buffer VertexBuffer {
    HexahedralCellVertexUnified hexahedralCellVertices[];
};

layout (std430, binding = 8) readonly buffer EdgeBuffer {
    HexahedralCellEdgeUnified hexahedralCellEdges[];
};

#ifdef RENDER_FOCUS_FACES
layout (std430, binding = 9) readonly buffer FaceCellLinkBuffer {
    // Link to the two incident cells for a face. Second entry can be 0xFFFFFFFF if only one cell.
    uvec2 hexahedralCellFacesCellLinks[];
};
layout (std430, binding = 10) readonly buffer CellBuffer {
    float hexahedralCells[]; // Cell deformation measure.
};
uniform float importantCellFactor;
out vec4 cellColor;
#endif


out vec3 fragmentPositionWorld;
out vec4 fragmentPositionClip;
out vec4 fragmentColor;
out float fragmentAttribute;
flat out vec3 vertexPositions[4];
#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
flat out vec4 lineColors[4];
#else
//flat out vec4 vertexColors[4];
#endif
flat out float lineAttributes[4];
flat out float edgeLodValues[4];
//flat out uint edgeSingularityInformationList[4];

#if defined (RENDER_FOCUS_FACES) && defined(USE_LIGHTING)
out vec3 fragmentNormal;
#endif

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
    vec4 vertexPosition;
    float vertexAttribute;

#ifdef RENDER_FOCUS_FACES
    uvec2 cellLinks = hexahedralCellFacesCellLinks[faceId];
    float maxCellValue = hexahedralCells[cellLinks.x];
    if (cellLinks.y != 0xFFFFFFFF) {
        maxCellValue = max(maxCellValue, hexahedralCells[cellLinks.y]);
    }
    bool showFace = maxCellValue >= 1.0 - importantCellFactor;
    cellColor = vec4(transferFunction(maxCellValue).rgb, showFace ? 1.0 : 0.0);
#endif

    // Copy the vertex and edge data.
    for (int i = 0; i < 4; i++) {
        HexahedralCellVertexUnified hexahedralCellVertex = hexahedralCellVertices[hexahedralCellFace.vertexIdx[i]];
        HexahedralCellEdgeUnified hexahedralCellEdge = hexahedralCellEdges[hexahedralCellFace.edgeIdx[i]];
        vertexPositions[i] = hexahedralCellVertex.vertexPosition;

        if (i == vertexId) {
            vertexPosition = vec4(hexahedralCellVertex.vertexPosition, 1.0);
            vertexAttribute = hexahedralCellVertex.vertexAttribute;
        }

        vec4 lineColor;
        //#ifdef USE_SINGULAR_EDGE_COLOR_MAP
        //uint edgeSingularityInformation = hexahedralCellFace.edgeSingularityInformationList[i];
        //if ((edgeSingularityInformation & 1u) == 1u) {
        //    // Singular edge.
        //    lineColor = lookupSingularEdgeColor(edgeSingularityInformation);
        //    lineAttributes[i] = 1.0;
        //} else {
        //#endif
            // Regular edge.
            lineColor = vec4(transferFunction(hexahedralCellEdge.edgeAttribute).rgb, 1.0);
            lineAttributes[i] = hexahedralCellEdge.edgeAttribute;
        //#ifdef USE_SINGULAR_EDGE_COLOR_MAP
        //}
        //#endif

#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
        lineColors[i] = lineColor;
#endif

        edgeLodValues[i] = hexahedralCellEdge.edgeLodValue;
        //edgeSingularityInformationList[i] = edgeSingularityInformation;
    }

    // Copy the face data.
    //for (int i = 0; i < 4; i++) {
        //vertexPositions[i] = hexahedralCellVertex.vertexPosition.xyz;

        /*#if !(defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP))
        vertexColors[i] = transferFunction(hexahedralCellFace.vertexAttributes[i]);
        #endif*/
    //}

#if defined (RENDER_FOCUS_FACES) && defined(USE_LIGHTING)
    fragmentNormal = normalize(cross(normalize(vertexPositions[0] - vertexPositions[1]), normalize(vertexPositions[2] - vertexPositions[1])));
#endif

    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    fragmentColor = transferFunction(vertexAttribute);
    fragmentAttribute = vertexAttribute;
    fragmentPositionClip = mvpMatrix * vertexPosition;
    gl_Position = fragmentPositionClip;
}

-- Fragment.ClearView_ObjectSpace

#version 430 core

in vec3 fragmentPositionWorld;
in vec4 fragmentPositionClip;
in vec4 fragmentColor;
in float fragmentAttribute;
flat in vec3 vertexPositions[4];
#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
flat in vec4 lineColors[4];
#else
//flat in vec4 vertexColors[4];
#endif
flat in float lineAttributes[4];
flat in float edgeLodValues[4];
//flat in uint edgeSingularityInformationList[4];

#ifdef RENDER_FOCUS_FACES
in vec4 cellColor;
#endif

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform float lineWidth;
uniform float maxLodValue;
uniform float selectedLodValueFocus;
uniform float selectedLodValueContext;
uniform float importantLineBoostFactor;
uniform vec3 backgroundColor = vec3(0.0);
uniform vec3 foregroundColor = vec3(1.0);

#ifdef USE_FOCUS_OUTLINE
uniform vec4 focusOutlineColor;
#endif

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#define GATHER_NO_DISCARD // as we gather two fragments in one shader
#include OIT_GATHER_HEADER
#endif

#include "PointToLineDistance.glsl"
#if !(defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP))
#include "ClosestPointOnLine.glsl"
#endif

#if defined (RENDER_FOCUS_FACES) && defined(USE_LIGHTING)
in vec3 fragmentNormal;
#include "LightingFlat.glsl"
#endif

#include "DepthCues.glsl"
#include "Antialiasing.glsl"

void main()
{
    float distanceToFocusPointNormalized = min(length(fragmentPositionWorld - sphereCenter) / sphereRadius, 1.0);
    float fragmentDistance = length(fragmentPositionWorld - cameraPosition);
    float contextFactor = getClearViewContextFragmentOpacityFactor();
    //float focusFactor = getClearViewFocusFragmentOpacityFactor();
    //float focusFactor = 1.0 - pow(distanceToFocusPointNormalized, 10.0);

    const float LOD_BLEND_FACTOR_BLEND_START = 0.7;
    float focusFactor = 1.0;
    if (distanceToFocusPointNormalized >= 1.0) {
        focusFactor = 0.0;
    } else if (distanceToFocusPointNormalized > LOD_BLEND_FACTOR_BLEND_START){
        float t = (distanceToFocusPointNormalized - LOD_BLEND_FACTOR_BLEND_START) / (1.0 - LOD_BLEND_FACTOR_BLEND_START);
        focusFactor = 1.0 - t * t * (3.0 - 2.0 * t);
    }

    // Intersect view ray with plane parallel to camera looking direction containing the sphere center.
    //vec3 negativeLookingDirection = -lookingDirection; // Assuming right-handed coordinate system.
    //vec3 projectedPoint;
    //rayPlaneIntersection(cameraPosition, normalize(fragmentPositionWorld - cameraPosition), sphereCenter, negativeLookingDirection, projectedPoint);
    //float screenSpaceSphereDistanceNormalized = length(projectedPoint - sphereCenter) / sphereRadius;

    float lineWidthPrime = lineWidth * (-distanceToFocusPointNormalized * 0.3 + 1.0);
#ifdef MODULATE_LINE_THICKNESS_BY_DEPTH
    lineWidthPrime = lineWidthPrime * fragmentDistance * 2.0;
#endif
    float lineRadius = lineWidthPrime / 2.0f;

    // Volume color.
    vec4 volumeColor = fragmentColor;
    volumeColor.a *= pow(distanceToFocusPointNormalized, 4.0);//contextFactor;

#ifdef RENDER_FOCUS_FACES
    if (cellColor.a > 0.0 && focusFactor > 1e-6) {
        vec4 colorSurface = vec4(cellColor.rgb, focusFactor);
#ifdef USE_LIGHTING
        colorSurface = blinnPhongShading(colorSurface);
#endif
        volumeColor.rgb = colorSurface.rgb * colorSurface.a + volumeColor.rgb * volumeColor.a * (1.0 - colorSurface.a);
        volumeColor.a = colorSurface.a + volumeColor.a * (1.0 - colorSurface.a);
        if (volumeColor.a > 1e-4) {
            volumeColor.rgb /= volumeColor.a;
        }
    }
#endif

    vec4 blendedColor = volumeColor;

#ifdef HIGHLIGHT_EDGES
    const float LOD_EPSILON = 0.001;
    float discreteSelectedLodValueFocus = max(selectedLodValueFocus * maxLodValue, LOD_EPSILON);
    float discreteSelectedLodValueContext = max(selectedLodValueContext * maxLodValue, LOD_EPSILON);
    /*float val = max(
            (1.0 - fragmentAttribute) * discreteSelectedLodValueFocus,
            distanceToFocusPointNormalized * discreteSelectedLodValueFocus);
    float lodLevelFocus = val + maxLodValue - (maxLodValue * val) / discreteSelectedLodValueFocus;*/
    float lodLevelContext = discreteSelectedLodValueContext;

    bool isLineNear = false;
    bool isAnyLineNear = false;
    float minDistance = 1e9;
    int minDistanceIndex = 0;
    float minDistanceAll = 1e9;
    int minDistanceIndexAll = 0;
    float currentDistance;
    for (int i = 0; i < 4; i++) {
        currentDistance = getDistanceToLineSegment(
                fragmentPositionWorld, vertexPositions[i], vertexPositions[(i + 1) % 4]);

        float lodLevelFocus = max(
                (lineAttributes[i] < 1.0 - importantLineBoostFactor ? 0.0 : 1.0) * maxLodValue,
                discreteSelectedLodValueFocus);

        float lodLineValue = edgeLodValues[i];
        float discreteLodValue = lodLineValue * maxLodValue;
        float lodLevelOpacityFactor = mix(
                discreteLodValue <= lodLevelContext + LOD_EPSILON ? 1.0 : 0.0,
                discreteLodValue <= lodLevelFocus + LOD_EPSILON ? 1.0 : 0.0,
                focusFactor);
        bool drawLine = lodLevelOpacityFactor > 0.01;

        if (currentDistance < minDistance && drawLine) {
            minDistance = currentDistance;
            minDistanceIndex = i;
            isLineNear = true;
        }
        if (currentDistance < minDistanceAll) {
            minDistanceAll = currentDistance;
            minDistanceIndexAll = i;
            isAnyLineNear = true;
        }
    }

    float lodLevelFocus = max(
            (lineAttributes[minDistanceIndex] < 1.0 - importantLineBoostFactor ? 0.0 : 1.0) * maxLodValue,
            discreteSelectedLodValueFocus);

    float lodLineValue = edgeLodValues[minDistanceIndex];
    float discreteLodValue = lodLineValue * maxLodValue;
    float lodLevelOpacityFactor = mix(
            discreteLodValue <= lodLevelContext + LOD_EPSILON ? 1.0 : 0.0,
            discreteLodValue <= lodLevelFocus + LOD_EPSILON ? 1.0 : 0.0,
            focusFactor);

#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
    vec4 lineBaseColor = lineColors[minDistanceIndex];
#else
    /*vec3 linePoints[2];
    linePoints[0] = vertexPositions[minDistanceIndex];
    linePoints[1] = vertexPositions[(minDistanceIndex + 1) % 4];
    vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
    float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
    vec4 lineBaseColor = vec4(mix(vertexColors[minDistanceIndex].rgb, vertexColors[(minDistanceIndex + 1) % 4].rgb, interpolationFactor), 1.0);*/
    vec4 lineBaseColor = vec4(fragmentColor.rgb, 1.0);
#endif

    vec3 lineBaseColorFocus = mix(lineBaseColor.rgb, backgroundColor, 0.3);
    vec3 lineBaseColorContext = mix(fragmentColor.rgb, foregroundColor, 0.3);
    lineBaseColor.rgb = mix(lineBaseColorContext, lineBaseColorFocus, focusFactor);

    float lineCoordinates = max(minDistance / lineRadius, 0.0);
    float lineCoordinatesAll = max(minDistanceAll / lineRadius * 1.5, 0.0);
    //const float fwidthLineCoordinates = fwidth(lineCoordinates);
    //const float coverage = 1.0 - smoothstep(1.0 - fwidthLineCoordinates, 1.0, lineCoordinates);
    //const float EPSILON = fwidthLineCoordinates;
    if (lineCoordinates <= 1.0) {
        float depthCueFactor = min(contextFactor, focusFactor);
        float lineColorToVolumeColorBlendFactor = lodLevelOpacityFactor;

        float depthCueFactorFocus = clamp(pow(distanceToFocusPointNormalized, 1.9), 0.0, 1.0);
        float depthCueFactorDistance = clamp(fragmentDistance, 0.0, 1.0) * 0.002 / lineWidthPrime;

#ifdef USE_DEPTH_CUES
        float depthCueFactorDepth = computeDepthCueFactor();
        depthCueFactorDepth *= focusFactor;
        lineBaseColor.rgb = mix(lineBaseColor.rgb, vec3(0.0, 0.0, 0.0), depthCueFactorDepth);
#endif

        // Color depth cue.
        lineBaseColor.rgb = mix(lineBaseColor.rgb, vec3(0.5, 0.5, 0.5), depthCueFactor * 0.6);
        //lineBaseColor.rgb = vec3(lodLineValue * 0.25 + 0.03);

        // Fade out the outline with increasing distance to the viewer and increasing distance to the focus center.
        vec3 outlineColor = vec3(1.0, 1.0, 1.0);
        // Outline color depth cue.
        outlineColor = mix(outlineColor, lineBaseColor.rgb, clamp(max(depthCueFactorFocus, depthCueFactorDistance), 0.0, 1.0));

        // Fade out the outline with increasing distance
        const float EPSILON = clamp(getAntialiasingFactor(fragmentDistance / lineRadius), 0.0, 0.49);
        const float WHITE_THRESHOLD = 0.7 + (0.3 + EPSILON) * contextFactor;
        float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
        vec4 lineColor = vec4(mix(lineBaseColor.rgb, outlineColor,
                smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), lineBaseColor.a);

        //const float WHITE_THRESHOLD = 0.7 + (0.3 + EPSILON) * contextFactor;
        //vec4 lineColor = vec4(mix(lineBaseColor.rgb, outlineColor,
        //        smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), lineBaseColor.a);

        if (lineCoordinates >= WHITE_THRESHOLD - EPSILON
#ifdef RENDER_FOCUS_FACES
                && cellColor.a <= 0.0
#endif
        ) {
            fragmentDistance += 0.005;
        }

        // Fade between volume and line color using back-to-front alpha blending.
        lineColor.a *= coverage * lineColorToVolumeColorBlendFactor;
        blendedColor.a = lineColor.a + volumeColor.a * (1.0 - lineColor.a);
        blendedColor.rgb = lineColor.rgb * lineColor.a + volumeColor.rgb * volumeColor.a * (1.0 - lineColor.a);
        if (blendedColor.a > 1e-4) {
            blendedColor.rgb /= blendedColor.a;
        }
    } else if (lineCoordinatesAll <= 1.0) {
#ifdef ACCENTUATE_ALL_EDGES
#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
        vec4 lineBaseColorAll = lineColors[minDistanceIndexAll];
#else
        /*vec3 linePoints[2];
        linePoints[0] = vertexPositions[minDistanceIndexAll];
        linePoints[1] = vertexPositions[(minDistanceIndexAll + 1) % 4];
        vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
        float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
        vec4 lineBaseColorAll = vec4(mix(vertexColors[minDistanceIndexAll].rgb, vertexColors[(minDistanceIndexAll + 1) % 4].rgb, interpolationFactor), 1.0);*/
        vec4 lineBaseColorAll = vec4(fragmentColor.rgb, 1.0);
#endif

        vec3 lineColor = mix(lineBaseColorAll.rgb, foregroundColor, 0.1);
        blendedColor.rgb = mix(volumeColor.rgb, lineColor.rgb, clamp(0.6 - fragmentDistance, 0.0, 0.3));
        blendedColor.a = clamp(blendedColor.a * 1.5, 0.0, 1.0);
#endif
    }
#endif

    // Render outline similar to what ClearView does.
#ifdef USE_FOCUS_OUTLINE
    const float FOCUS_OUTLINE_WIDTH = 2.0;
    //float dist = 1.0 - clamp(abs(length(fragmentPositionWorld - sphereCenter) / sphereRadius) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);
    //float dist = 1.0 - clamp(abs(length(fragmentPositionWorld - sphereCenter) / sphereRadius) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);
    //float dist = 1.0 - clamp(abs(length(projectedPoint - sphereCenter) / sphereRadius) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);
    //float zdiff = (1.0 - abs(fragmentPositionWorld.z - cameraPosition.z)) * 0.1;
    //float dist = 1.0 - clamp(abs(length(fragmentPositionWorld - sphereCenter) / sphereRadius) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);
    //float dist = 1.0 - clamp(abs(length(fragmentPositionWorld - sphereCenter) / sphereRadius) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);

    vec3 dir = normalize(cameraPosition - fragmentPositionWorld);
    vec3 sphereNormal = normalize(fragmentPositionWorld - sphereCenter);
    //float dist = 1.0 - clamp(abs(length(fragmentPositionWorld - sphereCenter) / sphereRadius) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);
    float dist = abs(1.0 - length(fragmentPositionWorld - sphereCenter) / sphereRadius);
    dist = clamp(1.0 - dist * 20.0, 0.0, 1.0);
    //dist = 1.0;
    //float abc = pow(1.0 - abs(dot(dir, sphereNormal)), 10.0);
    float abc = pow(1.0 - abs(dot(dir, sphereNormal)), 2.0);
    abc = clamp(abc, 0.0, 1.0);

    vec4 outlineColor = vec4(focusOutlineColor.rgb, abc * dist);
    blendedColor = mix(blendedColor, outlineColor, outlineColor.a);
#endif

    gatherFragmentCustomDepth(blendedColor, fragmentDistance);
}


-- Fragment.ClearView_ScreenSpace

#version 430 core

in vec3 fragmentPositionWorld;
in vec4 fragmentPositionClip;
in vec4 fragmentColor;
in float fragmentAttribute;
flat in vec3 vertexPositions[4];
#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
flat in vec4 lineColors[4];
#else
//flat in vec4 vertexColors[4];
#endif
flat in float lineAttributes[4];
flat in float edgeLodValues[4];
//flat in uint edgeSingularityInformationList[4];

#ifdef RENDER_FOCUS_FACES
in vec4 cellColor;
#endif

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform float lineWidth;
uniform float maxLodValue;
uniform float selectedLodValueFocus;
uniform float selectedLodValueContext;
uniform float importantLineBoostFactor;
uniform vec3 backgroundColor = vec3(0.0);
uniform vec3 foregroundColor = vec3(1.0);

// Camera data
uniform vec3 cameraPosition;
uniform vec3 lookingDirection;

// Focus region data
uniform vec2 sphereCenterScreen;
uniform float sphereRadiusPixels;

#ifdef USE_FOCUS_OUTLINE
uniform vec4 focusOutlineColor;
#endif

#if !defined(DIRECT_BLIT_GATHER)
#define GATHER_NO_DISCARD // as we gather two fragments in one shader
#include OIT_GATHER_HEADER
#endif

#include "PointToLineDistance.glsl"
#if !(defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP))
#include "ClosestPointOnLine.glsl"
#endif

#if defined (RENDER_FOCUS_FACES) && defined(USE_LIGHTING)
in vec3 fragmentNormal;
#include "LightingFlat.glsl"
#endif

#include "DepthCues.glsl"
#include "Antialiasing.glsl"

void main()
{
    vec3 fragmentPositionNdc = fragmentPositionClip.xyz / fragmentPositionClip.w;
    // Origin at lower left corner of screen.
    vec2 fragmentWindowPosition = (fragmentPositionNdc.xy * vec2(0.5) + vec2(0.5)) * vec2(viewportSize);
    float screenSpaceSphereDistanceNormalized = min(length(fragmentWindowPosition - sphereCenterScreen) / sphereRadiusPixels, 1.0);

    float fragmentDistance = length(fragmentPositionWorld - cameraPosition);

    const float LOD_BLEND_FACTOR_BLEND_START = 0.7;
    float focusFactor = 1.0;
    if (screenSpaceSphereDistanceNormalized >= 1.0) {
        focusFactor = 0.0;
    } else if (screenSpaceSphereDistanceNormalized > LOD_BLEND_FACTOR_BLEND_START){
        float t = (screenSpaceSphereDistanceNormalized - LOD_BLEND_FACTOR_BLEND_START) / (1.0 - LOD_BLEND_FACTOR_BLEND_START);
        focusFactor = 1.0 - t * t * (3.0 - 2.0 * t);
    }
    float contextFactor = pow(screenSpaceSphereDistanceNormalized, 4.0);
    //float contextFactor = 1.0 - focusFactor;

    float lineWidthPrime = lineWidth * (-screenSpaceSphereDistanceNormalized * 0.3 + 1.0);
#ifdef MODULATE_LINE_THICKNESS_BY_DEPTH
    lineWidthPrime = lineWidthPrime * fragmentDistance * 2.0;
#endif
    float lineRadius = lineWidthPrime / 2.0f;

    // Volume color.
    vec4 volumeColor = fragmentColor;
    // TEST: Show all lines.
    //volumeColor.rgb = mix(volumeColor.rgb, vec3(0.0), clamp((fragmentDistance - 0.35)*1.8, 0.0, 1.0));
    volumeColor.a *= contextFactor;

#ifdef RENDER_FOCUS_FACES
    if (cellColor.a > 0.0 && focusFactor > 1e-6) {
        vec4 colorSurface = vec4(cellColor.rgb, focusFactor);
#ifdef USE_LIGHTING
        colorSurface = blinnPhongShading(colorSurface);
#endif
        volumeColor.rgb = colorSurface.rgb * colorSurface.a + volumeColor.rgb * volumeColor.a * (1.0 - colorSurface.a);
        volumeColor.a = colorSurface.a + volumeColor.a * (1.0 - colorSurface.a);
        if (volumeColor.a > 1e-4) {
            volumeColor.rgb /= volumeColor.a;
        }
    }
#endif

    vec4 blendedColor = volumeColor;

#ifdef HIGHLIGHT_EDGES
    const float LOD_EPSILON = 0.001;
    float discreteSelectedLodValueFocus = selectedLodValueFocus * maxLodValue;
    float discreteSelectedLodValueContext = selectedLodValueContext * maxLodValue;
    float lodLevelContext = discreteSelectedLodValueContext;

    bool isLineNear = false;
    bool isAnyLineNear = false;
    float minDistance = 1e9;
    int minDistanceIndex = 0;
    float minDistanceAll = 1e9;
    int minDistanceIndexAll = 0;
    float currentDistance;
    for (int i = 0; i < 4; i++) {
        currentDistance = getDistanceToLineSegment(
                fragmentPositionWorld, vertexPositions[i], vertexPositions[(i + 1) % 4]);

        float lodLevelFocus = max(
                (lineAttributes[i] < 1.0 - importantLineBoostFactor ? 0.0 : 1.0) * maxLodValue,
                discreteSelectedLodValueFocus);

        float lodLineValue = edgeLodValues[i];
        float discreteLodValue = lodLineValue * maxLodValue;
        float lodLevelOpacityFactor = mix(
                discreteLodValue <= lodLevelContext + LOD_EPSILON ? 1.0 : 0.0,
                discreteLodValue <= lodLevelFocus + LOD_EPSILON ? 1.0 : 0.0,
                focusFactor);
        bool drawLine = lodLevelOpacityFactor > 0.01;

        if (currentDistance < minDistance && drawLine) {
            minDistance = currentDistance;
            minDistanceIndex = i;
            isLineNear = true;
        }
        if (currentDistance < minDistanceAll) {
            minDistanceAll = currentDistance;
            minDistanceIndexAll = i;
            isAnyLineNear = true;
        }
    }

    float lodLevelFocus = max(
            (lineAttributes[minDistanceIndex] < 1.0 - importantLineBoostFactor ? 0.0 : 1.0) * maxLodValue,
            discreteSelectedLodValueFocus);

    float lodLineValue = edgeLodValues[minDistanceIndex];
    float discreteLodValue = lodLineValue * maxLodValue;
    float lodLevelOpacityFactor = mix(
            discreteLodValue <= lodLevelContext + LOD_EPSILON ? 1.0 : 0.0,
            discreteLodValue <= lodLevelFocus + LOD_EPSILON ? 1.0 : 0.0,
            focusFactor);

#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
    vec4 lineBaseColor = lineColors[minDistanceIndex];
#else
    /*vec3 linePoints[2];
    linePoints[0] = vertexPositions[minDistanceIndex];
    linePoints[1] = vertexPositions[(minDistanceIndex + 1) % 4];
    vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
    float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
    vec4 lineBaseColor = vec4(mix(vertexColors[minDistanceIndex].rgb, vertexColors[(minDistanceIndex + 1) % 4].rgb, interpolationFactor), 1.0);*/
    vec4 lineBaseColor = vec4(fragmentColor.rgb, 1.0);
#endif

    vec3 lineBaseColorFocus;
    if (foregroundColor.x < 0.5) {
        // Bright background
        lineBaseColorFocus = mix(lineBaseColor.rgb, foregroundColor, 0.1);
    } else {
        lineBaseColorFocus = mix(lineBaseColor.rgb, backgroundColor, 0.3);
    }
#ifndef USE_SINGULAR_EDGE_COLOR_MAP
    vec3 lineBaseColorContext = mix(fragmentColor.rgb, foregroundColor, 0.3);
#else
    vec3 lineBaseColorContext = mix(lineBaseColor.rgb, foregroundColor, 0.3);
#endif
    lineBaseColor.rgb = mix(lineBaseColorContext, lineBaseColorFocus, focusFactor);

    float lineCoordinates = max(minDistance / lineRadius, 0.0);
    float lineCoordinatesAll = max(minDistanceAll / lineRadius * 1.5, 0.0);
    //const float fwidthLineCoordinates = fwidth(lineCoordinates);
    //const float coverage = 1.0 - smoothstep(1.0 - fwidthLineCoordinates, 1.0, lineCoordinates);
    //const float EPSILON = fwidthLineCoordinates;
    if (isLineNear && lineCoordinates <= 1.0) {
        float depthCueFactor = min(contextFactor, focusFactor);
        float lineColorToVolumeColorBlendFactor = lodLevelOpacityFactor;

        float depthCueFactorFocus = clamp(pow(screenSpaceSphereDistanceNormalized, 1.9), 0.0, 1.0);
        float depthCueFactorDistance = clamp(fragmentDistance, 0.0, 1.0) * 0.002 / lineWidthPrime;

#ifdef USE_DEPTH_CUES
        float depthCueFactorDepth = computeDepthCueFactor();// * 17.0;
        /*if (depthCueFactorDepth > depthCueStrength) {
            depthCueFactorDepth = depthCueStrength;
        }*/
        depthCueFactorDepth *= focusFactor;
        lineBaseColor.rgb = mix(lineBaseColor.rgb, vec3(0.0, 0.0, 0.0), depthCueFactorDepth);
#endif

        // Color depth cue.
        lineBaseColor.rgb = mix(lineBaseColor.rgb, vec3(0.5, 0.5, 0.5), depthCueFactor * 0.6);
        //lineBaseColor.rgb = vec3(lodLineValue * 0.25 + 0.03);

        // Fade out the outline with increasing distance to the viewer and increasing distance to the focus center.
        vec3 outlineColor = vec3(1.0, 1.0, 1.0);
        // Outline color depth cue.
        outlineColor = mix(outlineColor, lineBaseColor.rgb, clamp(max(depthCueFactorFocus, depthCueFactorDistance), 0.0, 1.0));

        // Fade out the outline with increasing distance
        const float EPSILON = clamp(getAntialiasingFactor(fragmentDistance / lineRadius), 0.0, 0.49);
        const float WHITE_THRESHOLD = 0.7 + (0.3 + EPSILON) * contextFactor;
        float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
        vec4 lineColor = vec4(mix(lineBaseColor.rgb, outlineColor,
                smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), lineBaseColor.a);
        //const float WHITE_THRESHOLD = 0.7 + (0.3 + EPSILON) * contextFactor;
        //vec4 lineColor = vec4(mix(lineBaseColor.rgb, outlineColor,
        //        smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), lineBaseColor.a);

        // TEST: Show all lines.
        //lineColor.rgb = mix(volumeColor.rgb, vec3(0.0), 0.7);

        if (lineCoordinates >= WHITE_THRESHOLD - EPSILON
#ifdef RENDER_FOCUS_FACES
                && cellColor.a <= 0.0
#endif
        ) {
            fragmentDistance += 0.005;
        }

        // Fade between volume and line color using back-to-front alpha blending.
        lineColor.a *= coverage * lineColorToVolumeColorBlendFactor;
        blendedColor.a = lineColor.a + volumeColor.a * (1.0 - lineColor.a);
        blendedColor.rgb = lineColor.rgb * lineColor.a + volumeColor.rgb * volumeColor.a * (1.0 - lineColor.a);
        if (blendedColor.a > 1e-4) {
            blendedColor.rgb /= blendedColor.a;
        }
    } else if (lineCoordinatesAll <= 1.0) {
#ifdef ACCENTUATE_ALL_EDGES
#if defined(USE_PER_LINE_ATTRIBUTES) || defined(USE_SINGULAR_EDGE_COLOR_MAP)
        vec4 lineBaseColorAll = lineColors[minDistanceIndexAll];
#else
        /*vec3 linePoints[2];
        linePoints[0] = vertexPositions[minDistanceIndexAll];
        linePoints[1] = vertexPositions[(minDistanceIndexAll + 1) % 4];
        vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
        float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
        vec4 lineBaseColorAll = vec4(mix(vertexColors[minDistanceIndexAll].rgb, vertexColors[(minDistanceIndexAll + 1) % 4].rgb, interpolationFactor), 1.0);*/
        vec4 lineBaseColorAll = vec4(fragmentColor.rgb, 1.0);
#endif

        // TEST: Show all lines.
        //vec3 lineColor = mix(volumeColor.rgb, foregroundColor, 0.15);
        //blendedColor.rgb = mix(volumeColor.rgb, lineColor.rgb, 1.0);
        //blendedColor.a = clamp(blendedColor.a * 1.5, 0.0, 1.0);
        vec3 lineColor = mix(lineBaseColorAll.rgb, foregroundColor, 0.1);
        blendedColor.rgb = mix(volumeColor.rgb, lineColor.rgb, clamp(0.6 - fragmentDistance, 0.0, 0.3));
        blendedColor.a = clamp(blendedColor.a * 1.5, 0.0, 1.0);
#endif
    }
#endif

    // Render outline similar to what ClearView does.
#ifdef USE_FOCUS_OUTLINE
    const float FOCUS_OUTLINE_WIDTH = 2.0;
    float dist = 1.0 - clamp(abs(length(fragmentWindowPosition - sphereCenterScreen) - sphereRadiusPixels) / FOCUS_OUTLINE_WIDTH, 0.0, 1.0);
    vec4 outlineColor = vec4(focusOutlineColor.rgb, dist);
    blendedColor = mix(blendedColor, outlineColor, outlineColor.a);
#endif

    //blendedColor.rgb = vec3(min(computeDepthCueFactor() * 10.0, 1.0));

    gatherFragmentCustomDepth(blendedColor, fragmentDistance);
}
