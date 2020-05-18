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
out vec4 fragmentPositionClip;
out vec4 fragmentColor;
out float fragmentAttribute;
flat out vec3 vertexPositions[4];
#ifdef USE_PER_LINE_ATTRIBUTES
flat out vec4 lineColors[4];
#else
flat out vec4 vertexColors[4];
#endif
flat out float lineAttributes[4];
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
        #ifdef USE_SINGULAR_EDGE_COLOR_MAP
        if ((edgeSingularityInformation & 1u) == 1u) {
            // Singular edge.
            lineColor = lookupSingularEdgeColor(edgeSingularityInformation);
            lineAttributes[i] = 1.0;
        } else {
        #endif
            // Regular edge.
            lineColor = vec4(transferFunction(hexahedralCellFace.edgeAttributes[i]).rgb, 1.0);
            lineAttributes[i] = hexahedralCellFace.edgeAttributes[i];
        #ifdef USE_SINGULAR_EDGE_COLOR_MAP
        }
        #endif

        #ifdef USE_PER_LINE_ATTRIBUTES
        lineColors[i] = lineColor;
        #endif

        edgeLodValues[i] = hexahedralCellFace.edgeLodValues[i];
        edgeSingularityInformationList[i] = edgeSingularityInformation;
    }

    // Copy the face data.
    for (int i = 0; i < 4; i++) {
        vertexPositions[i] = hexahedralCellFace.vertexPositions[i].xyz;

        #ifndef USE_PER_LINE_ATTRIBUTES
        vertexColors[i] = transferFunction(hexahedralCellFace.vertexAttributes[i]);
        #endif
    }

    vec4 vertexPosition = hexahedralCellFace.vertexPositions[vertexId];
    fragmentPositionWorld = (mMatrix * vertexPosition).xyz;
    fragmentColor = transferFunction(hexahedralCellFace.vertexAttributes[vertexId]);
    fragmentAttribute = hexahedralCellFace.vertexAttributes[vertexId];
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
#ifdef USE_PER_LINE_ATTRIBUTES
flat in vec4 lineColors[4];
#else
flat in vec4 vertexColors[4];
#endif
flat in float lineAttributes[4];
flat in float edgeLodValues[4];
flat in uint edgeSingularityInformationList[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform float lineWidth;
uniform float maxLodValue;
uniform float selectedLodValueFocus;
uniform float selectedLodValueContext;
uniform float importantLineBoostFactor;

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#define GATHER_NO_DISCARD // as we gather two fragments in one shader
#include OIT_GATHER_HEADER
#endif

#include "PointToLineDistance.glsl"
#ifndef USE_PER_LINE_ATTRIBUTES
#include "ClosestPointOnLine.glsl"
#endif

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
    vec3 negativeLookingDirection = -lookingDirection; // Assuming right-handed coordinate system.
    vec3 projectedPoint;
    rayPlaneIntersection(cameraPosition, normalize(fragmentPositionWorld - cameraPosition), sphereCenter, negativeLookingDirection, projectedPoint);
    float screenSpaceSphereDistanceNormalized = length(projectedPoint - sphereCenter) / sphereRadius;

    float lineWidthPrime = lineWidth * (-distanceToFocusPointNormalized * 0.4 + 1.0);
    float lineRadius = lineWidthPrime / 2.0f;

    // Volume color.
    vec4 volumeColor = fragmentColor;
    volumeColor.a *= pow(distanceToFocusPointNormalized, 4.0);//contextFactor;
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

    bool isSingularEdge = (edgeSingularityInformationList[minDistanceIndex] & 1u) == 1u;
    //vec4 lineBaseColor = vec4(mix(lineColors[minDistanceIndex].rgb, vec3(0.0), 0.4), lineColors[minDistanceIndex].a);
    #ifdef USE_PER_LINE_ATTRIBUTES
    vec4 lineBaseColor = lineColors[minDistanceIndex];
    #else
    vec3 linePoints[2];
    linePoints[0] = vertexPositions[minDistanceIndex];
    linePoints[1] = vertexPositions[(minDistanceIndex + 1) % 4];
    vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
    float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
    vec4 lineBaseColor = vec4(mix(vertexColors[minDistanceIndex].rgb, vertexColors[(minDistanceIndex + 1) % 4].rgb, interpolationFactor), 1.0);
    #endif
    //if (!isSingularEdge) {
    //    vec3 lineBaseColorFocus = mix(lineBaseColor.rgb, vec3(0.0), 0.3);
    //    vec3 lineBaseColorContext = mix(fragmentColor.rgb, vec3(1.0), 0.3);
    //    lineBaseColor.rgb = mix(lineBaseColorContext, lineBaseColorFocus, focusFactor);
    //}

    vec3 lineBaseColorFocus = mix(lineBaseColor.rgb, vec3(0.0), 0.3);
    vec3 lineBaseColorContext = mix(fragmentColor.rgb, vec3(1.0), 0.3);
    lineBaseColor.rgb = mix(lineBaseColorContext, lineBaseColorFocus, focusFactor);

    float lineCoordinates = max(minDistance / lineRadius, 0.0);
    float lineCoordinatesAll = max(minDistanceAll / lineRadius * 1.5, 0.0);
    if (lineCoordinates <= 1.0) {
        float depthCueFactor = min(contextFactor, focusFactor);
        float lineColorToVolumeColorBlendFactor = lodLevelOpacityFactor;

        float depthCueFactorFocus = clamp(pow(distanceToFocusPointNormalized, 1.9), 0.0, 1.0);
        float depthCueFactorDistance = clamp(fragmentDistance, 0.0, 1.0) * 0.002 / lineWidthPrime;

        // Color depth cue.
        lineBaseColor.rgb = mix(lineBaseColor.rgb, vec3(0.5, 0.5, 0.5), depthCueFactor * 0.6);

        // Fade out the outline with increasing distance to the viewer and increasing distance to the focus center.
        vec3 outlineColor = vec3(1.0, 1.0, 1.0);
        // Outline color depth cue.
        outlineColor = mix(outlineColor, lineBaseColor.rgb, clamp(max(depthCueFactorFocus, depthCueFactorDistance), 0.0, 1.0));

        // Fade out the outline with increasing distance
        const float EPSILON = clamp(fragmentDistance / lineRadius * 0.0005, 0.0, 0.49);
        const float WHITE_THRESHOLD = 0.7 + (0.3 + EPSILON) * contextFactor;
        float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
        vec4 lineColor = vec4(mix(lineBaseColor.rgb, outlineColor,
                smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), lineBaseColor.a);

        if (lineCoordinates >= WHITE_THRESHOLD - EPSILON) {
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
        #ifdef USE_PER_LINE_ATTRIBUTES
        vec4 lineBaseColorAll = lineColors[minDistanceIndexAll];
        #else
        vec3 linePoints[2];
        linePoints[0] = vertexPositions[minDistanceIndexAll];
        linePoints[1] = vertexPositions[(minDistanceIndexAll + 1) % 4];
        vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
        float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
        vec4 lineBaseColorAll = vec4(mix(vertexColors[minDistanceIndexAll].rgb, vertexColors[(minDistanceIndexAll + 1) % 4].rgb, interpolationFactor), 1.0);
        #endif

        vec3 lineColor = mix(lineBaseColor.rgb, vec3(1.0), 0.1);
        blendedColor.rgb = mix(volumeColor.rgb, lineColor.rgb, clamp(0.6 - fragmentDistance, 0.0, 0.3));
        blendedColor.a = clamp(blendedColor.a * 1.5, 0.0, 1.0);
    #endif
    }
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
#ifdef USE_PER_LINE_ATTRIBUTES
flat in vec4 lineColors[4];
#else
flat in vec4 vertexColors[4];
#endif
flat in float lineAttributes[4];
flat in float edgeLodValues[4];
flat in uint edgeSingularityInformationList[4];

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform ivec2 viewportSize;
uniform float lineWidth;
uniform float maxLodValue;
uniform float selectedLodValueFocus;
uniform float selectedLodValueContext;
uniform float importantLineBoostFactor;

// Camera data
uniform vec3 cameraPosition;
uniform vec3 lookingDirection;

// Focus region data
uniform vec2 sphereCenterScreen;
uniform float sphereRadiusPixels;

#if !defined(DIRECT_BLIT_GATHER)
#define GATHER_NO_DISCARD // as we gather two fragments in one shader
#include OIT_GATHER_HEADER
#endif

#include "PointToLineDistance.glsl"
#ifndef USE_PER_LINE_ATTRIBUTES
#include "ClosestPointOnLine.glsl"
#endif

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

    float lineWidthPrime = lineWidth * (-screenSpaceSphereDistanceNormalized * 0.4 + 1.0);
    float lineRadius = lineWidthPrime / 2.0f;

    // Volume color.
    vec4 volumeColor = fragmentColor;
    volumeColor.a *= contextFactor;
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

    bool isSingularEdge = (edgeSingularityInformationList[minDistanceIndex] & 1u) == 1u;
    //vec4 lineBaseColor = vec4(mix(lineColors[minDistanceIndex].rgb, vec3(0.0), 0.4), lineColors[minDistanceIndex].a);
    #ifdef USE_PER_LINE_ATTRIBUTES
    vec4 lineBaseColor = lineColors[minDistanceIndex];
    #else
    vec3 linePoints[2];
    linePoints[0] = vertexPositions[minDistanceIndex];
    linePoints[1] = vertexPositions[(minDistanceIndex + 1) % 4];
    vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
    float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
    vec4 lineBaseColor = vec4(mix(vertexColors[minDistanceIndex].rgb, vertexColors[(minDistanceIndex + 1) % 4].rgb, interpolationFactor), 1.0);
    #endif
    //if (!isSingularEdge) {
    //    vec3 lineBaseColorFocus = mix(lineBaseColor.rgb, vec3(0.0), 0.3);
    //    vec3 lineBaseColorContext = mix(fragmentColor.rgb, vec3(1.0), 0.3);
    //    lineBaseColor.rgb = mix(lineBaseColorContext, lineBaseColorFocus, focusFactor);
    //}

    vec3 lineBaseColorFocus = mix(lineBaseColor.rgb, vec3(0.0), 0.3);
    vec3 lineBaseColorContext = mix(fragmentColor.rgb, vec3(1.0), 0.3);
    lineBaseColor.rgb = mix(lineBaseColorContext, lineBaseColorFocus, focusFactor);

    float lineCoordinates = max(minDistance / lineRadius, 0.0);
    float lineCoordinatesAll = max(minDistanceAll / lineRadius * 1.5, 0.0);
    if (isLineNear && lineCoordinates <= 1.0) {
        float depthCueFactor = min(contextFactor, focusFactor);
        float lineColorToVolumeColorBlendFactor = lodLevelOpacityFactor;

        float depthCueFactorFocus = clamp(pow(screenSpaceSphereDistanceNormalized, 1.9), 0.0, 1.0);
        float depthCueFactorDistance = clamp(fragmentDistance, 0.0, 1.0) * 0.002 / lineWidthPrime;

        // Color depth cue.
        lineBaseColor.rgb = mix(lineBaseColor.rgb, vec3(0.5, 0.5, 0.5), depthCueFactor * 0.6);

        // Fade out the outline with increasing distance to the viewer and increasing distance to the focus center.
        vec3 outlineColor = vec3(1.0, 1.0, 1.0);
        // Outline color depth cue.
        outlineColor = mix(outlineColor, lineBaseColor.rgb, clamp(max(depthCueFactorFocus, depthCueFactorDistance), 0.0, 1.0));

        // Fade out the outline with increasing distance
        const float EPSILON = clamp(fragmentDistance / lineRadius * 0.0005, 0.0, 0.49);
        const float WHITE_THRESHOLD = 0.7 + (0.3 + EPSILON) * contextFactor;
        float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
        vec4 lineColor = vec4(mix(lineBaseColor.rgb, outlineColor,
                smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), lineBaseColor.a);

        if (lineCoordinates >= WHITE_THRESHOLD - EPSILON) {
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
        //const float EPSILON = clamp(fragmentDistance / lineRadius * 0.0005, 0.0, 0.49);
        //float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates); // TODO
    #ifdef ACCENTUATE_ALL_EDGES
        #ifdef USE_PER_LINE_ATTRIBUTES
        vec4 lineBaseColorAll = lineColors[minDistanceIndexAll];
        #else
        vec3 linePoints[2];
        linePoints[0] = vertexPositions[minDistanceIndexAll];
        linePoints[1] = vertexPositions[(minDistanceIndexAll + 1) % 4];
        vec3 closestLinePoint = getClosestPointOnLineSegment(fragmentPositionWorld, linePoints[0], linePoints[1]);
        float interpolationFactor = length(closestLinePoint - linePoints[0]) / length(linePoints[1] - linePoints[0]);
        vec4 lineBaseColorAll = vec4(mix(vertexColors[minDistanceIndexAll].rgb, vertexColors[(minDistanceIndexAll + 1) % 4].rgb, interpolationFactor), 1.0);
        #endif

        vec3 lineColor = mix(lineBaseColor.rgb, vec3(1.0), 0.1);
        blendedColor.rgb = mix(volumeColor.rgb, lineColor.rgb, clamp(0.6 - fragmentDistance, 0.0, 0.3));
        blendedColor.a = clamp(blendedColor.a * 1.5, 0.0, 1.0);
    #endif
    }
    #endif

    gatherFragmentCustomDepth(blendedColor, fragmentDistance);
}
