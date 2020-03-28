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

void main()
{
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

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

/**
 * Computes the distance of a point to a line segment.
 * See: http://geomalgorithms.com/a02-_lines.html
 *
 * @param p The position of the point.
 * @param l0 The first line point.
 * @param l1 The second line point.
 * @return The distance of p to the line segment.
 */
float distanceToLineSegment(vec3 p, vec3 l0, vec3 l1) {
    vec3 v = l1 - l0;
    vec3 w = p - l0;
    float c1 = dot(v, w);
    if (c1 <= 0.0) {
        return length(p - l0);
    }

    float c2 = dot(v, v);
    if (c2 <= c1) {
        return length(p - l1);
    }

    float b = c1 / c2;
    vec3 pb = l0 + b * v;
    return length(p - pb);
}

vec4 flatShadingWireframeSurfaceHalo(in vec4 baseColor, out float fragmentDepthFrag) {
    const vec3 n = normalize(fragmentNormal);
    const vec3 t = normalize(fragmentTangent);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);

    // n lies in the orthonormal space of t. Thus, project v onto this plane orthogonal to t to compute the angle to n.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

    float angle = abs(acos(dot(newV, n)));

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.65;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, angle);
    vec4 color = vec4(mix(baseColor.rgb, vec3(1.0, 1.0, 1.0),
    smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, angle)), baseColor.a * coverage);

    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (angle >= WHITE_THRESHOLD - EPSILON) {
        fragmentDepth += 0.004;
    }
    fragmentDepthFrag = fragmentDepth;

    return color;
}

/**
 * Flat shading, but adds a constant-sized halo at the outline of the tube. Assumes the following global variables are
 * given: cameraPosition, fragmentPositionWorld, fragmentNormal, fragmentTangent.
*/
vec4 flatShadingWireframeSurfaceTronHalo(in vec4 baseColor, out float fragmentDepthFrag) {
    const vec3 n = normalize(fragmentNormal);
    const vec3 t = normalize(fragmentTangent);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);

    // n lies in the orthonormal space of t. Thus, project v onto this plane orthogonal to t to compute the angle to n.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

    float angle = abs(acos(dot(newV, n)));

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float CORE_THRESHOLD = 0.4;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);

    const float GLOW_STRENGTH = 0.5;
    vec4 colorSolid = vec4(baseColor.rgb, 1.0 - smoothstep(CORE_THRESHOLD - EPSILON, CORE_THRESHOLD + EPSILON, angle));
    vec4 colorGlow = vec4(baseColor.rgb, GLOW_STRENGTH * (1.0 - smoothstep(0.0, 1.2, angle)));

    // Back-to-front blending:
    float a_out = colorGlow.a + colorSolid.a * (1.0 - colorGlow.a);
    vec3 c_out = (colorGlow.rgb * colorGlow.a + colorSolid.rgb * colorSolid.a) / a_out;

    fragmentDepthFrag = fragmentDepth;

    return vec4(c_out, a_out);
}

void main()
{
    const float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WIREFRAME_SMOOTHING = 0.0005 * fragmentDepth;
    const float WIREFRAME_THICKNESS = 0.0008;

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
    float blendFactor = smoothstep(WIREFRAME_THICKNESS - WIREFRAME_SMOOTHING, WIREFRAME_THICKNESS + WIREFRAME_SMOOTHING, minDistance);
    vec3 baseColor = mix(lineBaseColor.rgb, vec3(1.0, 1.0, 1.0), blendFactor);

    if (blendFactor > 0.999) {
        discard;
    }


    vec4 color = vec4(baseColor.rgb, lineBaseColor.a);

    #if defined(DIRECT_BLIT_GATHER)
    fragColor = color;
    #else
    gatherFragment(color);
    #endif
}
