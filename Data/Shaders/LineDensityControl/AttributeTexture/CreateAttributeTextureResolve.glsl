-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "CreateAttributeTextureHeader.glsl"

// The maximum number of fragments to account for 100% coverage.
const int MAX_NUM_FRAGMENTS_COVERAGE = 128;
const int MAX_NUM_FRAGMENTS = 1024;

uniform float zNear;
uniform float zFar;

/**
 * Channel 0: Maximum importance M.
 * Channel 1: Maximum importance depth D.
 * Channel 2: Coverage C.
 * Channel 3: Directional variance V.
 * IMPORTANT: Disable blend, as the alpha channel is also used as normal data!
 */
layout(location = 0) out vec4 outputRenderTarget;

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array.
    uint fragOffset = startOffset[pixelIndex];

    // Data for this pixel of the attribute texture.
    float maximumImportance = 0.0f;
    float maximumImportanceDepth = 1.0; // far depth
    vec3 lineDirectionSum = vec3(0.0);

    // Collect all fragments for this pixel,
    int numFrags = 0;
    LinkedListFragmentNodeAttributeTextures fragmentNode;
    for (int i = 0; i < MAX_NUM_FRAGMENTS; i++) {
        if (fragOffset == -1) {
            // End of list reached.
            break;
        }

        fragmentNode = fragmentBuffer[fragOffset];
        fragOffset = fragmentNode.next;

        // Compute the maximum importance and the corresponding depth.
        if ((fragmentNode.importanceAttribute == maximumImportance && fragmentNode.depth < maximumImportanceDepth)
                || fragmentNode.importanceAttribute > maximumImportance) {
            maximumImportance = fragmentNode.importanceAttribute;
            maximumImportanceDepth = fragmentNode.depth;
        }

        // Add the line direction to the line direction sum for computing the directional variance.
        vec3 lineDirection = vec3(0.0);
        unpackLineDirection(fragmentNode.directionQuantized, lineDirection);
        lineDirectionSum += lineDirection;

        numFrags++;
    }

    // Compute the coverage.
    float coverage = float(min(numFrags, MAX_NUM_FRAGMENTS_COVERAGE)) / float(MAX_NUM_FRAGMENTS_COVERAGE);

    // Compute the directional variance.
    float directionalVariance = 0.0;
    if (numFrags > 0) {
        directionalVariance = 1.0 - length(1.0 / float(numFrags) * lineDirectionSum);
    }

    // Write all computed values to the different image channels.
    outputRenderTarget = vec4(maximumImportance, maximumImportanceDepth, coverage, directionalVariance);
}