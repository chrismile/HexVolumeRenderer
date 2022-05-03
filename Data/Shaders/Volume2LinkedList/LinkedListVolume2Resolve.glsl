-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

uniform float zNear;
uniform float zFar;

#include "DepthHelper.glsl"
#include "LinkedListVolume2Header.glsl"

uint colorList[MAX_NUM_FRAGS];
uint depthList[MAX_NUM_FRAGS];

out vec4 fragColor;

/**
 * The implementation of the GPU-optimized heap is based on the original code from the bachelor's thesis "Opacity-Based
 * Rendering of Lagrangian Particle rajectories in Met.3D" by Maximilian Bandle, Technische Universität München.
 * It was optimized for the use in volume rendering by Christoph Neuhauser, Technische Universität München.
 */
// Swap two Frags in color and depth Array => Avoid bacause expensive
void swapFrags(uint i, uint j) {
    uint cTemp = colorList[i];
    colorList[i] = colorList[j];
    colorList[j] = cTemp;
    uint dTemp = depthList[i];
    depthList[i] = depthList[j];
    depthList[j] = dTemp;
}

void minHeapSink4(uint x, uint fragsCount) {
    uint c, t; // Child, Tmp
    while ((t = 4 * x + 1) < fragsCount) {
        if (t + 1 < fragsCount && depthList[t] > depthList[t+1]) {
            // 1st vs 2nd
            c = t + 1;
        } else {
            c = t;
        }

        if (t + 2 < fragsCount && depthList[c] > depthList[t+2]) {
            // Smallest vs 3rd
            c = t + 2;
        }

        if (t + 3 < fragsCount && depthList[c] > depthList[t+3]) {
            // Smallest vs 3rd
            c = t + 3;
        }

        if (depthList[x] <= depthList[c]) {
            return;
        } else {
            swapFrags(x, c);
            x = c;
        }
    }
}

void getNextFragment(in uint i, in uint fragsCount, out vec4 color, out float depth, out bool boundary) {
    minHeapSink4(0, fragsCount - i);
    color = unpackUnorm4x8(colorList[0]);
    depth = convertDepthBufferValueToLinearDepth(unpackDepth(depthList[0]));
    boundary = (depthList[0] & 1u) == 1u ? true : false;
    colorList[0] = colorList[fragsCount - i - 1];
    depthList[0] = depthList[fragsCount - i - 1];
}

vec4 frontToBackPQ_Volume(uint fragsCount) {
    uint i = 0;

    // Bring it to heap structure
    for (i = fragsCount/4; i > 0; --i) {
        // First is not one right place - will be done in for
        minHeapSink4(i, fragsCount); // Sink all inner nodes
    }

    vec4 fragment1Color, fragment2Color;
    float fragment1Depth, fragment2Depth;
    bool fragment1Boundary, fragment2Boundary;
    float accumDepth = 0.0, currLengthTraveled;
    getNextFragment(0, fragsCount, fragment2Color, fragment2Depth, fragment2Boundary);

    // Start with transparent Ray
    vec4 rayColor = vec4(0.0);
    vec4 currentColor;
    for (uint i = 1; i < fragsCount && rayColor.a < 0.99; i++) {
        // Load the new fragment.
        fragment1Color = fragment2Color;
        fragment1Depth = fragment2Depth;
        fragment1Boundary = fragment2Boundary;
        getNextFragment(i, fragsCount, fragment2Color, fragment2Depth, fragment2Boundary);

        // Skip if the closest fragment is a boundary face.
        if (fragment1Boundary) {
            continue;
        }

        // Compute the color of the fragment.
        currLengthTraveled = fragment2Depth - fragment1Depth;
        float volumeOpacityFactor = clamp(1.0 - exp(-fragment1Color.a * VOLUME_FACTOR * currLengthTraveled), 0.0, 1.0);
        currentColor = vec4(fragment1Color.rgb, volumeOpacityFactor);
        accumDepth += currLengthTraveled;

        // FTB Blending.
        rayColor.rgb = rayColor.rgb + (1.0 - rayColor.a) * currentColor.a * currentColor.rgb;
        rayColor.a = rayColor.a + (1.0 - rayColor.a) * currentColor.a;
    }

    rayColor.rgb = rayColor.rgb / rayColor.a; // Correct rgb with alpha
    return rayColor;
}

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array
    uint fragOffset = startOffset[pixelIndex];

#ifdef INITIALIZE_ARRAY_POW2
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        colorList[i] = 0;
        depthList[i] = 0;
    }
#endif

    // Collect all fragments for this pixel
    int numFrags = 0;
    LinkedListFragmentNode fragment;
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

        fragment = fragmentBuffer[fragOffset];
        fragOffset = fragment.next;

        colorList[i] = fragment.color;
        depthList[i] = fragment.depth;

        numFrags++;
    }

    if (numFrags <= 1) {
        discard;
    }

    fragColor = frontToBackPQ_Volume(numFrags);
}