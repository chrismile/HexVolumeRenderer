-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "LinkedListHeader.glsl"

uint colorList[MAX_NUM_FRAGS];
#ifdef DEPTH_TYPE_UINT
uint depthList[MAX_NUM_FRAGS];
#else
float depthList[MAX_NUM_FRAGS];
#endif

#ifdef DEPTH_TYPE_UINT
#include "FloatPack.glsl"
#include "ColorPack.glsl"
#endif

#include "LinkedListSort.glsl"

#ifdef USE_QUICKSORT
#include "LinkedListQuicksort.glsl"
#endif

out vec4 fragColor;

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array
    uint fragOffset = startOffset[pixelIndex];

#ifdef INITIALIZE_ARRAY_POW2
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        colorList[i] = 0;
#ifdef DEPTH_TYPE_UINT
        depthList[i] = 0;
#else
        depthList[i] = 0.0;
#endif
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

#ifdef FRAGMENT_BUFFER_ARRAY
        fragment = fragmentBuffers[fragOffset / NUM_FRAGS_PER_BUFFER].fragmentBuffer[fragOffset % NUM_FRAGS_PER_BUFFER];
#else
        fragment = fragmentBuffer[fragOffset];
#endif
        fragOffset = fragment.next;

        colorList[i] = fragment.color;
        depthList[i] = fragment.depth;

        numFrags++;
    }

    if (numFrags == 0) {
        discard;
    }

    fragColor = sortingAlgorithm(numFrags);
}