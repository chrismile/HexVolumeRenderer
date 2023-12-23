
#include "LinkedListHeader.glsl"

out vec4 fragColor;

#ifdef DEPTH_TYPE_UINT
#include "FloatPack.glsl"
#include "ColorPack.glsl"
#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#endif

void gatherFragment(vec4 color) {
    if (color.a < 1e-4) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;

#ifdef DEPTH_TYPE_UINT
    frag.color = packColor30bit(color);
    packFloat22Float10(frag.depth, gl_FragCoord.z, color.a);
#else
    frag.color = packUnorm4x8(color);
    frag.depth = length(fragmentPositionWorld - cameraPosition);//gl_FragCoord.z;//
#endif


    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        //#ifdef FRAGMENT_BUFFER_ARRAY
        //frag.color = packColor30bit(vec4(1.0, 0.0, 0.0, color.a));
        //packFloat22Float10(frag.depth, gl_FragCoord.z, color.a);
        //#endif
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
#ifdef FRAGMENT_BUFFER_ARRAY
        fragmentBuffers[insertIndex / NUM_FRAGS_PER_BUFFER].fragmentBuffer[insertIndex % NUM_FRAGS_PER_BUFFER] = frag;
#else
        fragmentBuffer[insertIndex] = frag;
#endif
    }
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    if (color.a < 1e-4) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;

#ifdef DEPTH_TYPE_UINT
    frag.color = packColor30bit(color);
    packFloat22Float10(frag.depth, convertLinearDepthToDepthBufferValue(depth), color.a);
#else
    frag.color = packUnorm4x8(color);
    frag.depth = depth;
#endif

    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
#ifdef FRAGMENT_BUFFER_ARRAY
        fragmentBuffers[insertIndex / NUM_FRAGS_PER_BUFFER].fragmentBuffer[insertIndex % NUM_FRAGS_PER_BUFFER] = frag;
#else
        fragmentBuffer[insertIndex] = frag;
#endif
    }
}
