
#include "LinkedListVolumeHeader.glsl"

out vec4 fragColor;

void gatherFragmentVolumeFrontFace(vec4 color) {
    // TODO: Test if this could possibly make problems (i.e., back face culled but not front face or vice versa).
    /*if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }*/

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.depth = length(fragmentPositionWorld - cameraPosition);
    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounterVolumeFrontFaces);

    if (insertIndex < linkedListVolumeFrontFacesCapacity) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffsetVolumeFrontFaces[pixelIndex], insertIndex);
        fragmentBufferVolumeFrontFaces[insertIndex] = frag;
    }
}

void gatherFragmentVolumeBackFace(vec4 color) {
    // TODO: Test if this could possibly make problems (i.e., back face culled but not front face or vice versa).
    /*if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }*/

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.depth = length(fragmentPositionWorld - cameraPosition);
    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounterVolumeBackFaces);

    if (insertIndex < linkedListVolumeBackFacesCapacity) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffsetVolumeBackFaces[pixelIndex], insertIndex);
        fragmentBufferVolumeBackFaces[insertIndex] = frag;
    }
}

// Not called gatherFragmentVolumeSurface, as surface fragment is the standard fallback.
void gatherFragment(vec4 color) {
    if (color.a < 0.001) {
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
    frag.color = packUnorm4x8(color);
    frag.depth = length(fragmentPositionWorld - cameraPosition);
    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounterSurface);

    if (insertIndex < linkedListSurfaceCapacity) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffsetSurface[pixelIndex], insertIndex);
        fragmentBufferSurface[insertIndex] = frag;
    }
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    if (color.a < 0.001) {
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
    frag.color = packUnorm4x8(color);
    frag.depth = depth;
    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounterSurface);

    if (insertIndex < linkedListSurfaceCapacity) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffsetSurface[pixelIndex], insertIndex);
        fragmentBufferSurface[insertIndex] = frag;
    }
}
