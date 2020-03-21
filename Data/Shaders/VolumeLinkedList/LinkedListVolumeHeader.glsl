// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_shader_image_load_store.txt
#extension GL_ARB_shader_image_load_store : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests) in;

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

// A fragment node stores rendering information about one specific fragment
struct LinkedListFragmentNode {
    // RGBA color of the node
    uint color;
    // Depth value of the fragment (in view space)
    float depth;
    // The index of the next node in "nodes" array
    uint next;
};

// fragment-and-link buffer and a start-offset buffer

// Fragment-and-link buffer (linked list) for
layout (std430, binding = 0) coherent buffer FragmentBufferVolumeFrontFaces {
    LinkedListFragmentNode fragmentBufferVolumeFrontFaces[];
};
layout (std430, binding = 1) coherent buffer FragmentBufferVolumeBackFaces {
    LinkedListFragmentNode fragmentBufferVolumeBackFaces[];
};
layout (std430, binding = 2) coherent buffer FragmentBufferSurface {
    LinkedListFragmentNode fragmentBufferSurface[];
};

// Start-offset buffer (mapping pixels to first pixel in the buffer) of size viewportW*viewportH.
layout (std430, binding = 3) coherent buffer StartOffsetBufferVolumeFrontFaces {
    uint startOffsetVolumeFrontFaces[];
};
layout (std430, binding = 4) coherent buffer StartOffsetBufferVolumeBackFaces {
    uint startOffsetVolumeBackFaces[];
};
layout (std430, binding = 5) coherent buffer StartOffsetBufferSurface {
    uint startOffsetSurface[];
};

// Position of the first free fragment node in the linked list
layout(binding = 0, offset = 0) uniform atomic_uint fragCounterVolumeFrontFaces;
layout(binding = 1, offset = 0) uniform atomic_uint fragCounterVolumeBackFaces;
layout(binding = 2, offset = 0) uniform atomic_uint fragCounterSurface;

// Number of fragments we can store in total in each of the three lists
uniform uint linkedListVolumeFrontFacesCapacity;
uniform uint linkedListVolumeBackFacesCapacity;
uniform uint linkedListSurfaceCapacity;

uniform int viewportW;
//uniform int viewportH; // Not needed

#include "TiledAddress.glsl"
