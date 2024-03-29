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
#ifdef DEPTH_TYPE_UINT
    // RGB color of the node (3x10 bit).
    uint color;
    // Depth (22 bit) and alpha (10 bit).
    uint depth;
#else
    // RGBA color of the node.
    uint color;
    // Depth value of the fragment (in view space).
    float depth;
#endif
    // The index of the next node in "nodes" array.
    uint next;
};

// fragment-and-link buffer and a start-offset buffer

// Fragment-and-link buffer (linked list). Stores "nodesPerPixel" number of fragments.
#ifdef FRAGMENT_BUFFER_ARRAY
layout (std430, binding = 1) coherent buffer FragmentBuffer {
    LinkedListFragmentNode fragmentBuffer[];
} fragmentBuffers[NUM_FRAGMENT_BUFFERS];
#else
layout (std430, binding = 1) coherent buffer FragmentBuffer {
    LinkedListFragmentNode fragmentBuffer[];
};
#endif

// Start-offset buffer (mapping pixels to first pixel in the buffer) of size viewportW*viewportH.
layout (std430, binding = 0) coherent buffer StartOffsetBuffer {
    uint startOffset[];
};

// Position of the first free fragment node in the linked list
layout(binding = 0, offset = 0) uniform atomic_uint fragCounter;

// Number of fragments we can store in total
uniform uint linkedListSize;

uniform int viewportW;
//uniform int viewportH; // Not needed

#include "TiledAddress.glsl"
