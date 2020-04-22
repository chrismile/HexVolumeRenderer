// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_shader_image_load_store.txt
#extension GL_ARB_shader_image_load_store : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests) in;

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNodeAttributeTextures {
    float importanceAttribute; ///< Between 0 and 1.
    float depth; ///< The linear depth of the fragment (i.e., distance to the camera).
    uint directionQuantized; ///< First 16 bits encode x, second 16 bits encode y.
    uint next; ///< Next entry index in the per-pixel linked list (or -1 == 0xFFFFFFFFu).
};

/**
 * Assumes the direction is normalized.
 */
void packLineDirection(in vec3 direction, out uint directionQuantized) {
    directionQuantized = min(65535u,  uint(direction.x * 65536.0f)) | (min(65535u,  uint(direction.y * 65536.0f)) << 16u);
}

void unpackLineDirection(in uint directionQuantized, out vec3 direction) {
    uint quantizedX = directionQuantized & 65535u;
    uint quantizedY = (directionQuantized >> 16u) & 65535u;
    direction.x = float(quantizedX) / 65535.0f;
    direction.y = float(quantizedY) / 65535.0f;
    float zLengthSquared = 1.0f - direction.x*direction.x - direction.y*direction.y;
    // Avoid NaN.
    if (zLengthSquared > 1e-8) {
        direction.z = sqrt(zLengthSquared);
    } else {
        direction.z = 0.0;
    }
}

// fragment-and-link buffer and a start-offset buffer

// Fragment-and-link buffer (linked list). Stores "nodesPerPixel" number of fragments.
layout (std430, binding = 0) coherent buffer FragmentBuffer {
    LinkedListFragmentNodeAttributeTextures fragmentBuffer[];
};

// Start-offset buffer (mapping pixels to first pixel in the buffer) of size viewportW*viewportH.
layout (std430, binding = 1) coherent buffer StartOffsetBuffer {
    uint startOffset[];
};

// Position of the first free fragment node in the linked list
layout(binding = 0, offset = 0) uniform atomic_uint fragCounter;

// Number of fragments we can store in total
uniform uint linkedListSize;

uniform int viewportW;
//uniform int viewportH; // Not needed

#include "TiledAddress.glsl"
