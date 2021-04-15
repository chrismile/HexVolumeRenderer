#ifdef USE_DEPTH_CUES
#ifdef COMPUTE_DEPTH_CUES_GPU
layout (std430, binding = 12) readonly buffer DepthMinMaxBuffer {
    float minDepth;
    float maxDepth;
};
#else
uniform float minDepth = 0.0f;
uniform float maxDepth = 1.0f;
#endif
uniform float depthCueStrength = 0.6f;
#endif

float computeDepthCueFactor() {
#ifdef USE_DEPTH_CUES
    vec4 screenSpacePosition = vMatrix * vec4(fragmentPositionWorld, 1.0);
    float depthCueFactor = (-screenSpacePosition.z - minDepth) / (maxDepth - minDepth);
    depthCueFactor = depthCueFactor * depthCueFactor * depthCueStrength;
    return depthCueFactor;
    //phongColor = mix(phongColor, vec3(0.5, 0.5, 0.5), depthCueFactor);
#else
    return 0.0f;
#endif
}
