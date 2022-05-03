-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "LinkedListVolumeHeader.glsl"

uint colorListVolumeFrontFaces[MAX_NUM_FRAGS_VOLUME];
float depthListVolumeFrontFaces[MAX_NUM_FRAGS_VOLUME];
uint colorListVolumeBackFaces[MAX_NUM_FRAGS_VOLUME];
float depthListVolumeBackFaces[MAX_NUM_FRAGS_VOLUME];
uint colorListSurfaces[MAX_NUM_FRAGS_SURFACE];
float depthListSurfaces[MAX_NUM_FRAGS_SURFACE];


out vec4 fragColor;

/**
 * The implementation of the GPU-optimized heap is based on the original code from the bachelor's thesis "Opacity-Based
 * Rendering of Lagrangian Particle rajectories in Met.3D" by Maximilian Bandle, Technische Universität München.
 * It was optimized for the use in volume rendering by Christoph Neuhauser, Technische Universität München.
 */
void swapFragsVolumeFrontFaces(uint i, uint j) {
    uint cTemp = colorListVolumeFrontFaces[i];
    colorListVolumeFrontFaces[i] = colorListVolumeFrontFaces[j];
    colorListVolumeFrontFaces[j] = cTemp;
    float dTemp = depthListVolumeFrontFaces[i];
    depthListVolumeFrontFaces[i] = depthListVolumeFrontFaces[j];
    depthListVolumeFrontFaces[j] = dTemp;
}

void minHeapSink4VolumeFrontFaces(uint x, uint fragsCountVolumeFrontFaces) {
    uint c, t; // Child, Tmp
    while ((t = 4 * x + 1) < fragsCountVolumeFrontFaces) {
        if (t + 1 < fragsCountVolumeFrontFaces && depthListVolumeFrontFaces[t] > depthListVolumeFrontFaces[t+1]) {
            // 1st vs 2nd
            c = t + 1;
        } else {
            c = t;
        }

        if (t + 2 < fragsCountVolumeFrontFaces && depthListVolumeFrontFaces[c] > depthListVolumeFrontFaces[t+2]) {
            // Smallest vs 3rd
            c = t + 2;
        }

        if (t + 3 < fragsCountVolumeFrontFaces && depthListVolumeFrontFaces[c] > depthListVolumeFrontFaces[t+3]) {
            // Smallest vs 3rd
            c = t + 3;
        }

        if (depthListVolumeFrontFaces[x] <= depthListVolumeFrontFaces[c]) {
            return;
        } else {
            swapFragsVolumeFrontFaces(x, c);
            x = c;
        }
    }
}

void getNextVolumeFrontFaceFragment(
        inout uint i_ff, in uint fragsCountVolumeFrontFaces, out vec4 color, out float depth) {
    minHeapSink4VolumeFrontFaces(0, fragsCountVolumeFrontFaces - i_ff++);
    color = unpackUnorm4x8(colorListVolumeFrontFaces[0]);
    depth = depthListVolumeFrontFaces[0];
    colorListVolumeFrontFaces[0] = colorListVolumeFrontFaces[fragsCountVolumeFrontFaces-i_ff];
    depthListVolumeFrontFaces[0] = depthListVolumeFrontFaces[fragsCountVolumeFrontFaces-i_ff];
}

void throwAwayNextVolumeFrontFaceFragment(
        inout uint i_ff, in uint fragsCountVolumeFrontFaces) {
    minHeapSink4VolumeFrontFaces(0, fragsCountVolumeFrontFaces - i_ff++);
    colorListVolumeFrontFaces[0] = colorListVolumeFrontFaces[fragsCountVolumeFrontFaces-i_ff];
    depthListVolumeFrontFaces[0] = depthListVolumeFrontFaces[fragsCountVolumeFrontFaces-i_ff];
}


void swapFragsVolumeBackFaces(uint i, uint j) {
    uint cTemp = colorListVolumeBackFaces[i];
    colorListVolumeBackFaces[i] = colorListVolumeBackFaces[j];
    colorListVolumeBackFaces[j] = cTemp;
    float dTemp = depthListVolumeBackFaces[i];
    depthListVolumeBackFaces[i] = depthListVolumeBackFaces[j];
    depthListVolumeBackFaces[j] = dTemp;
}

void minHeapSink4VolumeBackFaces(uint x, uint fragsCountVolumeBackFaces) {
    uint c, t; // Child, Tmp
    while ((t = 4 * x + 1) < fragsCountVolumeBackFaces) {
        if (t + 1 < fragsCountVolumeBackFaces && depthListVolumeBackFaces[t] > depthListVolumeBackFaces[t+1]) {
            // 1st vs 2nd
            c = t + 1;
        } else {
            c = t;
        }

        if (t + 2 < fragsCountVolumeBackFaces && depthListVolumeBackFaces[c] > depthListVolumeBackFaces[t+2]) {
            // Smallest vs 3rd
            c = t + 2;
        }

        if (t + 3 < fragsCountVolumeBackFaces && depthListVolumeBackFaces[c] > depthListVolumeBackFaces[t+3]) {
            // Smallest vs 3rd
            c = t + 3;
        }

        if (depthListVolumeBackFaces[x] <= depthListVolumeBackFaces[c]) {
            return;
        } else {
            swapFragsVolumeBackFaces(x, c);
            x = c;
        }
    }
}

void getNextVolumeBackFaceFragment(
        inout uint i_bf, in uint fragsCountVolumeBackFaces, out vec4 color, out float depth) {
    minHeapSink4VolumeBackFaces(0, fragsCountVolumeBackFaces - i_bf++);
    color = unpackUnorm4x8(colorListVolumeBackFaces[0]);
    depth = depthListVolumeBackFaces[0];
    colorListVolumeBackFaces[0] = colorListVolumeBackFaces[fragsCountVolumeBackFaces-i_bf];
    depthListVolumeBackFaces[0] = depthListVolumeBackFaces[fragsCountVolumeBackFaces-i_bf];
}


void swapFragsSurfaces(uint i, uint j) {
    uint cTemp = colorListSurfaces[i];
    colorListSurfaces[i] = colorListSurfaces[j];
    colorListSurfaces[j] = cTemp;
    float dTemp = depthListSurfaces[i];
    depthListSurfaces[i] = depthListSurfaces[j];
    depthListSurfaces[j] = dTemp;
}

void minHeapSink4Surface(uint x, uint fragsCountSurfaces) {
    uint c, t; // Child, Tmp
    while ((t = 4 * x + 1) < fragsCountSurfaces) {
        if (t + 1 < fragsCountSurfaces && depthListSurfaces[t] > depthListSurfaces[t+1]) {
            // 1st vs 2nd
            c = t + 1;
        } else {
            c = t;
        }

        if (t + 2 < fragsCountSurfaces && depthListSurfaces[c] > depthListSurfaces[t+2]) {
            // Smallest vs 3rd
            c = t + 2;
        }

        if (t + 3 < fragsCountSurfaces && depthListSurfaces[c] > depthListSurfaces[t+3]) {
            // Smallest vs 3rd
            c = t + 3;
        }

        if (depthListSurfaces[x] <= depthListSurfaces[c]) {
            return;
        } else {
            swapFragsSurfaces(x, c);
            x = c;
        }
    }
}

void getNextSurfaceFragment(
        inout uint i_s, in uint fragsCountSurfaces, out vec4 color, out float depth) {
    minHeapSink4Surface(0, fragsCountSurfaces - i_s++);
    color = unpackUnorm4x8(colorListSurfaces[0]);
    depth = depthListSurfaces[0];
    colorListSurfaces[0] = colorListSurfaces[fragsCountSurfaces-i_s];
    depthListSurfaces[0] = depthListSurfaces[fragsCountSurfaces-i_s];
}




vec4 frontToBackPQ_Volume(
        uint numFragsVolumeFrontFaces, uint numFragsVolumeBackFaces, uint numFragsSurfaces) {
    // Bring it to heap structure
    for (uint i = numFragsVolumeFrontFaces/4; i > 0; --i) {
        minHeapSink4VolumeFrontFaces(i, numFragsVolumeFrontFaces);
    }
    for (uint i = numFragsVolumeBackFaces/4; i > 0; --i) {
        minHeapSink4VolumeBackFaces(i, numFragsVolumeBackFaces);
    }
    for (uint i = numFragsSurfaces/4; i > 0; --i) {
        minHeapSink4Surface(i, numFragsSurfaces);
    }

    const float VOLUME_FACTOR = 255.0;
    const float INFINITY = 1e9;

    // Get first fragment
    uint i_ff = 0, i_bf = 0, i_s = 0;
    vec4 volumeFrontFaceColor, volumeBackFaceColor, surfaceColor;
    float volumeFrontFaceDepth = INFINITY, volumeBackFaceDepth = INFINITY, surfaceDepth = INFINITY;
    bool volumeFrontFaceFragmentLoaded = i_ff < numFragsVolumeFrontFaces;
    bool volumeBackFaceFragmentLoaded = i_bf < numFragsVolumeBackFaces;
    bool surfaceFragmentLoaded = i_s < numFragsSurfaces;

    if (i_bf < numFragsVolumeBackFaces) {
        getNextVolumeBackFaceFragment(i_bf, numFragsVolumeBackFaces, volumeBackFaceColor, volumeBackFaceDepth);
    }
    if (numFragsVolumeFrontFaces < numFragsVolumeBackFaces && numFragsVolumeBackFaces > 0) {
        // Inside of hexahedral cell -> front face is missing
        volumeFrontFaceFragmentLoaded = true;
        volumeFrontFaceColor = volumeBackFaceColor;
        volumeFrontFaceDepth = 0.0;
    } else if (i_ff < numFragsVolumeFrontFaces) {
        getNextVolumeFrontFaceFragment(i_ff, numFragsVolumeFrontFaces, volumeFrontFaceColor, volumeFrontFaceDepth);
    }
    if (i_s < numFragsSurfaces) {
        getNextSurfaceFragment(i_s, numFragsSurfaces, surfaceColor, surfaceDepth);
    }

    float accumDepth = 0.0, currLengthTraveled;

    // Start with transparent Ray
    vec4 rayColor = vec4(0.0);
    vec4 currentColor;
    while ((volumeFrontFaceFragmentLoaded || volumeBackFaceFragmentLoaded || surfaceFragmentLoaded)
            && rayColor.a < 0.99) {
        if (volumeFrontFaceFragmentLoaded ^^ volumeBackFaceFragmentLoaded) {
            // Make sure we don't have this rare corner case caused by unfortunate clipping.
            break;
        }

        if (volumeFrontFaceFragmentLoaded && volumeBackFaceFragmentLoaded && volumeFrontFaceDepth <= surfaceDepth) {
#if true
            // Volume
            vec4 volumeColor = (volumeFrontFaceColor + volumeFrontFaceColor) / 2.0;
            currLengthTraveled = volumeBackFaceDepth - volumeFrontFaceDepth;
            float volumeOpacityFactor = clamp(1.0 - exp(
                    -volumeColor.a * VOLUME_FACTOR * currLengthTraveled), 0.0, 1.0);
            currentColor = vec4(volumeColor.rgb, volumeOpacityFactor);
            accumDepth += currLengthTraveled;
#else
            // Volume
            currLengthTraveled = volumeBackFaceDepth - volumeFrontFaceDepth;
            float volumeOpacityFactor = clamp(1.0 - exp(
                    -volumeFrontFaceColor.a * VOLUME_FACTOR * currLengthTraveled), 0.0, 1.0);
            currentColor = vec4(volumeFrontFaceColor.rgb, volumeOpacityFactor);
            accumDepth += currLengthTraveled;
#endif

            if (i_ff < numFragsVolumeFrontFaces) {
                getNextVolumeFrontFaceFragment(i_ff, numFragsVolumeFrontFaces, volumeFrontFaceColor, volumeFrontFaceDepth);
                volumeFrontFaceFragmentLoaded = true;
            } else {
                volumeFrontFaceFragmentLoaded = false;
                volumeFrontFaceDepth = INFINITY;
            }
            if (i_bf < numFragsVolumeBackFaces) {
                getNextVolumeBackFaceFragment(i_bf, numFragsVolumeBackFaces, volumeBackFaceColor, volumeBackFaceDepth);
                volumeBackFaceFragmentLoaded = true;
            } else {
                volumeBackFaceFragmentLoaded = false;
                volumeBackFaceDepth = INFINITY;
            }
        } else if (surfaceFragmentLoaded && volumeFrontFaceDepth >= surfaceDepth){
            // Surface
            currentColor = surfaceColor;
            if (i_s < numFragsSurfaces) {
                getNextSurfaceFragment(i_s, numFragsSurfaces, surfaceColor, surfaceDepth);
                surfaceFragmentLoaded = true;
            } else {
                surfaceFragmentLoaded = false;
                surfaceDepth = INFINITY;
            }
        }

        // FTB Blending
        rayColor.rgb = rayColor.rgb + (1.0 - rayColor.a) * currentColor.a * currentColor.rgb;
        rayColor.a = rayColor.a + (1.0 - rayColor.a) * currentColor.a;
    }

    rayColor.rgb = rayColor.rgb / rayColor.a;
    return rayColor;
}

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array
    uint fragOffset;

#ifdef INITIALIZE_ARRAY_POW2
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        colorListVolumeFrontFaces[i] = 0;
        depthListVolumeFrontFaces[i] = 0.0;
    }
#endif

    // Collect all fragments for this pixel
    uint numFragsVolumeFrontFaces = 0, numFragsVolumeBackFaces = 0, numFragsSurfaces = 0;
    fragOffset = startOffsetVolumeFrontFaces[pixelIndex];
    for (uint i = 0; i < MAX_NUM_FRAGS_VOLUME; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

        LinkedListFragmentNode fragment = fragmentBufferVolumeFrontFaces[fragOffset];
        fragOffset = fragment.next;

        colorListVolumeFrontFaces[i] = fragment.color;
        depthListVolumeFrontFaces[i] = fragment.depth;

        numFragsVolumeFrontFaces++;
    }

    fragOffset = startOffsetVolumeBackFaces[pixelIndex];

#ifdef INITIALIZE_ARRAY_POW2
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        colorListVolumeBackFaces[i] = 0;
        depthListVolumeBackFaces[i] = 0.0;
    }
#endif

    for (uint i = 0; i < MAX_NUM_FRAGS_VOLUME; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

        LinkedListFragmentNode fragment = fragmentBufferVolumeBackFaces[fragOffset];
        fragOffset = fragment.next;

        colorListVolumeBackFaces[i] = fragment.color;
        depthListVolumeBackFaces[i] = fragment.depth;

        numFragsVolumeBackFaces++;
    }

    fragOffset = startOffsetSurface[pixelIndex];

#ifdef INITIALIZE_ARRAY_POW2
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        colorListSurfaces[i] = 0;
        depthListSurfaces[i] = 0.0;
    }
#endif

    for (uint i = 0; i < MAX_NUM_FRAGS_SURFACE; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

        LinkedListFragmentNode fragment = fragmentBufferSurface[fragOffset];
        fragOffset = fragment.next;

        colorListSurfaces[i] = fragment.color;
        depthListSurfaces[i] = fragment.depth;

        numFragsSurfaces++;
    }

    if (numFragsVolumeFrontFaces == 0 && numFragsVolumeBackFaces == 0 && numFragsSurfaces == 0) {
        discard;
    }

    fragColor = frontToBackPQ_Volume(numFragsVolumeFrontFaces, numFragsVolumeBackFaces, numFragsSurfaces);
}

