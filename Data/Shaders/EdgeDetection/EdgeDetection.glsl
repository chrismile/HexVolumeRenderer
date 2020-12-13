-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Fragment.ColorTexture

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform vec4 clearColor;

uniform sampler2D imageTexture;
uniform ivec2 imageTextureSize;

uniform sampler2D weightTexture;
uniform ivec2 weightTextureSize;

void main() {
    vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0).x;
            ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2 + x, -weightTextureSize.y/2 + y);
            vec2 textureCoordinates = samplePos / vec2(imageTextureSize.xy);
            vec4 colorSample = texture(imageTexture, textureCoordinates);
            color += weight * colorSample;
        }
    }
    float intensity = smoothstep(0.01, 0.2, color.r);

#ifdef DIRECT_BLIT_OUTPUT
    fragColor = vec4(intensity, intensity, intensity, 1.0);
#else
    fragColor = vec4(vec3(1.0) - clearColor.rgb, intensity * 0.5);
#endif
    if (fragColor.a < 0.01) {
        discard;
    }
}

-- Fragment.DepthTexture

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform vec4 clearColor;

uniform sampler2D depthTexture;
uniform ivec2 viewportSize;

uniform sampler2D weightTexture;
uniform ivec2 weightTextureSize;

uniform float zNear;
uniform float zFar;

#include "DepthHelper.glsl"

void main() {
    float accumulatedDepth = 0.0;
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    float maxDepth = -1e9;
    float minDepth = 1e9;
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0).x;
            ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2 + x, -weightTextureSize.y/2 + y);
            vec2 textureCoordinates = (vec2(samplePos) + vec2(0.5, 0.5)) / vec2(viewportSize.xy);
            float depthBufferValue = texture(depthTexture, textureCoordinates).x;
            float linearDepth = convertDepthBufferValueToLinearDepth(depthBufferValue);
            accumulatedDepth += weight * linearDepth;
        }
    }
    float intensity = smoothstep(0.005, 0.006, abs(accumulatedDepth));

#ifdef DIRECT_BLIT_OUTPUT
    fragColor = vec4(intensity, intensity, intensity, 1.0);
#else
    fragColor = vec4(vec3(1.0) - clearColor.rgb, intensity * 0.5);
#endif
    if (fragColor.a < 1e-4) {
        discard;
    }
}

-- Fragment.NormalTexture

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform vec4 clearColor;

uniform sampler2D normalTexture;
uniform ivec2 viewportSize;

uniform sampler2D weightTexture;
uniform ivec2 weightTextureSize;

void main() {
    vec3 accumulatedNormal = vec3(0.0);
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    float maxDepth = -1e9;
    float minDepth = 1e9;
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0).x;
            ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2 + x, -weightTextureSize.y/2 + y);
            vec2 textureCoordinates = (vec2(samplePos) + vec2(0.5, 0.5)) / vec2(viewportSize.xy);
            vec3 normal = texture(normalTexture, textureCoordinates).xyz;
            accumulatedNormal += weight * normal;
        }
    }
    float accumulatedNormalLength = length(accumulatedNormal);
    float intensity = smoothstep(0.2, 0.35, accumulatedNormalLength);

#ifdef DIRECT_BLIT_OUTPUT
    fragColor = vec4(intensity, intensity, intensity, 1.0);
#else
    fragColor = vec4(vec3(1.0) - clearColor.rgb, intensity * 0.5);
#endif
    if (fragColor.a < 1e-4) {
        discard;
    }
}

-- Fragment.DepthNormalTexture

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform vec4 clearColor;

uniform sampler2D depthTexture;
uniform sampler2D normalTexture;
uniform ivec2 viewportSize;

uniform sampler2D weightTexture;
uniform ivec2 weightTextureSize;

uniform float zNear;
uniform float zFar;

#include "DepthHelper.glsl"

void main() {
    float accumulatedDepth = 0.0;
    vec3 accumulatedNormal = vec3(0.0);
    ivec2 fragPos = ivec2(gl_FragCoord.xy);
    float maxDepth = -1e9;
    float minDepth = 1e9;
    for (int y = 0; y < weightTextureSize.y; y++) {
        for (int x = 0; x < weightTextureSize.x; x++) {
            float weight = texelFetch(weightTexture, ivec2(x, y), 0).x;
            ivec2 samplePos = fragPos + ivec2(-weightTextureSize.x/2 + x, -weightTextureSize.y/2 + y);
            vec2 textureCoordinates = (vec2(samplePos) + vec2(0.5, 0.5)) / vec2(viewportSize.xy);
            float depthBufferValue = texture(depthTexture, textureCoordinates).x;
            float linearDepth = convertDepthBufferValueToLinearDepth(depthBufferValue);
            accumulatedDepth += weight * linearDepth;
            vec3 normal = texture(normalTexture, textureCoordinates).xyz;
            accumulatedNormal += weight * normal;
        }
    }
    float depthFactor = smoothstep(0.005, 0.006, abs(accumulatedDepth));
    float accumulatedNormalLength = length(accumulatedNormal);
    float normalFactor = smoothstep(0.2, 0.35, accumulatedNormalLength);
    float intensity = max(depthFactor, normalFactor);

#ifdef DIRECT_BLIT_OUTPUT
    fragColor = vec4(intensity, intensity, intensity, 1.0);
#else
    fragColor = vec4(vec3(1.0) - clearColor.rgb, intensity * 0.5);
#endif
    if (fragColor.a < 1e-4) {
        discard;
    }
}
