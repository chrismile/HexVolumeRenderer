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
    fragColor = vec4(vec3(1.0) - clearColor.rgb, smoothstep(0.01, 0.2, color.r) * 0.5);
    if (fragColor.r < 0.01) {
        discard;
    }
}

-- Fragment.DepthTexture

#version 430 core

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

uniform vec4 clearColor;

uniform sampler2D depthTexture;
uniform ivec2 depthTextureSize;

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
            vec2 textureCoordinates = (vec2(samplePos) + vec2(0.5, 0.5)) / vec2(depthTextureSize.xy);
            float depthBufferValue = texture(depthTexture, textureCoordinates).x;
            //float linearDepth = convertDepthBufferValueToLinearDepth(depthBufferValue);
            float linearDepth = convertDepthBufferValueToLinearDepth(depthBufferValue);
            accumulatedDepth += weight * linearDepth;

            if (linearDepth < minDepth) {
                minDepth = linearDepth;
            }
            if (linearDepth > maxDepth) {
                maxDepth = linearDepth;
            }
        }
    }
    float colValue = 0.0;
    if (accumulatedDepth > 0.0001) {
        colValue = 1.0;
    }
    //fragColor = vec4(vec3(accumulatedDepth, abs(accumulatedDepth), 0.0), 1.0);
    fragColor = vec4(vec3(smoothstep(0.005, 0.006, accumulatedDepth)), 1.0);
    fragColor = vec4(vec3(1.0) - clearColor.rgb, smoothstep(0.005, 0.006, accumulatedDepth) * 0.5);
    //fragColor = vec4(vec3(1.0) - clearColor.rgb, smoothstep(0.03, 0.04, maxDepth - minDepth) * 0.5);



    //fragColor = vec4(vec3(smoothstep(0.0, 0.0, accumulatedDepth * 100.0)), 1.0);
    //float depth = texture(depthTexture, (vec2(fragPos) + vec2(0.5, 0.5)) / vec2(depthTextureSize.xy)).x;//texelFetch(depthTexture, fragPos, 0).x;
    //fragColor = vec4(vec3(convertDepthBufferValueToLinearDepth(depth)), 1.0);
    /*fragColor = vec4(vec3(1.0) - clearColor.rgb, smoothstep(0.01, 1.0, accumulatedDepth));
    if (fragColor.r < 0.01) {
        discard;
    }*/
}
