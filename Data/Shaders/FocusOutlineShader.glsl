-- Vertex

#version 430 core

uniform vec2 circleScreenPosition;
uniform float circleScreenRadius;
uniform float circlePixelThickness;
uniform ivec2 viewportSize;

out float texCoord;

const float PI = 3.1415926535897932f;

void main() {
    const int numCircleSubdivisions = 1024;
    float angle = 2.0f * float(gl_VertexID / 2) / float(numCircleSubdivisions) * PI;
    vec2 basePos = vec2(cos(angle), sin(angle));
    float shiftSign = gl_VertexID % 2 == 0 ? -1.0 : 1.0;
    vec2 screenPosition =
            circleScreenPosition + basePos * circleScreenRadius + shiftSign * circlePixelThickness * 0.5 * basePos;
    vec2 fragmentPositionNdc = 2.0 * screenPosition.xy / vec2(viewportSize) - vec2(1.0);
    texCoord = shiftSign;
    gl_Position = vec4(fragmentPositionNdc, 0.0, 1.0);
}

-- Fragment

#version 430 core

uniform vec4 focusOutlineColor;
uniform float circleDepth;

in float texCoord;

#if !defined(DIRECT_BLIT_GATHER)
vec3 fragmentPositionWorld;
vec3 cameraPosition;
#include OIT_GATHER_HEADER
#endif

void main() {
    vec4 fragmentColor = focusOutlineColor;
    float dst = abs(texCoord);
    float aaf = fwidth(dst);
    float alphaFactor = 1.0 - smoothstep(1.0 - aaf, 1.0, dst);
    fragmentColor.a *= alphaFactor;
    if (circleDepth >= -1e-8) {
        gatherFragmentCustomDepth(fragmentColor, circleDepth);
    }
}

