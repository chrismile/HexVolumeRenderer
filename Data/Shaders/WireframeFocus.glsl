-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;

out VertexData {
    vec3 linePosition;
    vec4 lineColor;
};

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineColor = vertexColor;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float lineWidth;

out vec3 fragmentPositionWorld;
out float quadCoords; // Between -1 and 1
out vec4 fragmentColor;

in VertexData {
    vec3 linePosition;
    vec4 lineColor;
} v_in[];

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    vec4 lineColor0 = v_in[0].lineColor;
    vec4 lineColor1 = v_in[1].lineColor;

    const float EPSILON = 0.0001;
    vec3 lineDir = normalize(linePosition0 - linePosition1);
    linePosition0 = linePosition0 + EPSILON * lineDir;
    linePosition1 = linePosition1 - EPSILON * lineDir;

    vec3 right = normalize(v_in[1].linePosition - v_in[0].linePosition);
    vec3 quadNormal0 = normalize(cameraPosition - linePosition0);
    vec3 quadNormal1 = normalize(cameraPosition - linePosition1);
    vec3 vertexPosition;

    vec3 up0 = normalize(cross(quadNormal0, right));
    vec3 up1 = normalize(cross(quadNormal1, right));

    vertexPosition = linePosition0 - (lineWidth / 2.0) * up0;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor0;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - (lineWidth / 2.0) * up1;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor1;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + (lineWidth / 2.0) * up0;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor0;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + (lineWidth / 2.0) * up1;
    fragmentPositionWorld = vertexPosition;
    fragmentColor = lineColor1;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in vec4 fragmentColor;
in float quadCoords; // Between -1 and 1

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

#include "ClearView.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

void main() {
    float distanceToCenterPercentage = length(fragmentPositionWorld - sphereCenter) / sphereRadius;
    //vec3 lineColor = mix(fragmentColor.rgb, vec3(0.5, 0.5, 0.5), clamp(distanceToCenterPercentage*distanceToCenterPercentage, 0.0, 1.0));
    vec3 lineColor = fragmentColor.rgb;

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    float absCoords = abs(quadCoords);

    #if defined(LINE_RENDERING_STYLE_HALO)

    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, absCoords);
    vec4 color = vec4(mix(lineColor, vec3(1.0, 1.0, 1.0),
    smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, absCoords)), fragmentColor.a * coverage);

    // To counteract depth fighting with overlay wireframe.
    if (absCoords >= WHITE_THRESHOLD - EPSILON) {
        fragmentDepth += gl_FragCoord.z * 0.01;
    }

    #elif defined(LINE_RENDERING_STYLE_TRON)

    const float CORE_THRESHOLD = 0.4;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);

    const float GLOW_STRENGTH = 0.5;
    vec4 colorSolid = vec4(lineColor.rgb, 1.0 - smoothstep(CORE_THRESHOLD - EPSILON, CORE_THRESHOLD + EPSILON, absCoords));
    vec4 colorGlow = vec4(lineColor.rgb, GLOW_STRENGTH * (1.0 - smoothstep(0.0, 1.2, absCoords)));

    // Back-to-front blending:
    float a_out = colorGlow.a + colorSolid.a * (1.0 - colorGlow.a);
    vec3 c_out = (colorGlow.rgb * colorGlow.a + colorSolid.rgb * colorSolid.a) / a_out;
    vec4 color = vec4(c_out, a_out);

    #endif

    color.a *= getClearViewFocusFragmentOpacityFactor();

    #if defined(DIRECT_BLIT_GATHER)
    fragColor = color;
    #else
    gatherFragmentCustomDepth(color, fragmentDepth);
    #endif
}
