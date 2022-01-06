-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexLodValue;
layout(location = 2) in vec4 vertexColor;

out VertexData {
    vec3 linePosition;
    float lineLodValue;
    vec4 lineColor;
};

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineLodValue = vertexLodValue;
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
out float fragmentLodValue;
out vec4 fragmentColor;
out float quadCoords; // Between -1 and 1

in VertexData {
    vec3 linePosition;
    float lineLodValue;
    vec4 lineColor;
} v_in[];

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    float lineLodValue0 = v_in[0].lineLodValue;
    float lineLodValue1 = v_in[1].lineLodValue;
    vec4 lineColor0 = v_in[0].lineColor;
    vec4 lineColor1 = v_in[1].lineColor;

    float lineWidth0 = lineWidth * (1.3 * (1.0 - lineLodValue0) + 0.2);
    float lineWidth1 = lineWidth * (1.3 * (1.0 - lineLodValue1) + 0.2);

    vec3 right = normalize(v_in[1].linePosition - v_in[0].linePosition);
    vec3 quadNormal0 = normalize(cameraPosition - linePosition0);
    vec3 quadNormal1 = normalize(cameraPosition - linePosition1);
    vec3 vertexPosition;

    vec3 up0 = normalize(cross(quadNormal0, right));
    vec3 up1 = normalize(cross(quadNormal1, right));

    vertexPosition = linePosition0 - (lineWidth0 / 2.0) * up0;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue0;
    fragmentColor = lineColor0;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - (lineWidth1 / 2.0) * up1;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue1;
    fragmentColor = lineColor1;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + (lineWidth0 / 2.0) * up0;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue0;
    fragmentColor = lineColor0;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + (lineWidth1 / 2.0) * up1;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue1;
    fragmentColor = lineColor1;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

uniform vec3 focusPoint;
uniform float maxDistance;

in vec3 fragmentPositionWorld;
in float fragmentLodValue;
in vec4 fragmentColor;
in float quadCoords; // Between -1 and 1
out vec4 fragColor;

uniform vec3 cameraPosition;

const float LOD_EPSILON = 0.001;

void main() {
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;

    float distanceToFocus = length(focusPoint - fragmentPositionWorld);
    float maxLodRenderValue = clamp(1.0 - distanceToFocus / maxDistance, 0.0, 1.0);
    if (fragmentLodValue > maxLodRenderValue + LOD_EPSILON) {
        discard;
    }

    //float coverage = 1.0 - smoothstep(0.90, 1.0, abs(quadCoords));
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    //fragColor = vec4(fragmentColor.rgb, fragmentColor.a * coverage);

    float absCoords = abs(quadCoords);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    fragColor = vec4(mix(fragmentColor.rgb, vec3(1.0, 1.0, 1.0),
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, absCoords)),
            fragmentColor.a * coverage);
}

-- Fragment.Preview

#version 430 core

uniform float maxLod;

in vec3 fragmentPositionWorld;
in float fragmentLodValue;
in vec4 fragmentColor;
in float quadCoords; // Between -1 and 1
out vec4 fragColor;

uniform vec3 cameraPosition;

const float LOD_EPSILON = 0.001;

void main() {
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;

    if (fragmentLodValue > maxLod + LOD_EPSILON) {
        discard;
    }

    //float coverage = 1.0 - smoothstep(0.90, 1.0, abs(quadCoords));
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    //fragColor = vec4(fragmentColor.rgb, fragmentColor.a * coverage);

    float absCoords = abs(quadCoords);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    fragColor = vec4(
            mix(
                fragmentColor.rgb, vec3(1.0, 1.0, 1.0),
                smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, absCoords)),
            fragmentColor.a * coverage);
}
