-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexLodValue;

out VertexData
{
    vec3 linePosition;
    float lineLodValue;
};

void main()
{
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineLodValue = vertexLodValue;
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
out float quadCoords; // Between -1 and 1

in VertexData
{
    vec3 linePosition;
    float lineLodValue;
} v_in[];

void main()
{
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    float lineLodValue0 = v_in[0].lineLodValue;
    float lineLodValue1 = v_in[1].lineLodValue;

    vec3 right = normalize(v_in[1].linePosition - v_in[0].linePosition);
    vec3 quadNormal = normalize(cameraPosition - (linePosition0 + linePosition1) / 2.0);
    vec3 vertexPosition;

    vec3 up = cross(quadNormal, right);

    vertexPosition = linePosition0 - (lineWidth / 2.0) * up;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue0;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - (lineWidth / 2.0) * up;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue1;
    quadCoords = -1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + (lineWidth / 2.0) * up;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue0;
    quadCoords = 1.0;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + (lineWidth / 2.0) * up;
    fragmentPositionWorld = vertexPosition;
    fragmentLodValue = lineLodValue1;
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
in float quadCoords; // Between -1 and 1
out vec4 fragColor;

const float EPSILON = 0.001;

void main()
{
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    vec4 fragmentColor = vec4(0.0, 0.0, 0.0, 1.0);

    float distanceToFocus = length(focusPoint - fragmentPositionWorld);
    float maxLodRenderValue = clamp(1.0 - distanceToFocus / maxDistance, 0.0, 1.0);
    if (fragmentLodValue > maxLodRenderValue + EPSILON) {
        discard;
        //fragmentColor.a = 0.0;
    }

    //float coverage = 1.0 - smoothstep(0.90, 1.0, abs(quadCoords));
    float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    fragColor = vec4(fragmentColor.rgb, fragmentColor.a * coverage);

}

