-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;

out VertexData
{
    vec3 linePosition;
    vec4 lineColor;
};

void main()
{
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

out float quadCoords; // Between -1 and 1
out vec4 fragmentColor;

in VertexData
{
    vec3 linePosition;
    vec4 lineColor;
} v_in[];

void main()
{
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    vec4 lineColor0 = v_in[0].lineColor;
    vec4 lineColor1 = v_in[1].lineColor;

    vec3 right = normalize(v_in[1].linePosition - v_in[0].linePosition);
    vec3 quadNormal = normalize(cameraPosition - (linePosition0 + linePosition1) / 2.0);
    vec3 vertexPosition;

    vec3 up = cross(quadNormal, right);

    vertexPosition = linePosition0 - (lineWidth / 2.0) * up;
    fragmentColor = lineColor0;
    quadCoords = -1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - (lineWidth / 2.0) * up;
    fragmentColor = lineColor1;
    quadCoords = -1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + (lineWidth / 2.0) * up;
    fragmentColor = lineColor0;
    quadCoords = 1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + (lineWidth / 2.0) * up;
    fragmentColor = lineColor1;
    quadCoords = 1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

in vec4 fragmentColor;
in float quadCoords; // Between -1 and 1
out vec4 fragColor;

void main()
{
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    //float coverage = 1.0 - smoothstep(0.90, 1.0, abs(quadCoords));
    float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    fragColor = vec4(fragmentColor.rgb, fragmentColor.a * coverage);
}

