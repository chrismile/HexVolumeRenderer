-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexColor;

out VertexData
{
    vec3 pointPosition;
    vec4 pointColor;
};

void main()
{
    pointPosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    pointColor = vertexColor;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Geometry

#version 430 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float radius;

out vec2 quadCoords; // Between -1 and 1
out vec4 fragmentColor;

in VertexData
{
    vec3 pointPosition;
    vec4 pointColor;
} v_in[];

void main()
{
    vec3 pointPosition = v_in[0].pointPosition;
    vec4 pointColor = v_in[0].pointColor;

    vec3 quadNormal = normalize(cameraPosition - pointPosition);
    vec3 vertexPosition;

    vec3 right = cross(quadNormal, vec3(0, 1, 0));
    vec3 top = cross(quadNormal, right);

    vertexPosition = pointPosition + radius * (right - top);
    fragmentColor = pointColor;
    quadCoords = vec2(1, -1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + radius * (right + top);
    fragmentColor = pointColor;
    quadCoords = vec2(1, 1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + radius * (-right - top);
    fragmentColor = pointColor;
    quadCoords = vec2(-1, -1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + radius * (-right + top);
    fragmentColor = pointColor;
    quadCoords = vec2(-1, 1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

in vec2 quadCoords; // Between -1 and 1
in vec4 fragmentColor;
out vec4 fragColor;

void main()
{
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.0002;
    float coverage = 1.0 - smoothstep(0.95, 1.0, length(quadCoords));
    fragColor = vec4(fragmentColor.rgb, fragmentColor.a * coverage);
}

