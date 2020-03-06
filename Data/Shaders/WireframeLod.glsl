-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in uint vertexLodValue;

out VertexData
{
    vec3 linePosition;
    uint lineLodValue;
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

out float quadCoords; // Between -1 and 1
flat out uint fragmentLodValue;

in VertexData
{
    vec3 linePosition;
    uint lineLodValue;
} v_in[];

void main()
{
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    uint lineLodValue0 = v_in[0].lineLodValue;
    uint lineLodValue1 = v_in[1].lineLodValue;

    vec3 right = normalize(v_in[1].linePosition - v_in[0].linePosition);
    vec3 quadNormal = normalize(cameraPosition - (linePosition0 + linePosition1) / 2.0);
    vec3 vertexPosition;

    vec3 up = cross(quadNormal, right);

    vertexPosition = linePosition0 - (lineWidth / 2.0) * up;
    fragmentLodValue = lineLodValue0;
    quadCoords = -1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - (lineWidth / 2.0) * up;
    fragmentLodValue = lineLodValue1;
    quadCoords = -1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + (lineWidth / 2.0) * up;
    fragmentLodValue = lineLodValue0;
    quadCoords = 1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + (lineWidth / 2.0) * up;
    fragmentLodValue = lineLodValue1;
    quadCoords = 1;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

flat in uint fragmentLodValue;
in float quadCoords; // Between -1 and 1
out vec4 fragColor;

void main()
{
    // To counteract depth fighting with overlay wireframe.
    gl_FragDepth = gl_FragCoord.z - 0.00001;
    //float scaledLod = mod(fragmentLodValue, 1000.0);
    //float colorValue = scaledLod / 1000.0;
    //int idx = int(scaledLod);
    float colorValue = 0.0;
    if (fragmentLodValue < 100) {
        uint direction = (fragmentLodValue/2) % 3;
        if (direction == 0) {
            colorValue = 0.2;
        }
        if (direction == 1) {
            colorValue = 0.6;
        }
        if (direction == 2) {
            colorValue = 1.0;
        }
    }
    vec4 fragmentColor = vec4(colorValue, 0.0, 0.0, 1.0);
    //fragmentColor[idx] = colorValue;

    //float coverage = 1.0 - smoothstep(0.90, 1.0, abs(quadCoords));
    float coverage = 1.0 - smoothstep(1.0, 1.0, abs(quadCoords));
    fragColor = vec4(fragmentColor.rgb, fragmentColor.a * coverage);

}

