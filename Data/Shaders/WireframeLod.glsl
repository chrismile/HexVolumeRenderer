-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in uint vertexLodValue;
flat out uint fragmentLodValue;

void main()
{
    fragmentLodValue = vertexLodValue;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Fragment

#version 430 core

flat in uint fragmentLodValue;
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
        colorValue = 1.0;
    }
    vec4 fragmentColor = vec4(colorValue, 0.0, 0.0, 1.0);
    //fragmentColor[idx] = colorValue;
    fragColor = fragmentColor;
}

