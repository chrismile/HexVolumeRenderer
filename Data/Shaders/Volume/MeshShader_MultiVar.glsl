-- Vertex.Attribute

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute0;
layout(location = 2) in float vertexAttribute1;

out vec3 fragmentPositionWorld;
out float fragmentAttribute0;
out float fragmentAttribute1;

//#include "TransferFunction.glsl"

void main()
{
    fragmentAttribute0 = vertexAttribute0;
    fragmentAttribute1 = vertexAttribute1;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in float fragmentAttribute0;
in float fragmentAttribute1;

#include "TransferFunction.glsl"

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition; // in world space

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif
vec3 sRGBToLinearRGB(vec3 u) {
    return mix(pow((u + 0.055) / 1.055, vec3(2.4)), u / 12.92, lessThanEqual(u, vec3(0.04045)));
}

void main() {
    //vec4 fragmentColor = vec4(fragmentAttribute0, fragmentAttribute1, 0.0, opacity * max(fragmentAttribute0, fragmentAttribute1));

    /*float val;
    if (opacity > 0.5) {
        val = fragmentAttribute0 / (fragmentAttribute0 + fragmentAttribute1 + 1e-10);
    } else {
        val = fragmentAttribute1 / (fragmentAttribute0 + fragmentAttribute1 + 1e-10);
    }
    vec4 fragmentColor = transferFunction(val);*/

    const vec4 C_MAP[4] = {
    vec4(1.0, 1.0, 1.0, 0.0), vec4(1.0, 1.0, 0.13286832155381798, 1.0),
    vec4(1.0, 0.17464740365558504, 1.0, 1.0), vec4(1.0, 0.0, 0.0, 1.0),
    };

    const float x = clamp(fragmentAttribute0, 0.0, 1.0);
    const float y = clamp(fragmentAttribute1 * 6.0, 0.0, 1.0);
    //float x = clamp(fragmentAttribute0 * 3.0, 0.0, 1.0);
    //float y = clamp(fragmentAttribute1 * 3.0, 0.0, 1.0);
    //x = 1.0 - pow(1.0 - x, 1.8);
    //y = 1.0 - pow(1.0 - y, 1.8);

    const vec4 c_x0 = x * C_MAP[1] + (1.0 - x) * C_MAP[0];
    const vec4 c_x1 = x * C_MAP[3] + (1.0 - x) * C_MAP[2];
    const vec4 c_f = y * c_x1 + (1.0 - y) * c_x0;

    float opacityOut = transferFunction(c_f.a).a;
    //float opacityOut = transferFunction(max(fragmentAttribute0, fragmentAttribute1)).a;

    const vec4 fragmentColor = vec4(sRGBToLinearRGB(vec3(c_f.r, c_f.g, c_f.b)), opacityOut);

#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = vec4(fragmentColor.rgb, 1.0);
#else
    gatherFragment(fragmentColor);
#endif
}
