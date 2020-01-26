-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec4 vertexColor;

out vec3 fragmentPositonWorld;
out vec3 fragmentNormal;
out vec4 fragmentColor;

void main()
{
    fragmentNormal = vertexNormal;
    fragmentColor = vertexColor;
    fragmentPositonWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

in vec3 fragmentPositonWorld;
in vec3 fragmentNormal;
in vec4 fragmentColor;

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 lightDirection = vec3(1.0, 0.0, 0.0);
uniform vec3 cameraPosition; // in world space

void main()
{
    // Blinn-Phong Shading
    const vec3 lightColor = vec3(1,1,1);
    const vec3 ambientColor = fragmentColor.rgb;//vec3(0.5, fragmentPositonWorld.g, fragmentPositonWorld.b);
    const vec3 diffuseColor = ambientColor;
    vec3 phongColor = vec3(0);

    const float kA = 0.2;
    const vec3 Ia = kA * ambientColor;
    const float kD = 0.7;
    const float kS = 0.1;
    const float s = 10;

    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositonWorld);
    const vec3 l = v;//normalize(lightDirection);
    const vec3 h = normalize(v + l);

    vec3 Id = kD * clamp(abs(dot(n, l)), 0.0, 1.0) * diffuseColor;
    vec3 Is = kS * pow(clamp(abs(dot(n, h)), 0.0, 1.0), s) * lightColor;

    phongColor = Ia + Id + Is;

    float tempAlpha = fragmentColor.a;
    vec4 color = vec4(phongColor, tempAlpha);


#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering
    fragColor = vec4(color.rgb, 1.0);
#else
    gatherFragment(color);
#endif
}
