-- Vertex

#version 430 core

in vec4 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment

#version 430 core

uniform sampler2D inputTexture;

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

// Uses simple anisotropic diffusion filtering.
void main() {
    const int NUM_NEIGHBORS = 8;
    const int NUM_DIRECTIONS = 4;

    ivec2 centerPosition = ivec2(gl_FragCoord.xy);

    // Load all pixel data of the 3x3 grid around our pixel position.
    vec4 centerColor = texelFetch(inputTexture, centerPosition, 0);
    const vec4 neighborColors[8] = {
        texelFetch(inputTexture, centerPosition + ivec2(-1, -1), 0),
        texelFetch(inputTexture, centerPosition + ivec2(0, -1), 0),
        texelFetch(inputTexture, centerPosition + ivec2(1, -1), 0),
        texelFetch(inputTexture, centerPosition + ivec2(-1, 0), 0),
        texelFetch(inputTexture, centerPosition + ivec2(1, 0), 0),
        texelFetch(inputTexture, centerPosition + ivec2(-1, 1), 0),
        texelFetch(inputTexture, centerPosition + ivec2(0, 1), 0),
        texelFetch(inputTexture, centerPosition + ivec2(1, 1), 0),
    };

    // Compute the derivatives in all directions.
    vec4 derivatives[NUM_DIRECTIONS];
    for (int direction = 0; direction < NUM_DIRECTIONS; direction++) {
        vec4 colorDirection0 = neighborColors[direction];
        vec4 colorDirection1 = neighborColors[NUM_NEIGHBORS - direction - 1];
        vec4 derivative = 2.0 * centerColor - colorDirection0 - colorDirection1;
        derivatives[direction] = derivative * derivative;
    }

    vec4 accumColor = centerColor;
    vec4 numAccumColors = vec4(1.0);
    for (int neighbor = 0; neighbor < NUM_NEIGHBORS; neighbor++) {
        // Compute the mean color of the neighbor and the center.
        vec4 neighborColor = neighborColors[neighbor];
        vec4 meanColor = 0.5 * (neighborColor + centerColor);

        ivec4 colorMask = ivec4(1, 1, 1, 1);
        for (int direction = 0; direction < NUM_DIRECTIONS; direction++) {
            // Compute the derivative using the mean color.
            vec4 colorDirection0 = neighborColors[direction];
            vec4 colorDirection1  = neighborColors[NUM_NEIGHBORS - direction - 1];
            vec4 derivativeInterpolated = 2.0 * meanColor - colorDirection0 - colorDirection1;
            derivativeInterpolated = derivativeInterpolated * derivativeInterpolated;

            // Is this likely to be noise based on the derivative using the mean value?
            colorMask = ivec4(lessThan(derivativeInterpolated, derivatives[direction])) & colorMask;
        }

        // Update the accumulated color & counter.
        accumColor += vec4(greaterThan(colorMask, ivec4(0))) * meanColor;
        numAccumColors += vec4(greaterThan(colorMask, ivec4(0)));
    }

    vec4 colorOut = accumColor / numAccumColors;
    fragColor = vec4(colorOut.rgb, centerColor.a); // For now use alpha of center...
}

-- Fragment.SingleChannel

#version 430 core

uniform sampler2D inputTexture;

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColor;

// Uses simple anisotropic diffusion filtering.
void main() {
    const int NUM_NEIGHBORS = 8;
    const int NUM_DIRECTIONS = 4;

    ivec2 centerPosition = ivec2(gl_FragCoord.xy);

    // Load all pixel data of the 3x3 grid around our pixel position.
    float centerColor = texelFetch(inputTexture, centerPosition, 0).r;
    const float neighborColors[8] = {
        texelFetch(inputTexture, centerPosition + ivec2(-1, -1), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(0, -1), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(1, -1), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(-1, 0), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(1, 0), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(-1, 1), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(0, 1), 0).r,
        texelFetch(inputTexture, centerPosition + ivec2(1, 1), 0).r,
    };

    // Compute the derivatives in all directions.
    float derivatives[NUM_DIRECTIONS];
    for (int direction = 0; direction < NUM_DIRECTIONS; direction++) {
        float colorDirection0 = neighborColors[direction];
        float colorDirection1 = neighborColors[NUM_NEIGHBORS - direction - 1];
        float derivative = 2.0 * centerColor - colorDirection0 - colorDirection1;
        derivatives[direction] = derivative * derivative;
    }

    float accumColor = centerColor;
    float numAccumColors = float(1.0);
    for (int neighbor = 0; neighbor < NUM_NEIGHBORS; neighbor++) {
        // Compute the mean color of the neighbor and the center.
        float neighborColor = neighborColors[neighbor];
        float meanColor = 0.5 * (neighborColor + centerColor);

        int colorMask = 1;
        for (int direction = 0; direction < NUM_DIRECTIONS; direction++) {
            // Compute the derivative using the mean color.
            float colorDirection0 = neighborColors[direction];
            float colorDirection1  = neighborColors[NUM_NEIGHBORS - direction - 1];
            float derivativeInterpolated = 2.0 * meanColor - colorDirection0 - colorDirection1;
            derivativeInterpolated = derivativeInterpolated * derivativeInterpolated;
            
            // Is this likely to be noise based on the derivative using the mean value?
            colorMask = int(derivativeInterpolated < derivatives[direction]) & colorMask;
        }
        
        // Update the accumulated color & counter.
        accumColor += float(colorMask > 0) * meanColor;
        numAccumColors += float(colorMask > 0);
    }

    float colorOut = accumColor / numAccumColors;
    fragColor = vec4(colorOut);
}
