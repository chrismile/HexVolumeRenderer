uniform float fieldOfViewY;
uniform ivec2 viewportSize;

float getAntialiasingFactor(float distance) {
    return distance / float(viewportSize.y) * fieldOfViewY;
}
