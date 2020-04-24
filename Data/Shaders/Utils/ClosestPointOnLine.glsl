/**
 * Computes the closest point on a line.
 * See: http://geomalgorithms.com/a02-_lines.html
 *
 * @param p The position of the point.
 * @param l0 The first line point.
 * @param l1 The second line point.
 * @return The closest point on the line.
 */
vec3 getClosestPointOnLine(vec3 p, vec3 l0, vec3 l1) {
    vec3 v = l1 - l0;
    vec3 w = p - l0;
    float c1 = dot(v, w);
    float c2 = dot(v, v);

    float b = c1 / c2;
    vec3 pb = l0 + b * v;
    return pb;
}

/**
 * Computes the closest point on a line segment.
 * See: http://geomalgorithms.com/a02-_lines.html
 *
 * @param p The position of the point.
 * @param l0 The first line point.
 * @param l1 The second line point.
 * @return The closest point on the line segment.
 */
vec3 getClosestPointOnLineSegment(vec3 p, vec3 l0, vec3 l1) {
    vec3 v = l1 - l0;
    vec3 w = p - l0;
    float c1 = dot(v, w);
    if (c1 <= 0.0) {
        return l0;
    }

    float c2 = dot(v, v);
    if (c2 <= c1) {
        return l1;
    }

    float b = c1 / c2;
    vec3 pb = l0 + b * v;
    return pb;
}
