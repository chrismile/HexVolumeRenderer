-- Vertex.Plain

#version 430 core

in vec4 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.Plain

#version 430 core

void main() {
}


-- Vertex.PlainNormal

#version 430 core

in vec4 vertexPosition;
out vec3 fragmentPositionView;

void main() {
    fragmentPositionView = (vMatrix * mMatrix * vertexPosition).xyz;
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.PlainNormal

#version 430 core

in vec3 fragmentPositionView;
layout(location = 0) out vec3 fragNormal;

void main() {
    vec3 tangentX = dFdx(fragmentPositionView);
    vec3 tangentY = dFdy(fragmentPositionView);
    fragNormal = normalize(cross(tangentX, tangentY));
}
