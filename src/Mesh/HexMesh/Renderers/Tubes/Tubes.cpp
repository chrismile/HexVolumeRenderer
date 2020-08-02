/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Math/Math.hpp>
#include "Tubes.hpp"

float globalTubeRadius = 0.0f;
std::vector<glm::vec3> globalCircleVertexPositions;

void initGlobalCircleVertexPositions(int numCircleSubdivisions, float tubeRadius) {
    globalCircleVertexPositions.clear();
    globalTubeRadius = tubeRadius;

    const float theta = sgl::TWO_PI / numCircleSubdivisions;
    const float tangentialFactor = std::tan(theta); // opposite / adjacent
    const float radialFactor = std::cos(theta); // adjacent / hypotenuse
    glm::vec3 position(tubeRadius, 0, 0);

    for (int i = 0; i < numCircleSubdivisions; i++) {
        globalCircleVertexPositions.push_back(position);

        // Add the tangent vector and correct the position using the radial factor.
        glm::vec3 tangent(-position.y, position.x, 0);
        position += tangentialFactor * tangent;
        position *= radialFactor;
    }
}

void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& normal, glm::vec3& lastTangent,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals) {
    glm::vec3 helperAxis = lastTangent;
    if (glm::length(glm::cross(helperAxis, normal)) < 0.01f) {
        // If normal == lastTangent
        helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
        if (glm::length(glm::cross(helperAxis, normal)) < 0.01f) {
            // If normal == helperAxis
            helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
        }
    }
    glm::vec3 tangent = glm::normalize(helperAxis - glm::dot(helperAxis, normal) * normal); // Gram-Schmidt
    lastTangent = tangent;
    glm::vec3 binormal = glm::cross(normal, tangent);

    for (size_t i = 0; i < globalCircleVertexPositions.size(); i++) {
        glm::vec3 pt = globalCircleVertexPositions.at(i);
        glm::vec3 transformedPoint(
                pt.x * tangent.x + pt.y * binormal.x + pt.z * normal.x + center.x,
                pt.x * tangent.y + pt.y * binormal.y + pt.z * normal.y + center.y,
                pt.x * tangent.z + pt.y * binormal.z + pt.z * normal.z + center.z
        );
        vertexPositions.push_back(transformedPoint);

        glm::vec3 vertexNormal = glm::normalize(transformedPoint - center);
        vertexNormals.push_back(vertexNormal);
    }
}

void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& normal, glm::vec3& lastTangent,
        std::vector<glm::vec3>& vertexPositions) {
    glm::vec3 helperAxis = lastTangent;
    if (glm::length(glm::cross(helperAxis, normal)) < 0.01f) {
        // If normal == lastTangent
        helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
        if (glm::length(glm::cross(helperAxis, normal)) < 0.01f) {
            // If normal == helperAxis
            helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
        }
    }
    glm::vec3 tangent = glm::normalize(helperAxis - glm::dot(helperAxis, normal) * normal); // Gram-Schmidt
    lastTangent = tangent;
    glm::vec3 binormal = glm::cross(normal, tangent);

    for (size_t i = 0; i < globalCircleVertexPositions.size(); i++) {
        glm::vec3 pt = globalCircleVertexPositions.at(i);
        glm::vec3 transformedPoint(
                pt.x * tangent.x + pt.y * binormal.x + pt.z * normal.x + center.x,
                pt.x * tangent.y + pt.y * binormal.y + pt.z * normal.y + center.y,
                pt.x * tangent.z + pt.y * binormal.z + pt.z * normal.z + center.z
        );
        vertexPositions.push_back(transformedPoint);
    }
}
