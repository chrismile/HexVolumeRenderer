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

#include <glm/glm.hpp>
#include "HexahedronVolume.hpp"

float det3(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3) {
    return v1.x*v2.y*v3.z + v2.x*v3.y*v1.z + v3.x*v1.y*v2.z
           - v3.x*v2.y*v1.z - v1.x*v3.y*v2.z - v2.x*v1.y*v3.z;
}

float computeHexahedralCellVolume_TetrakisHexahedron(const glm::vec3* v) {
    return 1.0f / 12.0f *
           det3((v[6] - v[1]) + (v[7] - v[0]), (v[6] - v[2]), (v[2] - v[0]))
           + det3((v[7] - v[0]), (v[6] - v[3]) + (v[5] - v[0]), (v[6] - v[4]))
           + det3((v[6] - v[1]), (v[5] - v[0]), (v[6] - v[4]) + (v[2] - v[0]));
}

float computeHexahedralCellVolume_LongDiagonal(const glm::vec3* v) {
    return 1.0f / 6.0f *
           det3((v[6] - v[0]), (v[1] - v[0]), (v[2] - v[5]))
           + det3((v[6] - v[0]), (v[4] - v[0]), (v[5] - v[6]))
           + det3((v[6] - v[0]), (v[3] - v[0]), (v[6] - v[2]));
}

float computeQuadrilateralFaceArea_Diagonal(const glm::vec3* v) {
    return 0.5f * (
            glm::length(glm::cross(v[1] - v[0], v[3] - v[0])) +
            glm::length(glm::cross(v[3] - v[2], v[1] - v[2])));
}

float computeQuadrilateralFaceArea_Barycenter(const glm::vec3* v) {
    glm::vec3 barycenter = 0.25f * (v[0] + v[1] + v[2] + v[3]);
    return 0.5f * (
            glm::length(glm::cross(v[0] - barycenter, v[1] - barycenter)) +
            glm::length(glm::cross(v[1] - barycenter, v[2] - barycenter)) +
            glm::length(glm::cross(v[2] - barycenter, v[3] - barycenter)) +
            glm::length(glm::cross(v[3] - barycenter, v[0] - barycenter)));
}
