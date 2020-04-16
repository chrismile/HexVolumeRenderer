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

#ifndef HEXVOLUMERENDERER_HEXAHEDRONVOLUME_HPP
#define HEXVOLUMERENDERER_HEXAHEDRONVOLUME_HPP

#include <vector>
#include <glm/vec3.hpp>

/**
 * Computes the determinant of the three passed 3x1 vectors building a 3x3 matrix.
 */
float det3(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3);

/**
 * "Efficient Computation of Volume of Hexahedral Cells", J. Grandy (1997)
 * URL: https://www.osti.gov/servlets/purl/632793 (retrieved 2020-03-31).
 *
 * This function computes the volume of a tetrakis hexahedron (TH).
 * It is assumed that we have the following cell point layout:
 *
 *     7 ---------------- 6
 *      /|             /|
 *     /              / |
 *    /  |           /  |
 * 4 ---------------- 5 |
 *   |   |          |   |
 *   |              |   |
 *   |   |          |   |
 *   | 3  - - - - - | - / 2
 *   | /            |  /
 *   |              | /
 * 0 ----------------/ 1
 *
 * @param v The eight corner cells of the tetrahedron.
 * @return The volume of the tetrakis hexahedron (TH).
 */
float computeHexahedralCellVolume_TetrakisHexahedron(const glm::vec3* v);

/**
 * "Efficient Computation of Volume of Hexahedral Cells", J. Grandy (1997)
 * URL: https://www.osti.gov/servlets/purl/632793 (retrieved 2020-03-31).
 *
 * This function computes the volume of a hexahedron using the long diagonal (LD) method.
 * It is assumed that we have the following cell point layout:
 *
 *     7 ---------------- 6
 *      /|             /|
 *     /              / |
 *    /  |           /  |
 * 4 ---------------- 5 |
 *   |   |          |   |
 *   |              |   |
 *   |   |          |   |
 *   | 3  - - - - - | - / 2
 *   | /            |  /
 *   |              | /
 * 0 ----------------/ 1
 *
 * @param v The eight corner cells of the tetrahedron.
 * @return The volume of the hexahedron using the long diagonal (LD) method.
 */
float computeHexahedralCellVolume_LongDiagonal(const glm::vec3* v);

/**
 * This function computes the area of a quadrilateral using the diagonal.
 * It is assumed that we have the following face point layout:
 *
 * 3 ---------------- 2
 *   |              |
 *   |              |
 *   |              |
 *   |              |
 *   |              |
 *   |              |
 * 0 ---------------- 1
 *
 * @param v The four corner cells of the quadrilateral.
 * @return The area of the quadrilateral using the diagonal.
 */
float computeQuadrilateralFaceArea_Diagonal(const glm::vec3* v);

/**
 * This function computes the area of a quadrilateral using the barycenter.
 * It is assumed that we have the following face point layout:
 *
 * 3 ---------------- 2
 *   |              |
 *   |              |
 *   |              |
 *   |              |
 *   |              |
 *   |              |
 * 0 ---------------- 1
 *
 * @param v The four corner cells of the quadrilateral.
 * @return The area of the quadrilateral using the barycenter.
 */
float computeQuadrilateralFaceArea_Barycenter(const glm::vec3* v);

#endif //HEXVOLUMERENDERER_HEXAHEDRONVOLUME_HPP
