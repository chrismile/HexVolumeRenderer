/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#include <cassert>

#include <Utils/File/LineReader.hpp>

#include "DegStressLoader.hpp"

#ifdef USE_EIGEN
#include <Eigen/Eigenvalues>

void computePrincipalStresses(
        float xx, float yy, float zz, float xy, float yz, float zx,
        float& majorStress, float& mediumStress, float& minorStress/*,
        glm::vec3& v0, glm::vec3& v1, glm::vec3& v2*/) {
    Eigen::Matrix3f stressTensor;
    stressTensor.row(0) << xx, xy, zx;
    stressTensor.row(1) << xy, yy, yz;
    stressTensor.row(2) << zx, yz, zz;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> selfAdjointEigenSolver;
    selfAdjointEigenSolver.compute(stressTensor);
    Eigen::Vector3f eigenvalues = selfAdjointEigenSolver.eigenvalues();
    /*Eigen::Matrix3f eigenvectors = selfAdjointEigenSolver.eigenvectors();
    v0 = glm::vec3(eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0));
    v1 = glm::vec3(eigenvectors(0, 1), eigenvectors(1, 1), eigenvectors(2, 1));
    v2 = glm::vec3(eigenvectors(0, 2), eigenvectors(1, 2), eigenvectors(2, 2));*/

    minorStress = eigenvalues(0);
    mediumStress = eigenvalues(1);
    majorStress = eigenvalues(2);
}

float computeDegeneracyMeasure(float sigma1, float sigma2, float sigma3) {
    float degeneracyMeasure = 1.0f - std::abs((sigma1 - sigma2) / (sigma1 + sigma2));
    degeneracyMeasure = std::max(degeneracyMeasure, 1.0f - std::abs((sigma3 - sigma2) / (sigma3 + sigma2)));
    return degeneracyMeasure;
}
#endif

bool DegStressLoader::loadHexahedralMeshFromFile(
        const std::string& filename,
        std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices,
        std::vector<glm::vec3>& deformations, std::vector<float>& attributeList,
        bool& isPerVertexData) {
    sgl::LineReader lineReader(filename);

    auto verticesLine = lineReader.readVectorLine<std::string>();
    if (verticesLine.size() != 2 || verticesLine.at(0) != "#Vertices:") {
        sgl::Logfile::get()->throwError(
                "Error in DegStressLoader::loadHexahedralMeshFromFile: Invalid vertices statement.");
    }
    auto numVertices = sgl::fromString<uint32_t>(verticesLine.at(1));
    vertices.reserve(numVertices);
    std::vector<float> vertexLine;
    vertexLine.reserve(3);
    for (uint32_t vertexIdx = 0; vertexIdx < numVertices; vertexIdx++) {
        lineReader.readVectorLine<float>(vertexLine);
        if (vertexLine.size() != 3) {
            sgl::Logfile::get()->throwError(
                    "Error in DegStressLoader::loadHexahedralMeshFromFile: Invalid vertex line.");
        }
        vertices.emplace_back(glm::vec3(vertexLine.at(0), vertexLine.at(1), vertexLine.at(2)));
    }

    auto elementsLine = lineReader.readVectorLine<std::string>();
    if (elementsLine.size() != 2 || elementsLine.at(0) != "#Elements:") {
        sgl::Logfile::get()->throwError(
                "Error in DegStressLoader::loadHexahedralMeshFromFile: Invalid elements statement.");
    }
    auto numCells = sgl::fromString<uint32_t>(elementsLine.at(1));
    cellIndices.reserve(numCells);
    std::vector<uint32_t> elementLine;
    elementLine.reserve(8);
    for (uint32_t elementIdx = 0; elementIdx < numCells; elementIdx++) {
        lineReader.readVectorLine<uint32_t>(elementLine);
        if (elementLine.size() != 8) {
            sgl::Logfile::get()->throwError(
                    "Error in DegStressLoader::loadHexahedralMeshFromFile: Invalid element line.");
        }
        for (int i = 0; i < 8; i++) {
            cellIndices.emplace_back(elementLine.at(i) - 1);
        }
    }

    auto cartesianStressLine = lineReader.readVectorLine<std::string>();
    if (cartesianStressLine.size() != 3 || cartesianStressLine.at(0) != "Cartesian"
            || cartesianStressLine.at(1) != "Stress:") {
        sgl::Logfile::get()->throwError(
                "Error in DegStressLoader::loadHexahedralMeshFromFile: Invalid Cartesian Stress statement.");
    }
    auto numCartesianStresses = sgl::fromString<uint32_t>(cartesianStressLine.at(2));
    if (numCartesianStresses != numVertices) {
        sgl::Logfile::get()->throwError(
                "Error in DegStressLoader::loadHexahedralMeshFromFile: Number of vertices and Cartesian stresses "
                "does not match.");
    }
    attributeList.reserve(numVertices);

    float majorStress, mediumStress, minorStress;
    int xxIdx = 0;
    int yyIdx = 1;
    int zzIdx = 2;
    int xyIdx = 5;
    int yzIdx = 4;
    int zxIdx = 3;

    std::vector<float> cartesianStressesLine;
    cartesianStressesLine.reserve(6);
    for (uint32_t vertexIdx = 0; vertexIdx < numVertices; vertexIdx++) {
        lineReader.readVectorLine<float>(cartesianStressesLine);
        if (cartesianStressesLine.size() != 6) {
            sgl::Logfile::get()->throwError(
                    "Error in DegStressLoader::loadHexahedralMeshFromFile: Invalid Cartesian stresses line.");
        }
#ifdef USE_EIGEN
        computePrincipalStresses(
                cartesianStressesLine.at(xxIdx),
                cartesianStressesLine.at(yyIdx),
                cartesianStressesLine.at(zzIdx),
                cartesianStressesLine.at(xyIdx),
                cartesianStressesLine.at(yzIdx),
                cartesianStressesLine.at(zxIdx),
                majorStress, mediumStress, minorStress/*, v0, v1, v2*/);
        float degeneracyMeasure = computeDegeneracyMeasure(
                minorStress, mediumStress, majorStress);
        attributeList.emplace_back(degeneracyMeasure);
#else
        attributeList.emplace_back(cartesianStressesLine.at(0));
#endif
    }
    isPerVertexData = true;

    /*
     * Ignore Silhouette data for now.
     * It is stored as follows:
     * Silhouette:
     * #Vertices: <num vertices>
     * <list of 3x float>
     * #Faces: <num faces>
     * <list of 3x uint>
     */

    return true;
}
