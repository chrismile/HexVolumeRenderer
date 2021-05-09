/*
 * The code below contains parts from BaseComplex/global_functions.cpp
 *
 * That file is part of the implementation of
 *    Robust Structure Simplification for Hex Re-meshing
 *    Xifeng Gao, Daniele Panozzo, Wenping Wang, Zhigang Deng, Guoning Chen
 *    In ACM Transactions on Graphics (Proceedings of SIGGRAPH ASIA 2017)
 *
 * Copyright (C) 2017 Xifeng Gao<gxf.xisha@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License
 * v. 2.0. If a copy of the MPL was not distributed with this file, You can
 * obtain one at http://mozilla.org/MPL/2.0/.
 *
 * -----------------------------------------------------------------------------
 *
 * Rest of the code:
 *
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

#include <set>
#include <unordered_map>
#include "HexMesh.hpp"

void buildVerticesSlim(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices,
        std::vector<VertexSlim>& verticesSlim) {
    verticesSlim.resize(vertices.size());
    const uint32_t numCells = cellIndices.size() / 8;
    for (uint32_t h_id = 0; h_id < numCells; h_id++) {
        for (uint32_t v_internal_id = 0; v_internal_id < 8; v_internal_id++) {
            uint32_t v_id = cellIndices.at(h_id * 8 + v_internal_id);
            verticesSlim.at(v_id).hs.push_back(h_id);
        }
    }
}

const int hexFaceTable[6][4] = {
        // Use consistent winding for faces at the boundary (normals pointing out of the cell - no arbitrary decisions).
        { 0,1,2,3 },
        { 5,4,7,6 },
        { 4,5,1,0 },
        { 4,0,3,7 },
        { 6,7,3,2 },
        { 1,5,6,2 },
};

void buildFacesSlim(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices,
        std::vector<FaceSlim>& facesSlim, std::vector<bool>& isBoundaryFace) {
    struct TempFace {
        uint32_t vertexId[4];
        uint32_t faceId;

        inline bool operator==(const TempFace& other) const {
            for (int i = 0; i < 4; i++) {
                if (vertexId[i] != other.vertexId[i]) {
                    return false;
                }
            }
            return true;
        }

        inline bool operator!=(const TempFace& other) const {
            for (int i = 0; i < 4; i++) {
                if (vertexId[i] != other.vertexId[i]) {
                    return true;
                }
            }
            return false;
        }

        inline bool operator<(const TempFace& other) const {
            for (int i = 0; i < 4; i++) {
                if (vertexId[i] < other.vertexId[i]) {
                    return true;
                } else if (vertexId[i] > other.vertexId[i]) {
                    return false;
                }
            }
            return faceId < other.faceId;
        }
    };

    const uint32_t numCells = cellIndices.size() / 8;
    std::vector<FaceSlim> totalFaces(numCells * 6);
    std::vector<TempFace> tempFaces(numCells * 6);

    FaceSlim face;
    for (uint32_t cellId = 0; cellId < numCells; ++cellId) {
        for (uint32_t faceIdx = 0; faceIdx < 6; faceIdx++){
            for (uint32_t vertexIdx = 0; vertexIdx < 4; vertexIdx++) {
                face.vs[vertexIdx] = cellIndices.at(cellId * 8 + hexFaceTable[faceIdx][vertexIdx]);
            }

            uint32_t faceId = 6 * cellId + faceIdx;
            totalFaces[faceId] = face;
            std::sort(face.vs, face.vs + 4);
            tempFaces[faceId] = TempFace{
                    face.vs[0], face.vs[1], face.vs[2], face.vs[3], faceId
            };
        }
    }
    std::sort(tempFaces.begin(), tempFaces.end());

    facesSlim.reserve(tempFaces.size() / 3);
    uint32_t numFaces = 0;
    for (uint32_t i = 0; i < tempFaces.size(); ++i) {
        if (i == 0 || tempFaces[i] != tempFaces[i - 1]) {
            face = totalFaces[tempFaces[i].faceId];
            facesSlim.push_back(face);
            isBoundaryFace.push_back(true);
            numFaces++;
        } else {
            isBoundaryFace[numFaces - 1] = false;
        }
    }
}

float maximumCellAttributePerVertexSlim(
        const std::vector<VertexSlim>& verticesSlim, const std::vector<float>& cellQualityMeasureList, uint32_t v_id) {
    const VertexSlim& v = verticesSlim.at(v_id);

    float maximumCellAttributeValue = 0.0f;
    for (uint32_t h_id : v.hs) {
        maximumCellAttributeValue = std::max(maximumCellAttributeValue, cellQualityMeasureList.at(h_id));
    }
    return maximumCellAttributeValue;
}

void HexMesh::rebuildInternalRepresentationIfNecessary_Slim() {
    if (verticesSlim.empty()) {
        buildVerticesSlim(vertices, cellIndices, verticesSlim);
        buildFacesSlim(vertices, cellIndices, facesSlim, facesBoundarySlim);
    }

    if (dirty) {
        updateMeshTriangleIntersectionDataStructure_Slim();
        dirty = false;
    }
}

void HexMesh::updateMeshTriangleIntersectionDataStructure_Slim() {
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertices;

    // Add all hexahedral mesh vertices to the triangle mesh vertex data.
    vertices.reserve(vertices.size());
    for (uint32_t v_id = 0; v_id < vertices.size(); v_id++) {
        vertices.push_back(vertices.at(v_id));
    }

    // Add all triangle indices.
    triangleIndices.reserve(facesSlim.size() * 12);
    for (FaceSlim& f : facesSlim) {
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[1]);
        triangleIndices.push_back(f.vs[0]);
        triangleIndices.push_back(f.vs[3]);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[0]);

        triangleIndices.push_back(f.vs[0]);
        triangleIndices.push_back(f.vs[1]);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[0]);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[3]);
    }

    rayMeshIntersection.setMeshTriangleData(vertices, triangleIndices);
}

void HexMesh::getSurfaceData_Slim(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        bool removeFilteredCells) {
    rebuildInternalRepresentationIfNecessary_Slim();

    std::set<uint32_t> usedVertexSet;
    std::unordered_map<uint32_t, uint32_t> vertexIndexMap;

    for (size_t f_id = 0; f_id < facesSlim.size(); f_id++) {
        FaceSlim &f = facesSlim.at(f_id);
        if (!facesBoundarySlim.at(f_id)) {
            continue;
        }
        for (size_t i = 0; i < 4; i++) {
            usedVertexSet.insert(f.vs[i]);
        }
    }

    // Add the used hex-mesh vertices to the triangle mesh vertex data.
    uint32_t ctr = 0;
    for (uint32_t v_id : usedVertexSet) {
        vertexIndexMap.insert(std::make_pair(v_id, ctr));
        vertexPositions.push_back(this->vertices.at(v_id));
        ctr++;
    }

    // Add the triangle indices.
    for (size_t f_id = 0; f_id < facesSlim.size(); f_id++) {
        FaceSlim& f = facesSlim.at(f_id);
        if (!facesBoundarySlim.at(f_id)) {
            continue;
        }

        triangleIndices.push_back(vertexIndexMap[f.vs[2]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[1]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[0]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[3]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[2]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[0]]);

        triangleIndices.push_back(vertexIndexMap[f.vs[0]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[1]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[2]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[0]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[2]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[3]]);
    }
}

void HexMesh::getVolumeData_FacesShared_Slim(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<float>& vertexAttributes,
        bool useVolumeWeighting) {
    rebuildInternalRepresentationIfNecessary_Slim();

    // Add all hexahedral mesh vertices to the triangle mesh vertex data.
    for (uint32_t v_id = 0; v_id < this->vertices.size(); v_id++) {
        vertexPositions.push_back(this->vertices.at(v_id));
    }
    if (!useManualVertexAttribute) {
        for (uint32_t v_id = 0; v_id < this->vertices.size(); v_id++) {
            vertexAttributes.push_back(maximumCellAttributePerVertexSlim(verticesSlim, cellQualityMeasureList, v_id));
        }
    } else {
        for (uint32_t v_id = 0; v_id < this->vertices.size(); v_id++) {
            vertexAttributes.push_back(this->manualVertexAttributes->at(v_id));
        }
    }


    // Add all triangle indices.
    triangleIndices.reserve(facesSlim.size() * 6);
    for (FaceSlim& f : facesSlim) {
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[1]);
        triangleIndices.push_back(f.vs[0]);
        triangleIndices.push_back(f.vs[3]);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[0]);
    }
}

void HexMesh::getVolumeData_DepthComplexity_Slim(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions) {
    rebuildInternalRepresentationIfNecessary_Slim();

    for (uint32_t v_id = 0; v_id < this->vertices.size(); v_id++) {
        vertexPositions.push_back(this->vertices.at(v_id));
    }

    triangleIndices.reserve(facesSlim.size() * 6);
    for (FaceSlim& f : facesSlim) {
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[1]);
        triangleIndices.push_back(f.vs[0]);
        triangleIndices.push_back(f.vs[3]);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[0]);
    }
}


std::vector<float> HexMesh::getInterpolatedCellAttributeVertexData() const {
    std::vector<float> vertexAttributes;
    vertexAttributes.reserve(verticesSlim.size());
    for (uint32_t v_id = 0; v_id < this->vertices.size(); v_id++) {
        vertexAttributes.push_back(maximumCellAttributePerVertexSlim(verticesSlim, cellQualityMeasureList, v_id));
    }
    return vertexAttributes;
}

std::vector<float> HexMesh::getManualVertexAttributeDataNormalized() const {
    glm::vec2 minMaxValue = manualVertexAttributesMinMax.at(manualVertexAttributeIdx);
    std::vector<float> attributeData;
    attributeData.resize(manualVertexAttributes->size());

#if _OPENMP >= 201107
    #pragma omp parallel for shared(manualVertexAttributes, attributeData, minMaxValue) default(none)
#endif
    for (size_t i = 0; i < manualVertexAttributes->size(); i++) {
        attributeData.at(i) = (manualVertexAttributes->at(i) - minMaxValue.x) / (minMaxValue.y - minMaxValue.x);
    }

    return attributeData;
}

std::vector<float> HexMesh::getManualVertexAttributeDataNormalized(int attrIdx) const {
    glm::vec2 minMaxValue = manualVertexAttributesMinMax.at(attrIdx);
    const std::vector<float>& manualVertexAttributes = manualVertexAttributesList.at(attrIdx);
    std::vector<float> attributeData;
    attributeData.resize(manualVertexAttributes.size());

#if _OPENMP >= 200805
    #pragma omp parallel for shared(manualVertexAttributes, attributeData, minMaxValue) default(none)
#endif
    for (size_t i = 0; i < manualVertexAttributes.size(); i++) {
        attributeData.at(i) = (manualVertexAttributes.at(i) - minMaxValue.x) / (minMaxValue.y - minMaxValue.x);
    }

    return attributeData;
}
