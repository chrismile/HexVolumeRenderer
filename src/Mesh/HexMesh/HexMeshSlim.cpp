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

/**
 * This function code below contains parts from BaseComplex/global_functions.cpp
 *
 * This file is part of the implementation of
 *    Robust Structure Simplification for Hex Re-meshing
 *    Xifeng Gao, Daniele Panozzo, Wenping Wang, Zhigang Deng, Guoning Chen
 *    In ACM Transactions on Graphics (Proceedings of SIGGRAPH ASIA 2017)
 *
 * Copyright (C) 2017 Xifeng Gao<gxf.xisha@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License
 * v. 2.0. If a copy of the MPL was not distributed with this file, You can
 * obtain one at http://mozilla.org/MPL/2.0/.
 */
void buildFacesSlim(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices,
        std::vector<FaceSlim>& facesSlim, std::vector<bool>& facesBoundary) {
    const uint32_t numCells = cellIndices.size() / 8;
    std::vector<FaceSlim> total_fs(numCells * 6);
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t>> tempF(numCells * 6);
    FaceSlim f;
    for (uint32_t h_id = 0; h_id < numCells; ++h_id) {
        for (short j = 0; j < 6; j++){
            for (short k = 0; k < 4; k++) f.vs[k] = cellIndices.at(h_id * 8 + hexFaceTable[j][k]);
            uint32_t id = 6 * h_id + j;
            total_fs[id] = f;
            std::sort(f.vs, f.vs + 4);
            tempF[id] = std::make_tuple(f.vs[0], f.vs[1], f.vs[2], f.vs[3], id, h_id, j);
        }
    }
    std::sort(tempF.begin(), tempF.end());
    facesSlim.reserve(tempF.size() / 3);
    uint32_t F_num = 0;
    for (uint32_t i = 0; i < tempF.size(); ++i) {
        if (i == 0 || (i != 0 &&
                       (std::get<0>(tempF[i]) != std::get<0>(tempF[i - 1]) || std::get<1>(tempF[i]) != std::get<1>(tempF[i - 1]) ||
                        std::get<2>(tempF[i]) != std::get<2>(tempF[i - 1]) || std::get<3>(tempF[i]) != std::get<3>(tempF[i - 1])))) {
            F_num++;
            f = total_fs[std::get<4>(tempF[i])];
            facesSlim.push_back(f);
            facesBoundary.push_back(true);
        }
        else if (i != 0 && (std::get<0>(tempF[i]) == std::get<0>(tempF[i - 1]) && std::get<1>(tempF[i]) == std::get<1>(tempF[i - 1]) &&
                            std::get<2>(tempF[i]) == std::get<2>(tempF[i - 1]) && std::get<3>(tempF[i]) == std::get<3>(tempF[i - 1])))
            facesBoundary[F_num - 1] = false;
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
        vertexAttributes.push_back(maximumCellAttributePerVertexSlim(verticesSlim, cellQualityMeasureList, v_id));
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