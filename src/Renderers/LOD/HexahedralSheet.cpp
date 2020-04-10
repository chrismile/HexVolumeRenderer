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

#include <unordered_set>
#include <limits>
#include <queue>
#include "BaseComplex/global_types.h"
#include "HexMesh/HexMesh.hpp"
#include "HexahedralSheet.hpp"

void getParallelEdgesInCell(Hybrid& h, Hybrid_E& e, uint32_t parallelEdgeIds[3]) {
    // Find the local index of the edge e in the cell h.
    size_t edgeCellIndex = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < h.es.size(); i++) {
        if (h.es.at(i) == e.id) {
            edgeCellIndex = i;
        }
    }

    // Add all parallel edges (this works thanks to @see buildCellEdgeList in HexMesh.cpp).
    size_t baseEdgeIndex = (edgeCellIndex / 4) * 4;
    int insertionIndex = 0;
    for (int i = 0; i < 4; i++) {
        if (baseEdgeIndex + i != edgeCellIndex) {
            parallelEdgeIds[insertionIndex] = h.es.at(baseEdgeIndex + i);
            insertionIndex++;
        }
    }
}

void extractHexahedralSheet(
        HexMeshPtr& hexMesh,
        uint32_t e_id,
        std::unordered_set<uint32_t>& closedEdgeIds,
        HexahedralSheet& hexahedralSheet) {
    Mesh& mesh = hexMesh->getBaseComplexMesh();
    uint32_t parallelEdgeIds[3];
    std::unordered_set<uint32_t> addedCellIds;
    std::queue<uint32_t> openEdgeIds;
    openEdgeIds.push(e_id);
    closedEdgeIds.insert(e_id);

    while (!openEdgeIds.empty()) {
        Hybrid_E& e = mesh.Es.at(openEdgeIds.front());
        openEdgeIds.pop();
        for (uint32_t h_id : e.neighbor_hs) {
            if (addedCellIds.find(h_id) != addedCellIds.end()) {
                continue;
            }
            hexahedralSheet.cellIds.push_back(h_id);
            addedCellIds.insert(h_id);

            Hybrid& h = mesh.Hs.at(h_id);
            getParallelEdgesInCell(h, e, parallelEdgeIds);
            for (int i = 0; i < 3; i++) {
                if (closedEdgeIds.find(parallelEdgeIds[i]) != closedEdgeIds.end()) {
                    continue;
                }
                openEdgeIds.push(parallelEdgeIds[i]);
                closedEdgeIds.insert(parallelEdgeIds[i]);
            }
        }
    }

    std::unordered_set<uint32_t> addedEdgeIds;
    for (uint32_t cellId : hexahedralSheet.cellIds) {
        Hybrid& h = mesh.Hs.at(cellId);
        for (uint32_t e_id : h.es) {
            if (addedEdgeIds.find(e_id) == addedEdgeIds.end()) {
                addedEdgeIds.insert(e_id);
                hexahedralSheet.edgeIds.push_back(e_id);
            }
        }
    }

    std::unordered_set<uint32_t> cellIdSet;
    for (uint32_t cellId : hexahedralSheet.cellIds) {
        cellIdSet.insert(cellId);
    }
    for (uint32_t cellId : hexahedralSheet.cellIds) {
        Hybrid& h = mesh.Hs.at(cellId);
        for (uint32_t f_id : h.fs) {
            Hybrid_F& f = mesh.Fs.at(f_id);

            // Boundary of the mesh?
            if (f.neighbor_hs.size() == 1) {
                hexahedralSheet.boundaryFaceIds.push_back(f_id);
            }

            // Boundary between two sheets?
            assert(f.neighbor_hs.size() == 2);
            uint32_t neighborCellId;
            if (f.neighbor_hs.at(0) == cellId) {
                neighborCellId = f.neighbor_hs.at(1);
            } else {
                assert(f.neighbor_hs.at(1) == cellId);
                neighborCellId = f.neighbor_hs.at(0);
            }
            // Neighboring cell not part of this sheet?
            if (cellIdSet.find(neighborCellId) == cellIdSet.end()) {
                hexahedralSheet.boundaryFaceIds.push_back(f_id);
            }
        }
        cellIdSet.insert(cellId);
    }
}

void extractAllHexahedralSheets(HexMeshPtr hexMesh, std::vector<HexahedralSheet>& hexahedralSheets) {
    Mesh& mesh = hexMesh->getBaseComplexMesh();
    std::unordered_set<uint32_t> closedEdgeIds;
    for (Hybrid_E& e : mesh.Es) {
        if (closedEdgeIds.find(e.id) != closedEdgeIds.end()) {
            continue;
        }

        HexahedralSheet hexahedralSheet;
        extractHexahedralSheet(hexMesh, e.id, closedEdgeIds, hexahedralSheet);
        hexahedralSheets.push_back(hexahedralSheet);
    }
}

bool computeHexahedralSheetComponentNeighborship(
        HexMeshPtr hexMesh, SheetComponent& component0, SheetComponent& component1, float& matchingWeight) {
    std::vector<uint32_t> sharedBoundaryFaceIds;
    std::set_intersection(
            component0.boundaryFaceIds.begin(), component0.boundaryFaceIds.end(),
            component1.boundaryFaceIds.begin(), component1.boundaryFaceIds.end(),
            std::back_inserter(sharedBoundaryFaceIds));

    float percentageOfAdjacency = sharedBoundaryFaceIds.size()
            / (component0.boundaryFaceIds.size() + component1.boundaryFaceIds.size());
    matchingWeight = percentageOfAdjacency;
    return true;
}

void computeHexahedralSheetComponentConnectionData(
        HexMeshPtr hexMesh,
        std::vector<SheetComponent>& components,
        std::vector<ComponentConnectionData>& connectionDataList) {
    for (size_t i = 0; i < components.size(); i++) {
        for (size_t j = i + 1; j < components.size(); j++) {
            SheetComponent& component0 = components.at(i);
            SheetComponent& component1 = components.at(j);
            float edgeWeight = 1.0f;
            bool componentsAreNeighbors = computeHexahedralSheetComponentNeighborship(
                    hexMesh, component0, component1, edgeWeight);
            if (!componentsAreNeighbors) {
                continue;
            }

            component0.neighborIndices.insert(j);
            component1.neighborIndices.insert(i);

            ComponentConnectionData connectionData;
            connectionData.firstIdx = i;
            connectionData.secondIdx = j;
            connectionData.weight = edgeWeight;
            connectionDataList.push_back(connectionData);
        }
    }
}
