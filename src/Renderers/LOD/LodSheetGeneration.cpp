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

#include <chrono>

#include <Utils/File/Logfile.hpp>

#include "BaseComplex/global_types.h"
#include "HexMesh/HexMesh.hpp"
#include "HexahedralSheet.hpp"
#include "SheetComponentMatching.hpp"
#include "LodSheetGeneration.hpp"

/**
 * std::set_union unfortunately does not work for std::unordered_set, as the inserter cannot be used and the ordering
 * is not guaranteed.
 * @param set0 Union component 0.
 * @param set1 Union component 1.
 * @param setUnion Used to store UNION(set0, set1).
 */
template <typename T>
void set_union(const std::unordered_set<T>& set0, const std::unordered_set<T>& set1, std::unordered_set<T>& setUnion) {
    setUnion = set0;
    setUnion.insert(set1.begin(), set1.end());
}

//union FloatPtrConstRemoval {
//    float* ptr;
//    const float* constPtr;
//};

void generateSheetLevelOfDetailLineStructure(
        HexMeshPtr& hexMesh,
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        std::vector<float> &lineLodValues) {
    Mesh& mesh = hexMesh->getBaseComplexMesh();
    Singularity& si = hexMesh->getBaseComplexMeshSingularity();

    sgl::Logfile::get()->writeInfo("Starting to generate mesh sheet level of detail structure...");
    auto start = std::chrono::system_clock::now();

    // First, extract all hexahedral sheets from the mesh.
    std::vector<HexahedralSheet> hexahedralSheets;
    extractAllHexahedralSheets(hexMesh, hexahedralSheets);

    // Initially, every sheet belongs to its own component.
    std::vector<SheetComponent> components;
    components.resize(hexahedralSheets.size());
    for (size_t i = 0; i < components.size(); i++) {
        SheetComponent& component = components.at(i);
        HexahedralSheet& sheet = hexahedralSheets.at(i);
        component.cellIds = sheet.cellIds;
        component.edgeIds = sheet.edgeIds;
        component.boundaryFaceIds = sheet.boundaryFaceIds;
        std::sort(component.cellIds.begin(), component.cellIds.end());
        std::sort(component.edgeIds.begin(), component.edgeIds.end());
        std::sort(component.boundaryFaceIds.begin(), component.boundaryFaceIds.end());
    }

    // Compute the neighborhood relation of all components and the edge weight of edges between components.
    //std::vector<ComponentConnectionData> connectionDataList;
    //computeHexahedralSheetComponentConnectionData(hexMesh, components, connectionDataList);

    std::vector<int> lodEdgeVisibilityMap;
    lodEdgeVisibilityMap.resize(mesh.Es.size(), 0);


    for (int iterationNumber = 1; ; iterationNumber++) {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Starting iteration number " + std::to_string(iterationNumber) + "...");
        auto startIteration = std::chrono::system_clock::now();

        // Compute the neighborhood relation of all components and the edge weight of edges between components.
        std::vector<ComponentConnectionData> connectionDataList;
        computeHexahedralSheetComponentConnectionData(hexMesh, components, connectionDataList);

        // Create a perfect matching of all neighboring components.
        SheetComponentMatching matching(hexMesh, components, connectionDataList);
        if (!matching.getCouldMatchAny()) {
            auto endIteration = std::chrono::system_clock::now();
            auto elapsedIteration = std::chrono::duration_cast<std::chrono::milliseconds>(endIteration - startIteration);
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Stopping at iteration number " + std::to_string(iterationNumber)
                    + ". Time for this iteration: " + std::to_string(elapsedIteration.count()) + "ms");
            break;
        }

        // An index map mapping the indices of the components to their indices after merging.
        std::unordered_map<size_t, size_t> mergedComponentIndexMap;
        std::vector<SheetComponent> mergedComponents;
        //std::vector<ComponentConnectionData> mergedConnectionDataList;

        // Mark all edges as invisible on this level shared by matched components.
        size_t mergedComponentIndex = 0;
        std::vector<std::pair<size_t, size_t>> matchedComponents = matching.getMatchedComponents();
        for (std::pair<size_t, size_t>& matchedEdge : matchedComponents) {
            SheetComponent& component0 = components.at(matchedEdge.first);
            SheetComponent& component1 = components.at(matchedEdge.second);
            SheetComponent mergedComponent;
            // Cells(c') = (Cells(c_0) UNION Cells(c_1))
            std::set_union(
                    component0.cellIds.begin(), component0.cellIds.end(),
                    component1.cellIds.begin(), component1.cellIds.end(),
                    std::back_inserter(mergedComponent.cellIds));

            // Edges(c') = (Edges(c_0) UNION Edges(c_1))
            std::set_union(
                    component0.edgeIds.begin(), component0.edgeIds.end(),
                    component1.edgeIds.begin(), component1.edgeIds.end(),
                    std::back_inserter(mergedComponent.edgeIds));

            // Recompute the boundary faces of the merged mesh.
            setHexahedralSheetBoundaryFaceIds(hexMesh, mergedComponent);
            std::sort(mergedComponent.boundaryFaceIds.begin(), mergedComponent.boundaryFaceIds.end());

            // Neighbors(c') = (Neighbors(c_0) UNION Neighbors(c_1)) \ {c_0, c_1} (these are removed later)
            //set_union(component0.neighborIndices, component1.neighborIndices, mergedComponent.neighborIndices);

            // Mark all edges E(c_0) INTERSECTION E(c_1) as not visible at the current LOD level.
            std::vector<uint32_t> sharedEdgeSet;
            std::set_intersection(
                    component0.edgeIds.begin(), component0.edgeIds.end(),
                    component1.edgeIds.begin(), component1.edgeIds.end(),
                    std::back_inserter(sharedEdgeSet));
            for (uint32_t e_id : sharedEdgeSet) {
                lodEdgeVisibilityMap.at(e_id) = std::max(lodEdgeVisibilityMap.at(e_id), iterationNumber);
            }

            mergedComponents.push_back(mergedComponent);
            /*mergedComponentIndexMap.insert(std::make_pair(matchedEdge.first, mergedComponentIndex));
            mergedComponentIndexMap.insert(std::make_pair(matchedEdge.second, mergedComponentIndex));
            mergedComponentIndex++;*/
        }

        // Add all unmatched (i.e., unchanged) components to the index map and the merged component set.
        std::vector<size_t> unmatchedComponents = matching.getUnmatchedComponents();
        for (size_t unmatchedComponent : unmatchedComponents) {
            mergedComponents.push_back(components.at(unmatchedComponent));
            /*mergedComponentIndexMap.insert(std::make_pair(unmatchedComponent, mergedComponentIndex));
            mergedComponentIndex++;*/
        }

        // Fix the indices of the neighbor relation using the index map.
        /*for (int i = 0; i < mergedComponents.size(); i++) {
            SheetComponent& component = mergedComponents.at(i);
            std::unordered_set<size_t> newNeighborIndices;
            for (size_t oldNeighborIndex : component.neighborIndices) {
                size_t newNeighborIndex = mergedComponentIndexMap.find(oldNeighborIndex)->second;
                if (newNeighborIndex != i) {
                    newNeighborIndices.insert(newNeighborIndex);
                }
            }
            component.neighborIndices = newNeighborIndices;
        }
        std::unordered_set<ComponentConnectionData, ComponentConnectionDataHasher> addedConnections;
        for (ComponentConnectionData connectionData : connectionDataList) {
            ComponentConnectionData mergedConnectionData;
            mergedConnectionData.firstIdx = mergedComponentIndexMap.find(connectionData.firstIdx)->second;
            mergedConnectionData.secondIdx = mergedComponentIndexMap.find(connectionData.secondIdx)->second;
            if (mergedConnectionData.firstIdx > mergedConnectionData.secondIdx) {
                size_t tmp = mergedConnectionData.firstIdx;
                mergedConnectionData.firstIdx = mergedConnectionData.secondIdx;
                mergedConnectionData.secondIdx = tmp;
            }
            mergedConnectionData.weight = connectionData.weight;

            auto it = addedConnections.find(mergedConnectionData);
            if (it == addedConnections.end()) {
                mergedConnectionDataList.push_back(mergedConnectionData);
            } else {
                // TODO: Max, min, average?
                // Dirty fix, as C++ standard doesn't allow changing values in std::unordered_set... :(
                const float* weightPtrConst = &it->weight;
                FloatPtrConstRemoval constRemoval;
                constRemoval.constPtr = weightPtrConst;
                float& weight = *constRemoval.ptr;
                weight = std::max(weight, mergedConnectionData.weight);
            }
        }*/

        // Compute the neighborhood relation of all components and the edge weight of edges between components.
        //std::vector<ComponentConnectionData> connectionDataList;
        //computeHexahedralSheetComponentConnectionData(hexMesh, mergedComponents, mergedConnectionDataList);

        // Set data to merged data for the next iteration.
        components = mergedComponents;
        //connectionDataList = mergedConnectionDataList;

        auto endIteration = std::chrono::system_clock::now();
        auto elapsedIteration = std::chrono::duration_cast<std::chrono::milliseconds>(endIteration - startIteration);
        sgl::Logfile::get()->writeInfo(
                std::string() + "Time for iteration number " + std::to_string(iterationNumber) + ": "
                + std::to_string(elapsedIteration.count()) + "ms");
    }

    // We want to normalize the LOD values to the range [0, 1]. First, compute the maximum value.
    float maxValue = 1.0f;
    //#pragma omp parallel for reduction(max: maxValue)
    for (size_t i = 0; i < lodEdgeVisibilityMap.size(); i++) {
        maxValue = std::max(maxValue, float(lodEdgeVisibilityMap.at(i)));
    }

    // Now, normalize the values by division.
    lineLodValues.reserve(lodEdgeVisibilityMap.size() * 2);
    //#pragma omp parallel for
    for (size_t i = 0; i < lodEdgeVisibilityMap.size(); i++) {
        float value = lodEdgeVisibilityMap.at(i) / maxValue;
        lineLodValues.push_back(value);
        lineLodValues.push_back(value);
    }

    // Find the set of all singular edges.
    std::unordered_set<uint32_t> singularEdgeIds;
    for (Singular_E& se : si.SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIds.insert(e_id);
        }
    }

    const glm::vec4 regularColor = hexMesh->outlineColorRegular;
    const glm::vec4 singularColor = hexMesh->outlineColorSingular;

    // Add all edge line colors (colors depend both on whether an edge is singular and what LOD value it has assigned).
    for (size_t i = 0; i < mesh.Es.size(); i++) {
        Hybrid_E& e = mesh.Es.at(i);
        bool isSingular = singularEdgeIds.find(e.id) != singularEdgeIds.end();
        glm::vec4 vertexColor;
        if (isSingular) {
            vertexColor = singularColor;
        } else {
            vertexColor = regularColor;
        }

        float lodValue = lineLodValues.at(i * 2) * maxValue;
        if (lodValue > 0.0001) {
            float interpolationFactor = (lodValue - 1.0f) / (maxValue - 1.0f) * 0.5;
            vertexColor = glm::vec4(glm::mix(
                    glm::vec3(vertexColor.x, vertexColor.y, vertexColor.z),
                    glm::vec3(0.8f, 0.8f, 0.8f), interpolationFactor), vertexColor.a);
        }

        for (uint32_t v_id : e.vs) {
            glm::vec3 vertexPosition(mesh.V(0, v_id), mesh.V(1, v_id), mesh.V(2, v_id));
            lineVertices.push_back(vertexPosition);
            lineColors.push_back(vertexColor);
        }
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    sgl::Logfile::get()->writeInfo(
            std::string() + "Computational time to create sheet LOD structure: "
            + std::to_string(elapsed.count()) + "ms");
}
