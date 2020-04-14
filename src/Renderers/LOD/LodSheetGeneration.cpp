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

void generateSheetLevelOfDetailLineStructure(
        HexMeshPtr& hexMesh,
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        std::vector<float> &lineLodValues) {
    Mesh& mesh = hexMesh->getBaseComplexMesh();
    Singularity& si = hexMesh->getBaseComplexMeshSingularity();

    sgl::Logfile::get()->writeInfo("Starting to generate mesh sheet level of detail structure...");
    auto start = std::chrono::system_clock::now();

    // Find the set of all singular edges.
    std::unordered_set<uint32_t> singularEdgeIds;
    for (Singular_E& se : si.SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIds.insert(e_id);
        }
    }

    // Don't highlight singular edges when we have far too many of them.
    bool tooMuchSingularEdgeMode = singularEdgeIds.size() > 10000u;

    // First, extract all hexahedral sheets from the mesh.
    std::vector<HexahedralSheet> hexahedralSheets;
    extractAllHexahedralSheets(hexMesh, hexahedralSheets);

    // Initially, every sheet belongs to its own component.
    std::vector<SheetComponent*> components;
    components.reserve(hexahedralSheets.size());
    for (size_t i = 0; i < hexahedralSheets.size(); i++) {
        SheetComponent* component = new SheetComponent;
        components.push_back(component);
        HexahedralSheet& sheet = hexahedralSheets.at(i);
        component->cellIds = sheet.cellIds;
        component->boundaryFaceIds = sheet.boundaryFaceIds;
        std::sort(component->cellIds.begin(), component->cellIds.end());
        std::sort(component->boundaryFaceIds.begin(), component->boundaryFaceIds.end());
    }

    // Compute the neighborhood relation of all components and the edge weight of edges between components.
    std::vector<ComponentConnectionData> connectionDataList;
    computeHexahedralSheetComponentConnectionData(hexMesh, components, connectionDataList);
    std::set<ComponentConnectionData> connectionDataSet; // Use similarly to a priority queue
    for (ComponentConnectionData& componentConnectionData : connectionDataList) {
        connectionDataSet.insert(componentConnectionData);
    }

    // This array represents a map storing for each line the LOD level where it was marked as last visible.
    std::vector<int> lodEdgeVisibilityMap;
    lodEdgeVisibilityMap.resize(mesh.Es.size(), 0);

    // Data for merging similar merges into one LOD level. If 'mergeLodLevels' is set to false, each merge gets its own
    // LOD level, i.e., LOD level @see iterationNumber.
    bool mergeLodLevels = true;
    int lodLevel = 0;
    int lodLevelFirstNumCellsAfterMerging = 0;
    // For creating a discrete LOD when switching from merging adjacent to hybrid or intersecting sheet components.
    bool justSwitchedToIntersectingOrHybridComponentMerging = false;
    bool switchedToIntersectingOrHybridComponentMerging = false;

    int iterationNumber = 1;
    while (true) {
        if (connectionDataSet.size() == 0) {
            sgl::Logfile::get()->writeInfo("Finished merging all mesh sheet components.");
            break;
        }

        sgl::Logfile::get()->writeInfo(
                std::string() + "Starting iteration number " + std::to_string(iterationNumber) + "...");
        auto startIteration = std::chrono::system_clock::now();

        // An index map mapping the indices of the components to their indices after merging (in @see mergedComponents).
        std::unordered_map<uint32_t, uint32_t> mergedComponentIndexMap;
        std::vector<SheetComponent*> mergedComponents;
        mergedComponents.reserve(components.size() - 1);

        // Find the LOD with the best merging weight.
        ComponentConnectionData bestMatchingComponentConnectionData = *connectionDataSet.begin();
        connectionDataSet.erase(connectionDataSet.begin());

        // For creating a discrete LOD when switching from merging adjacent to hybrid or intersecting sheet components.
        if (bestMatchingComponentConnectionData.componentConnectionType == ComponentConnectionType::INTERSECTING
                || bestMatchingComponentConnectionData.componentConnectionType == ComponentConnectionType::HYBRID) {
            if (!switchedToIntersectingOrHybridComponentMerging) {
                justSwitchedToIntersectingOrHybridComponentMerging = true;
            } else {
                justSwitchedToIntersectingOrHybridComponentMerging = false;
            }
            switchedToIntersectingOrHybridComponentMerging = true;
        }

        // Mark all edges as invisible on this level shared by matched components.
        SheetComponent* component0 = components.at(bestMatchingComponentConnectionData.firstIdx);
        SheetComponent* component1 = components.at(bestMatchingComponentConnectionData.secondIdx);
        SheetComponent* mergedComponent = new SheetComponent;
        // Cells(c') = (Cells(c_0) UNION Cells(c_1))
        std::set_union(
                component0->cellIds.begin(), component0->cellIds.end(),
                component1->cellIds.begin(), component1->cellIds.end(),
                std::back_inserter(mergedComponent->cellIds));

        // Recompute the boundary faces of the merged mesh.
        setHexahedralSheetBoundaryFaceIds(hexMesh, *mergedComponent);
        std::sort(mergedComponent->boundaryFaceIds.begin(), mergedComponent->boundaryFaceIds.end());

        // Compute the edges that vanish after the merge.
        std::vector<uint32_t> boundaryFaceIdsUnion;
        std::set_union(
                component0->boundaryFaceIds.begin(), component0->boundaryFaceIds.end(),
                component1->boundaryFaceIds.begin(), component1->boundaryFaceIds.end(),
                std::back_inserter(boundaryFaceIdsUnion));

        std::vector<uint32_t> boundaryFaceIdsNoLongerBoundaryAfterMerging;
        std::set_difference(
                boundaryFaceIdsUnion.begin(), boundaryFaceIdsUnion.end(),
                mergedComponent->boundaryFaceIds.begin(), mergedComponent->boundaryFaceIds.end(),
                std::back_inserter(boundaryFaceIdsNoLongerBoundaryAfterMerging));

        std::unordered_set<uint32_t> vanishedEdgeIdsSet;
        std::vector<uint32_t> vanishedEdgeIds;
        for (uint32_t f_id : boundaryFaceIdsNoLongerBoundaryAfterMerging) {
            Hybrid_F& f = mesh.Fs.at(f_id);
            for (uint32_t e_id : f.es) {
                if (vanishedEdgeIdsSet.find(e_id) == vanishedEdgeIdsSet.end()) {
                    vanishedEdgeIdsSet.insert(e_id);
                    vanishedEdgeIds.push_back(e_id);
                }
            }
        }

        // If merging LOD levels is active, compute whether we need to start a new LOD. Otherwise use the current
        // iteration number as the LOD level.
        if (mergeLodLevels) {
            size_t numCellsAfterMerging = mergedComponent->cellIds.size();
            if (numCellsAfterMerging >= 2 * lodLevelFirstNumCellsAfterMerging
                    || justSwitchedToIntersectingOrHybridComponentMerging) {
                lodLevel++;
                lodLevelFirstNumCellsAfterMerging = numCellsAfterMerging;
            } else if (justSwitchedToIntersectingOrHybridComponentMerging) {
                lodLevel++;
                lodLevelFirstNumCellsAfterMerging = std::max(
                        lodLevelFirstNumCellsAfterMerging, int(numCellsAfterMerging));
            }
        } else {
            lodLevel = iterationNumber;
        }

        // Mark all edges that vanished from this LOD.
        for (uint32_t e_id : vanishedEdgeIds) {
            bool isSingular = singularEdgeIds.find(e_id) != singularEdgeIds.end();
            if (isSingular && !tooMuchSingularEdgeMode) {
                continue;
            }

            if (lodEdgeVisibilityMap.at(e_id) == 0) {
                lodEdgeVisibilityMap.at(e_id) = lodLevel;
            } else {
                lodEdgeVisibilityMap.at(e_id) = std::min(lodEdgeVisibilityMap.at(e_id), lodLevel);
            }
        }

        // Add the merged component to the index map and the merged component set.
        mergedComponents.push_back(mergedComponent);
        size_t mergedComponentIndex = 0;
        mergedComponentIndexMap.insert(std::make_pair(
                bestMatchingComponentConnectionData.firstIdx, mergedComponentIndex));
        mergedComponentIndexMap.insert(std::make_pair(
                bestMatchingComponentConnectionData.secondIdx, mergedComponentIndex));
        mergedComponentIndex++;

        // Add all unmatched (i.e., unchanged) components to the index map and the merged component set.
        size_t oldComponentIndex = 0;
        for (SheetComponent* component : components) {
            mergedComponentIndexMap.insert(std::make_pair(oldComponentIndex, mergedComponentIndex));
            if (component != component0 && component != component1) {
                mergedComponents.push_back(component);
                mergedComponentIndex++;
            }
            oldComponentIndex++;
        }

        // Free the memory of the old components. They are no longer used afterwards.
        delete component0;
        delete component1;

        // Fix the indices of the neighborship relation using the index map and recompute the merging weight between
        // the neighbors of the merged component and the merged component itself.
        for (int i = 1; i < mergedComponents.size(); i++) {
            SheetComponent* component = mergedComponents.at(i);
            std::unordered_set<uint32_t> newNeighborIndices;
            for (uint32_t oldNeighborIndex : component->neighborIndices) {
                uint32_t newNeighborIndex = mergedComponentIndexMap.find(oldNeighborIndex)->second;
                if (newNeighborIndex != i) {
                    newNeighborIndices.insert(newNeighborIndex);
                }
            }
            component->neighborIndices = newNeighborIndices;
        }
        std::set<ComponentConnectionData> mergedConnectionDataSet;
        std::unordered_set<ComponentConnectionData, ComponentConnectionDataHasher> mergedConnectionDataUnorderedSet;
        for (const ComponentConnectionData& connectionData : connectionDataSet) {
            ComponentConnectionData mergedConnectionData;
            mergedConnectionData.firstIdx = mergedComponentIndexMap.find(connectionData.firstIdx)->second;
            mergedConnectionData.secondIdx = mergedComponentIndexMap.find(connectionData.secondIdx)->second;
            if (mergedConnectionData.firstIdx > mergedConnectionData.secondIdx) {
                size_t tmp = mergedConnectionData.firstIdx;
                mergedConnectionData.firstIdx = mergedConnectionData.secondIdx;
                mergedConnectionData.secondIdx = tmp;
            }

            auto it = mergedConnectionDataUnorderedSet.find(mergedConnectionData);
            if (it == mergedConnectionDataUnorderedSet.end()) {
                if (mergedConnectionData.firstIdx == 0 || mergedConnectionData.secondIdx == 0) {
                    float weight;
                    ComponentConnectionType componentConnectionType;
                    bool isNeighbor = computeHexahedralSheetComponentNeighborship(
                            hexMesh,
                            *mergedComponents.at(mergedConnectionData.firstIdx),
                            *mergedComponents.at(mergedConnectionData.secondIdx),
                            weight, componentConnectionType);
                    mergedConnectionData.weight = weight;
                    mergedConnectionData.componentConnectionType = componentConnectionType;
                    mergedComponents.at(mergedConnectionData.firstIdx)->neighborIndices.insert(
                            mergedConnectionData.secondIdx);
                    mergedComponents.at(mergedConnectionData.secondIdx)->neighborIndices.insert(
                            mergedConnectionData.firstIdx);
                    if (!isNeighbor) {
                        continue;
                    }
                } else {
                    mergedConnectionData.weight = connectionData.weight;
                    mergedConnectionData.componentConnectionType = connectionData.componentConnectionType;
                }
                mergedConnectionDataUnorderedSet.insert(mergedConnectionData);
                mergedConnectionDataSet.insert(mergedConnectionData);
            }
        }

        // Set data to merged data for the next iteration.
        components = mergedComponents;
        connectionDataSet = mergedConnectionDataSet;

        auto endIteration = std::chrono::system_clock::now();
        auto elapsedIteration = std::chrono::duration_cast<std::chrono::milliseconds>(endIteration - startIteration);
        sgl::Logfile::get()->writeInfo(
                std::string() + "Time for iteration number " + std::to_string(iterationNumber) + ": "
                + std::to_string(elapsedIteration.count()) + "ms");
        iterationNumber++;
    }

    for (SheetComponent* component : components) {
        delete component;
    }
    components.clear();


    /*if (tooMuchSingularEdgeMode) {
        for (size_t i = 0; i < lodEdgeVisibilityMap.size(); i++) {
            if (lodEdgeVisibilityMap.at(i) != 0) {
                lodEdgeVisibilityMap.at(i)++;
            }
        }
        for (uint32_t e_id : singularEdgeIds) {
            lodEdgeVisibilityMap.at(e_id) = 1;
        }
    }*/

    // We want to normalize the LOD values to the range [0, 1]. First, compute the maximum value.
    int maxValueInt = 1;
    //#pragma omp parallel for reduction(max: maxValue)
    for (size_t i = 0; i < lodEdgeVisibilityMap.size(); i++) {
        maxValueInt = std::max(maxValueInt, lodEdgeVisibilityMap.at(i));
    }
    float maxValue = float(maxValueInt);

    //#pragma omp parallel for
    for (size_t i = 0; i < lodEdgeVisibilityMap.size(); i++) {
        if (lodEdgeVisibilityMap.at(i) > 0) {
            lodEdgeVisibilityMap.at(i) = maxValueInt - lodEdgeVisibilityMap.at(i) + 1;
        }
    }

    // Now, normalize the values by division.
    lineLodValues.reserve(lodEdgeVisibilityMap.size() * 2);
    //#pragma omp parallel for
    for (size_t i = 0; i < lodEdgeVisibilityMap.size(); i++) {
        float value = lodEdgeVisibilityMap.at(i) / maxValue;
        lineLodValues.push_back(value);
        lineLodValues.push_back(value);
    }

    // Add all edge line colors (colors depend both on whether an edge is singular and what LOD value it has assigned).
    for (size_t i = 0; i < mesh.Es.size(); i++) {
        Hybrid_E& e = mesh.Es.at(i);
        bool isSingular = singularEdgeIds.find(e.id) != singularEdgeIds.end();
        int edgeValence = int(e.neighbor_hs.size());
        glm::vec4 vertexColor = hexMesh->edgeColorMap(isSingular, e.boundary, edgeValence);

        float lodValue = lineLodValues.at(i * 2) * maxValue;
        if (lodValue > 0.0001) {
            float interpolationFactor = 1.0f;
            if (maxValue > 1.00001f) {
                interpolationFactor = (lodValue - 1.0f) / (maxValue - 1.0f);
            }
            interpolationFactor *= 0.5f;
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
