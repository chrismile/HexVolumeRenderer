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

#ifndef HEXVOLUMERENDERER_LODSHEETGENERATION_HPP
#define HEXVOLUMERENDERER_LODSHEETGENERATION_HPP

#include <vector>
#include <memory>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

struct LodSettings {
    bool operator==(const LodSettings& rhs) const {
        return this->lodMergeFactor == rhs.lodMergeFactor
                && this->useVolumeAndAreaMeasures == rhs.useVolumeAndAreaMeasures
                && this->useWeightsForMerging == rhs.useWeightsForMerging
                && this->useNumCellsOrVolume == rhs.useNumCellsOrVolume;
    }
    bool operator!=(const LodSettings& rhs) const {
        return this->lodMergeFactor != rhs.lodMergeFactor
                || this->useVolumeAndAreaMeasures != rhs.useVolumeAndAreaMeasures
                || this->useWeightsForMerging != rhs.useWeightsForMerging
                || this->useNumCellsOrVolume != rhs.useNumCellsOrVolume;
    }

    /**
     * By what factor (multiplicative) the merging criterion must increase such that a new LOD is started.
     * In case "useWeightsForMerging" is true, the inverse factor is used.
     */
    float lodMergeFactor = 2.0f;
    /// Whether to use volumes and areas or cell counts and face counts as measures.
    bool useVolumeAndAreaMeasures = false;
    /// useWeightsForMerging Whether to use weights for merging instead of the measures defined above.
    bool useWeightsForMerging = false;
    /// Whether to use the number of cells (!useVolumeAndAreaMeasures) / the cell volume (useVolumeAndAreaMeasures).
    bool useNumCellsOrVolume = true;
};

/**
 * Uses hexahedral mesh sheets to compute a level of detail structure of the grid lines.
 * This function returns the LOD levels for all mesh edges.
 * The sheets are merged to so-called components.
 *
 * What is a component?
 * - A component is a union of one or multiple base complex sheets.
 *
 * How is an LOD structure created using components?
 * - Initially, every sheet belongs to its own component.
 * - Then, in each iteration until only one component is left, one pair of neighboring components with the largest
 *   weight is matched and merged into a larger component. All shared edges are marked as not visible at the current
 *   LOD level. An exception is made for singular edges, which always remain visible.
 *
 * What does 'neighboring' mean for two components?
 * - Component c_0 shares at least one boundary face with another component c_1.
 *
 * How can we derive the neighborhood relation from two components c_0, c_1 when merging them to a component c'?
 * - Neighbors(c') = (Neighbors(c_0) UNION Neighbors(c_1)) \ {c_0, c_1}
 *
 * What edges do we mark as not visible when merging two components c_0, c_1?
 * - Mark all edges incident to the the shared boundary faces of the two components as not visible at the current LOD.
 *   The algorithm makes sure to only handle shared boundary faces that would no longer be on the boundary after
 *   merging.
 *
 * @param hexMesh The hexahedral mesh.
 * @param edgeLodValues The LOD values of all edges (between 0 and 1). Zero means always visible, one means visible
 * only at the most detailed LOD.
 * @param maxValueIntPtr Can optionally store the highest discrete LOD value (between 0 and MAX_LOD).
 * Can be used to scale the values in edgeLodValues.
 * @param lodSettings Settings for the algorithm.
 */
void generateSheetLevelOfDetailEdgeStructure(
        HexMesh* hexMesh,
        std::vector<float> &edgeLodValues,
        int* maxValueIntPtr = nullptr,
        LodSettings lodSettings = LodSettings());

/**
 * Uses hexahedral mesh sheets to compute a level of detail structure of the grid lines.
 * This function returns all edges in the mesh as lines and the corresponding vertex data.
 * The sheets are merged to so-called components.
 *
 * What is a component?
 * - A component is a union of one or multiple base complex sheets.
 *
 * How is an LOD structure created using components?
 * - Initially, every sheet belongs to its own component.
 * - Then, in each iteration until only one component is left, one pair of neighboring components with the largest
 *   weight is matched and merged into a larger component. All shared edges are marked as not visible at the current
 *   LOD level. An exception is made for singular edges, which always remain visible.
 *
 * What does 'neighboring' mean for two components?
 * - Component c_0 shares at least one boundary face with another component c_1.
 *
 * How can we derive the neighborhood relation from two components c_0, c_1 when merging them to a component c'?
 * - Neighbors(c') = (Neighbors(c_0) UNION Neighbors(c_1)) \ {c_0, c_1}
 *
 * What edges do we mark as not visible when merging two components c_0, c_1?
 * - Mark all edges incident to the the shared boundary faces of the two components as not visible at the current LOD.
 *   The algorithm makes sure to only handle shared boundary faces that would no longer be on the boundary after
 *   merging.
 *
 * @param hexMesh The hexahedral mesh.
 * @param lineVertices The list of line vertex positions.
 * @param lineColors The list of line vertex colors.
 * @param lineLodValues The list of line indices.
 * @param maxValueIntPtr Can optionally store the highest discrete LOD value (between 0 and MAX_LOD).
 * Can be used to scale the values in edgeLodValues.
 * @param lodSettings Settings for the algorithm.
 */
void generateSheetLevelOfDetailLineStructureAndVertexData(
        HexMesh* hexMesh,
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        std::vector<float> &lineLodValues,
        int* maxValueIntPtr = nullptr,
        LodSettings lodSettings = LodSettings());


struct LodHexahedralCellFace {
    glm::vec4 vertexPositions[4];
    glm::vec4 edgeColors[4];
    float edgeLodValues[4];
};

/**
 * Similar to @see generateSheetLevelOfDetailLineStructureAndVertexData, but instead of lines uses face-based rendering.
 */
void generateSheetLevelOfDetailLineStructureAndVertexData(
        HexMesh* hexMesh,
        std::vector<uint32_t>& triangleIndices,
        std::vector<LodHexahedralCellFace>& hexahedralCellFaces,
        int* maxValueIntPtr = nullptr,
        LodSettings lodSettings = LodSettings());


struct LodPreviewHexahedralCellFace {
    glm::vec4 vertexPositions[4];
    float edgeLodValues[4];
};

/**
 * Similar to @see generateSheetLevelOfDetailLineStructureAndVertexData, but instead of lines uses face-based rendering.
 */
void generateSheetPreviewLevelOfDetailLineStructureAndVertexData(
        HexMesh* hexMesh,
        std::vector<uint32_t>& triangleIndices,
        std::vector<LodPreviewHexahedralCellFace>& hexahedralCellFaces,
        int* maxValueIntPtr = nullptr,
        LodSettings lodSettings = LodSettings());

#endif //HEXVOLUMERENDERER_LODSHEETGENERATION_HPP
