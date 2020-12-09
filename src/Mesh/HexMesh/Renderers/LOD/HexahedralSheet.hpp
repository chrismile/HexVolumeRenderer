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

#ifndef HEXVOLUMERENDERER_HEXAHEDRALSHEET_HPP
#define HEXVOLUMERENDERER_HEXAHEDRALSHEET_HPP

#include <set>
#include <unordered_set>
#include <vector>
#include <memory>

// Use the following define to use volumes and surfaces instead of cell and face counts.
#define LOD_USE_VOLUME_AND_SURFACE_MEASURES

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

/**
 * A sheet of a hexahedral mesh.
 * A sheet is created by first selecting one edge in the mesh, and by adding all incident cells to the sheet.
 * This is then also done recursively for all edges incident to these cells that are parallel to the last edge.
 */
class HexahedralSheet {
public:
    std::vector<uint32_t> cellIds; ///< All cells belonging to the sheet.
    std::vector<uint32_t> boundaryFaceIds; ///< All boundary faces belonging to the sheet.
};

/**
 * A component consisting of one or more merged sheets.
 */
class SheetComponent : public HexahedralSheet {
public:
    std::unordered_set<uint32_t> neighborIndices; ///< Stores the indices of the neighbors in the sheet component array.
};

enum class ComponentConnectionType {
    ADJACENT = 0, HYBRID = 1, INTERSECTING = 2
};

class ComponentConnectionData {
public:
    uint32_t firstIdx, secondIdx;
    ComponentConnectionType componentConnectionType;
    float weight;

    bool operator<(const ComponentConnectionData& rhs) const {
        if (componentConnectionType != rhs.componentConnectionType) {
            return int(componentConnectionType) < int(rhs.componentConnectionType);
        } else if (weight != rhs.weight) {
            return weight > rhs.weight; // For priority queue: Store elements with higher weight at the front.
        } else if (firstIdx != rhs.firstIdx) {
            return firstIdx < rhs.firstIdx;
        } else {
            return secondIdx < rhs.secondIdx;
        }
    }

    bool operator==(const ComponentConnectionData& rhs) const {
        return firstIdx == rhs.firstIdx && secondIdx == rhs.secondIdx
                && componentConnectionType == rhs.componentConnectionType && weight == rhs.weight;
    }
};

// Hash Function: H(i,j) = (i0*p_1 xor i1*jp_2) mod n
struct ComponentConnectionDataHasher {
    std::size_t operator()(const ComponentConnectionData& key) const {
        const size_t PRIME_NUMBERS[] = { 12582917, 3145739 };
        return (key.firstIdx * PRIME_NUMBERS[0]) ^ (key.secondIdx * PRIME_NUMBERS[1]);
    }
};

/**
 * Extracts all mesh sheets from a hexahedral mesh.
 * A sheet is created by first selecting one edge in the mesh, and by adding all incident cells to the sheet.
 * This is then also done recursively for all edges incident to these cells that are parallel to the last edge.
 * @param hexMesh The hexahedral mesh.
 * @param hexahedralSheets The list of extracted hexahedral mesh sheets.
 */
void extractAllHexahedralSheets(HexMesh* hexMesh, std::vector<HexahedralSheet>& hexahedralSheets);

/**
 * For more details see:
 *
 * "Hexahedral Sheet Extraction", Michael J. Borden, Steven E. Benzley, Jason F. Shepherd (IMR 2002).
 * https://cfwebprod.sandia.gov/cfdocs/CompResearch/docs/HexahedralSheetExtraction.pdf
 *
 * "Localized Coarsening of Conforming All-Hexahedral Meshes", Adam Woodbury, Jason Shepherd, Matthew Staten, Steven
 * Benzley (2011). Eng. Comput. (Lond.). 27. 95-104. 10.1007/978-3-540-87921-3_36.
 * https://www.researchgate.net/publication/220677908_Localized_Coarsening_of_Conforming_All-Hexahedral_Meshes
 *
 * Select an edge e that defines the twist plane p.
 * Find all cells that share the edge and add them to the sheet (if not done already).
 * Then for each of the elements, find the three edges topologically parallel to the original edge.
 * Then use these edges to find new cells.
 *
 * @param hexMesh The hexahedral mesh.
 * @param e_id The ID of the edge defining the twist plane.
 * @param closedEdgeIds The set of edges that were already visited during sheet creation.
 * @param hexahedralSheet The hexahedral sheet extracted (output).
 */
void extractHexahedralSheet(
        HexMesh* hexMesh,
        uint32_t e_id,
        std::unordered_set<uint32_t>& closedEdgeIds,
        HexahedralSheet& hexahedralSheet);

/**
 * A helper function for @see extractHexahedralSheet and @see generateSheetLevelOfDetailLineStructure.
 * It computs the boundary surface face IDs of a hexahedral sheet.
 * @param hexMesh The hexahedral mesh.
 * @param hexahedralSheet The hexahedral sheet to compute the boundary surface face IDs of.
 */
void setHexahedralSheetBoundaryFaceIds(
        HexMesh* hexMesh,
        HexahedralSheet& hexahedralSheet);

/**
 * This function computes whether two sheets (or merged sheet components) are neighbors and what weight they should be
 * used for maximum weighted perfect matching.
 * Similarly to Xu et al. in their paper "Hexahedral Mesh Structure Visualization and Evaluation" (IEEE VIS 2018),
 * we define three types of relation between hexahedral mesh sheet components: Adjacent (or tangent), intersecting,
 * or hybrid (i.e., both tangent and intersecting). Sheet components are only neighbors if they share at least one
 * boundary face, and we give a higher matching weight to neighbors that are tangent. This is done by using the
 * percentage of adjacency between two sheets (again, similar to Xu et al.).
 *
 * @param hexMesh The hexahedral mesh.
 * @param component0 The first hexahedral sheet component.
 * @param component1 The second hexahedral sheet component.
 * @param useVolumeAndAreaMeasures Whether to use volumes and areas or cell counts and face counts as measures.
 * @param useNumCellsOrVolume Whether to use the number of cells (!useVolumeAndAreaMeasures) / the cell volume
 * (useVolumeAndAreaMeasures).
 * @param matchingWeight The weight the neighborship relation should have when merging/matching components.
 * @param componentConnectionType Whether the components are adjacent, intersecting or hybrid.
 * @return Whether the two passed hexahedral mesh sheets are neighbors.
 */
bool computeHexahedralSheetComponentNeighborship(
        HexMesh* hexMesh, SheetComponent& component0, SheetComponent& component1,
        bool useVolumeAndAreaMeasures, bool useNumCellsOrVolume,
        float& matchingWeight, ComponentConnectionType& componentConnectionType);

/**
 * Compute the neighborhood relation of all components and the edge weight of edges between components.
 * @param hexMesh The hexahedral mesh.
 * @param components The hexahedral mesh sheet components.
 * @param useVolumeAndAreaMeasures Whether to use volumes and areas or cell counts and face counts as measures.
 * @param useNumCellsOrVolume Whether to use the number of cells (!useVolumeAndAreaMeasures) / the cell volume
 * (useVolumeAndAreaMeasures).
 * @param connectionDataList The connection data of the components.
 * @param excludeIntersecting Whether to allow matching of intersecting or hybrid sheets or not.
 */
void computeHexahedralSheetComponentConnectionData(
        HexMesh* hexMesh, std::vector<SheetComponent*>& components,
        bool useVolumeAndAreaMeasures, bool useNumCellsOrVolume,
        std::vector<ComponentConnectionData>& connectionDataList);

#endif //HEXVOLUMERENDERER_HEXAHEDRALSHEET_HPP
