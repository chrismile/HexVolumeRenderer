/*
 * This file is the interface between HexVolumeRenderer, HexaMesh classes and
 * the code from Robust Hexahedral Re-Meshing (see README).
 * Some functions from HexaMesh are used here, which are covered by the MIT license.
 * The changes for the interoperability are covered by the BSD 2-Clause license.
 *
 *
 * MIT License
 *
 * Copyright (c) 2018 Visual Computing Lab - ISTI - CNR
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
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

#ifndef GENERALMAP_GENERALMAP_HPP
#define GENERALMAP_GENERALMAP_HPP

#include <vector>
#include <unordered_set>
#include <memory>

#include <glm/vec3.hpp>

#include <Math/Geometry/AABB3.hpp>

#include "QualityMeasure/QualityMeasure.hpp"
#include "Renderers/TransferFunctionWindow.hpp"
#include "Renderers/Intersection/RayMeshIntersection.hpp"
#include "HexaLab/builder.h"
#include "HexaLab/app.h"

class HexMesh;
class Mesh;
class Singularity;
class Hybrid_E;
class Frame;
class ParametrizedGrid;
class OctreeNode;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

struct HexahedralCellFace {
    glm::vec4 vertexPositions[4];
    glm::vec4 lineColors[4];
};

class HexMesh {
public:
    HexMesh(TransferFunctionWindow &transferFunctionWindow, RayMeshIntersection& rayMeshIntersection)
        : transferFunctionWindow(transferFunctionWindow), rayMeshIntersection(rayMeshIntersection) {}
    ~HexMesh();
    void setHexMeshData(const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices);
    void setQualityMeasure(QualityMeasure qualityMeasure);
    void onTransferFunctionMapRebuilt();
    void unmark();
    bool isDirty() { return dirty; }

    // For filters
    HexaLab::Mesh& getMesh();

    /**
     * Get the triangle data of the boundary surface of the hexahedral mesh.
     */
    void getSurfaceData(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec3>& normals,
            std::vector<glm::vec4>& colors);
    /**
     * Get the wireframe data of the boundary surface of the hexahedral mesh.
     */
    void getWireframeData(
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec4>& colors);
    /**
     * Get the surface data of all front faces (and backfaces for boundary surface) of every cell of the hexahedral
     * mesh.
     */
    void getVolumeData_Faces(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec3>& normals,
            std::vector<glm::vec4>& colors);
    /**
     * Get the surface data of all front faces of every cell of the hexahedral mesh.
     */
    void getVolumeData_Volume(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec3>& normals,
            std::vector<glm::vec4>& colors);
    /**
     * Get the singular edges and points of the hexahedral mesh.
     */
    void getSingularityData(
            std::vector<glm::vec3>& lineVertices,
            std::vector<glm::vec4>& lineColors,
            std::vector<glm::vec3>& pointVertices,
            std::vector<glm::vec4>& pointColors);
    /**
     * Get the base-complex edges and corner vertices of the hexahedral mesh.
     * @param drawRegularLines If this value is true, also regular edges and corners are added.
     */
    void getBaseComplexDataWireframe(
            std::vector<glm::vec3>& lineVertices,
            std::vector<glm::vec4>& lineColors,
            std::vector<glm::vec3>& pointVertices,
            std::vector<glm::vec4>& pointColors,
            bool drawRegularLines);
    /**
     * Get the base-complex partition surface data of the hexahedral mesh.
     * @param cullInterior If this value is true, only the boundary surfaces are exported.
     * Otherwise, also inner surfaces are also added.
     */
    void getBaseComplexDataSurface(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec3>& normals,
            std::vector<glm::vec4>& colors,
            bool cullInterior);
    /**
     * Exports all lines of the hexahedral mesh, but colors them according to their parametrization in each base-complex
     * partition with (R, G, B, A) = (u/u_max, v/v_max, w/w_max, 1).
     */
    void getColoredPartitionLines(
            std::vector<glm::vec3>& lineVertices,
            std::vector<glm::vec4>& lineColors);
    /**
     * Get the list of lines associated with LOD values. Less important lines have higher values and can be filtered
     * out during rendering.
     * @param lineLodValues The LOD value normalized to [0, 1].
     * @param previewColors If this value is true, more vibrant colors are assigned to the line vertices.
     */
    void getLodLineRepresentation(
            std::vector<glm::vec3> &lineVertices,
            std::vector<glm::vec4> &lineColors,
            std::vector<float> &lineLodValues,
            bool previewColors);
    /**
     * Uses an octree to construct an LOD representation of the mesh. It creates a renderable line representation of
     * the edge meshes that is only refined in regions close to a focus point.
     * @param focusPoint The focus point.
     * @param focusRadius The radius that defines the 'near' region of the focus point that should be highly refined.
     */
    void getLodLineRepresentationClosest(
            std::vector<glm::vec3> &lineVertices,
            std::vector<glm::vec4> &lineColors,
            const glm::vec3& focusPoint,
            float focusRadius);
    /**
     * Get a list of all wireframe lines. The color is either black (regular edge) or red (irregular/singular edge).
     */
    void getCompleteWireframeData(
            std::vector<glm::vec3> &lineVertices,
            std::vector<glm::vec4> &lineColors);
    /**
     * Get a list of all wireframe lines as lists of tube line center vertices.
     * The color is either black (regular edge) or red (irregular/singular edge).
     */
    void getCompleteWireframeTubeData(
            std::vector<std::vector<glm::vec3>>& lineCentersList,
            std::vector<std::vector<glm::vec4>>& lineColorsList);
    /**
     * Get a list of all vertex positions. The color is either black (regular vertex) or red (irregular vertex).
     */
    void getCompleteVertexData(
            std::vector<glm::vec3> &pointVertices,
            std::vector<glm::vec4> &pointColors);
    /**
     * Get a list of all vertex positions necessary for closing holes in tubes generated by
     * @see getCompleteWireframeTubeData.
     * The color is either black (regular vertex) or red (irregular vertex).
     */
    void getVertexTubeData(
            std::vector<glm::vec3> &pointVertices,
            std::vector<glm::vec4> &pointColors);

    /**
     * Get all surface face triangles including barycentric coordinates as attributes.
     * The points with alpha_0 = 0 are assumed to lie on the edge inside of the face.
     *
     * (0,1,0)     (1,0,0)
     *  | - - - - - |
     *  | \         |
     *  |   \       |
     *  |     \     |
     *  |       \   |
     *  |         \ |
     *  | - - - - - |
     * (1,0,0)     (0,0,1)
     */
    void getSurfaceDataBarycentric(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertexPositions,
            std::vector<glm::vec4>& vertexColors,
            std::vector<glm::vec3>& barycentricCoordinates);

    /**
     * Get all surface face triangles in a special raw format as defined in the shader "WireframeSurface.glsl".
     *
     * vertex 1     edge 1    vertex 2
     *          | - - - - - |
     *          | \         |
     *          |   \       |
     *   edge 0 |     \     | edge 2
     *          |       \   |
     *          |         \ |
     *          | - - - - - |
     * vertex 0     edge 3    vertex 3
     */
    void getSurfaceDataWireframeFaces(
            std::vector<uint32_t>& indices,
            std::vector<HexahedralCellFace>& hexahedralCellFaces,
            bool useGlowColors = false);

private:
    void rebuildInternalRepresentationIfNecessary();

    /**
     * Updates the ray-mesh intersection data structure for a newly loaded mesh.
     */
    void updateMeshTriangleIntersectionDataStructure();

    // Base-complex computations
    /**
     * Converts the passed vertices and cell indices (8*num_cells) of a hexahedral mesh to its singularity and
     * base-complex representation. The data is stored in baseComplexMesh, si and frame, respectively.
     * @param vertices The hexahedral mesh vertices
     * @param cellIndices Cell vertex indices (8*num_cells).
     */
    void computeBaseComplexMesh(
            const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices);
    void computeBaseComplexMeshFrame();
    /**
     * A helper function for computeBaseComplexParametrizedGrid.
     * It infers the vertex belonging to the parameters encoded in idxShared by using the vertices parametrized by
     * idx0 and idx1. The indices are converted to unsigned integers from their (u,v,w) representation by using either
     * PARAM_IDX_LIST or PARAM_IDX_VEC.
     * The function assumes that the vertex that is an unvisited neighbor to both the vertices corresponding to idx0 and
     * idx1 and is exactly parametrized by idxShared. For determining the neighbors, it uses set intersection
     * operations.
     *
     * @param idx0 The compressed parametrization of vertex 0.
     * @param idx1 The compressed parametrization of vertex 1.
     * @param idxShared The compressed parametrization of the shared, not yet unvisited neighbor of vertex 0 and 1.
     * @param partitionParam The mapping compressed parametrization index -> vertex ID.
     * @param visitedVertices A set of already visited and parametrized vertices.
     * @return False if an error occured. True otherwise.
     */
    bool indexShared(
            uint32_t idx0, uint32_t idx1, uint32_t idxShared, uint32_t* partitionParam,
            std::unordered_set<uint32_t>& visitedVertices);
    /**
     * Computes/extracts the parametrized curvilinear grid components from the base-complex.
     * @return The list of grid components.
     */
    std::vector<ParametrizedGrid> computeBaseComplexParametrizedGrid();

    // Helpers for getLodLineRepresentationClosest
    void addEdgeToLodRenderData(
            const glm::ivec3 ptIdx0, const glm::ivec3 ptIdx1, ParametrizedGrid& grid,
            std::vector<glm::vec3>& lineVertices, std::vector<glm::vec4>& lineColors,
            std::unordered_set<uint64_t>& addedEdgeSet, int level, int numLevels);
    void getListOfOctreeEdges(
            OctreeNode& octree, ParametrizedGrid& grid,
            std::vector<glm::vec3>& lineVertices, std::vector<glm::vec4>& lineColors,
            std::unordered_set<uint64_t>& addedEdgeSet, float focusRadius, int level, int numLevels);

    // Helpers for getCompleteWireframeTubeData.
    Hybrid_E* pickNextUnvisitedNeighbor(std::unordered_set<uint32_t>& visitedEdgeIds, Hybrid_E& e, uint32_t v_id);


    void recomputeHistogram();
    QualityMeasure qualityMeasure = QUALITY_MEASURE_SCALED_JACOBIAN;
    TransferFunctionWindow &transferFunctionWindow;
    RayMeshIntersection& rayMeshIntersection;
    bool dirty = false;

    // HexaLab data
    HexaLab::App* hexaLabApp = nullptr;
    HexaLab::QualityMeasureEnum hexaLabQualityMeasure;

    // Base-complex data
    Mesh* baseComplexMesh = nullptr;
    Singularity* si = nullptr;
    Frame* frame = nullptr;
};

#endif //GENERALMAP_GENERALMAP_HPP
