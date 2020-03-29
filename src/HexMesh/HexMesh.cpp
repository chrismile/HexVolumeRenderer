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


#include <algorithm>
#include <functional>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/color_space.hpp>

#include <Utils/File/Logfile.hpp>
#include <Utils/Random/Xorshift.hpp>

#include "HexaLab/hex_quality.h"
#include "HexaLab/hex_quality_color_maps.h"
#include "HexaLab/mesh.h"
#include "HexaLab/mesh_navigator.h"

#include "BaseComplex/base_complex.h"

#include "HexMesh.hpp"

static const glm::vec4 glowColorRegular = glm::vec4(0.0f, 0.5f, 0.2f, 1.0f);
static const glm::vec4 glowColorSingular = glm::vec4(0.8f, 0.1f, 0.1f, 1.0f);
static const glm::vec4 outlineColorRegular = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
static const glm::vec4 outlineColorSingular = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

HexMesh::~HexMesh() {
    if (hexaLabApp != nullptr) {
        delete hexaLabApp;
        hexaLabApp = nullptr;
    }
    if (baseComplexMesh != nullptr) {
        delete baseComplexMesh;
        baseComplexMesh = nullptr;
        delete si;
        si = nullptr;
        delete frame;
        frame = nullptr;
    }
}

void HexMesh::setHexMeshData(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices) {
    if (hexaLabApp != nullptr) {
        delete hexaLabApp;
        hexaLabApp = nullptr;
    }
    if (baseComplexMesh != nullptr) {
        delete baseComplexMesh;
        baseComplexMesh = nullptr;
    }
    if (si != nullptr) {
        delete si;
        si = nullptr;
    }
    if (frame != nullptr) {
        delete frame;
        frame = nullptr;
    }

    hexaLabApp = new HexaLab::App(transferFunctionWindow);
    std::vector<HexaLab::Index> indices;
    for (const uint32_t& idx : cellIndices) {
        indices.push_back(idx);
    }
    hexaLabApp->import_mesh(vertices, indices);
    setQualityMeasure(qualityMeasure);

    baseComplexMesh = new Mesh;
    si = new Singularity;
    frame = new Frame;
    computeBaseComplexMesh(vertices, cellIndices);

    dirty = true;
}

void HexMesh::computeBaseComplexMesh(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices) {
    Mesh &mesh = *baseComplexMesh;
    const uint32_t numVertices = vertices.size();
    const uint32_t numCells = cellIndices.size() / 8;

    mesh.type = Mesh_type::Hex;
    mesh.V.resize(3, numVertices);
    mesh.V.setZero();
    mesh.Vs.resize(numVertices);

    for (uint32_t i = 0; i < numVertices; i++) {
        const glm::vec3 &vertexPosition = vertices.at(i);
        mesh.V(0, i) = vertexPosition.x;
        mesh.V(1, i) = vertexPosition.y;
        mesh.V(2, i) = vertexPosition.z;

        Hybrid_V v;
        v.id = i;
        v.boundary = false;
        mesh.Vs[i] = v;
    }

    mesh.Hs.resize(numCells);
    Hybrid h;
    h.vs.resize(8);
    for (uint32_t i = 0; i < numCells; i++) {
        h.id = i;
        for (int vertIdx = 0; vertIdx < 8; vertIdx++) {
            uint32_t vertexIndex = cellIndices.at(i * 8 + vertIdx);
            h.vs[vertIdx] = vertexIndex;
            mesh.Vs[vertexIndex].neighbor_hs.push_back(h.id);
        }
        mesh.Hs[h.id] = h;
    }

    build_connectivity(mesh);
    base_complex bc;
    bc.singularity_structure(*si, mesh);
}

void HexMesh::computeBaseComplexMeshFrame() {
    base_complex bc;
    bc.base_complex_extraction(*si, *frame, *baseComplexMesh);

    // Set the singularity attribute on the frame vertices.
    std::unordered_set<uint32_t> singularVertexSet;
    for (size_t i = 0; i < si->SVs.size(); i++) {
        Singular_V &sv = si->SVs.at(i);
        singularVertexSet.insert(sv.hid);
    }

    for (Frame_V &fv : frame->FVs) {
        fv.singular = (singularVertexSet.find(fv.hid) != singularVertexSet.end());
    }
}



void HexMesh::onTransferFunctionMapRebuilt() {
    dirty = true;
    hexaLabApp->onTransferFunctionMapRebuilt();
}

void HexMesh::unmark() {
    hexaLabApp->mesh->unmark_all();
    dirty = true;
}

void HexMesh::setQualityMeasure(QualityMeasure qualityMeasure) {
    this->qualityMeasure = qualityMeasure;
    switch (qualityMeasure) {
        case QUALITY_MEASURE_SCALED_JACOBIAN:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::SJ;
            break;
        case QUALITY_MEASURE_EDGE_RATIO:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::ER;
            break;
        case QUALITY_MEASURE_DIAGONAL:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::DIA;
            break;
        case QUALITY_MEASURE_DIMENSION:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::DIM;
            break;
        case QUALITY_MEASURE_DISTORTION:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::DIS;
            break;
        case QUALITY_MEASURE_JACOBIAN:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::J;
            break;
        case QUALITY_MEASURE_MAX_EDGE_RATIO:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::MER;
            break;
        case QUALITY_MEASURE_MAX_ASPECT_FROBENIUS:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::MAAF;
            break;
        case QUALITY_MEASURE_MEAN_ASPECT_FROBENIUS:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::MEAF;
            break;
        case QUALITY_MEASURE_ODDY:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::ODD;
            break;
        case QUALITY_MEASURE_RELATIVE_SIZE_SQUARED:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::RSS;
            break;
        case QUALITY_MEASURE_SHAPE:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::SHA;
            break;
        case QUALITY_MEASURE_SHAPE_AND_SIZE:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::SHAS;
            break;
        case QUALITY_MEASURE_SHEAR:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::SHE;
            break;
        case QUALITY_MEASURE_SHEAR_AND_SIZE:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::SHES;
            break;
        case QUALITY_MEASURE_SKEW:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::SKE;
            break;
        case QUALITY_MEASURE_STRETCH:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::STR;
            break;
        case QUALITY_MEASURE_TAPER:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::TAP;
            break;
        case QUALITY_MEASURE_VOLUME:
            hexaLabQualityMeasure = HexaLab::QualityMeasureEnum::VOL;
            break;
    }

    hexaLabApp->set_quality_measure(hexaLabQualityMeasure);
    recomputeHistogram();
    dirty = true;
}

void HexMesh::recomputeHistogram() {
    std::vector<float> qualityMeasures = *hexaLabApp->get_normalized_hexa_quality();
    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;
    #pragma omp parallel for reduction(min:minValue) reduction(max:maxValue)
    for (size_t i = 0; i < qualityMeasures.size(); i++) {
        qualityMeasures.at(i) = 1.0f - qualityMeasures.at(i);
        minValue = std::min(minValue, qualityMeasures.at(i));
        maxValue = std::max(maxValue, qualityMeasures.at(i));
    }
    transferFunctionWindow.computeHistogram(qualityMeasures, minValue, maxValue);
}

HexaLab::Mesh& HexMesh::getMesh() {
    return *hexaLabApp->get_mesh();
}

void HexMesh::rebuildInternalRepresentationIfNecessary() {
    if (dirty) {
        hexaLabApp->update_models();
        updateMeshTriangleIntersectionDataStructure();
        dirty = false;
    }
}

void HexMesh::updateMeshTriangleIntersectionDataStructure() {
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices;
    indices.reserve(hexaLabApp->get_visible_model()->mesh_ibuffer.size());
    for (HexaLab::Index& idx : hexaLabApp->get_visible_model()->mesh_ibuffer) {
        indices.push_back(idx);
    }
    vertices = hexaLabApp->get_visible_model()->mesh_vert_pos;
    /*indices.reserve(hexaLabApp->get_visible_model()->surface_ibuffer.size());
    for (HexaLab::Index& idx : hexaLabApp->get_visible_model()->surface_ibuffer) {
        indices.push_back(idx);
    }
    vertices = hexaLabApp->get_visible_model()->surface_vert_pos;*/
    rayMeshIntersection.setMeshTriangleData(vertices, indices);
}

void HexMesh::getSurfaceData(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec3>& normals,
        std::vector<glm::vec4>& colors) {
    rebuildInternalRepresentationIfNecessary();
    indices.clear();
    indices.reserve(hexaLabApp->get_visible_model()->surface_ibuffer.size());
    for (HexaLab::Index& idx : hexaLabApp->get_visible_model()->surface_ibuffer) {
        indices.push_back(idx);
    }
    vertices = hexaLabApp->get_visible_model()->surface_vert_pos;
    normals = hexaLabApp->get_visible_model()->surface_vert_norm;
    colors = hexaLabApp->get_visible_model()->surface_vert_color;
}

void HexMesh::getWireframeData(
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec4>& colors) {
    rebuildInternalRepresentationIfNecessary();
    vertices = hexaLabApp->get_visible_model()->wireframe_vert_pos;
    colors = hexaLabApp->get_visible_model()->wireframe_vert_color;
}

void HexMesh::getVolumeData_Faces(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec3>& normals,
        std::vector<glm::vec4>& colors) {
    rebuildInternalRepresentationIfNecessary();
    hexaLabApp->get_volume_geometry_faces();
    indices.clear();
    indices.reserve(hexaLabApp->get_visible_model()->mesh_ibuffer.size());
    for (HexaLab::Index& idx : hexaLabApp->get_visible_model()->mesh_ibuffer) {
        indices.push_back(idx);
    }
    vertices = hexaLabApp->get_visible_model()->mesh_vert_pos;
    normals = hexaLabApp->get_visible_model()->mesh_vert_norm;
    colors = hexaLabApp->get_visible_model()->mesh_vert_color;
}

void HexMesh::getVolumeData_Volume(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec3>& normals,
        std::vector<glm::vec4>& colors) {
    rebuildInternalRepresentationIfNecessary();
    hexaLabApp->get_volume_geometry_volume();
    indices.clear();
    indices.reserve(hexaLabApp->get_visible_model()->mesh_ibuffer.size());
    for (HexaLab::Index& idx : hexaLabApp->get_visible_model()->mesh_ibuffer) {
        indices.push_back(idx);
    }
    vertices = hexaLabApp->get_visible_model()->mesh_vert_pos;
    normals = hexaLabApp->get_visible_model()->mesh_vert_norm;
    colors = hexaLabApp->get_visible_model()->mesh_vert_color;
}

void HexMesh::getSingularityData(
        std::vector<glm::vec3>& lineVertices,
        std::vector<glm::vec4>& lineColors,
        std::vector<glm::vec3>& pointVertices,
        std::vector<glm::vec4>& pointColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;

    for (size_t i = 0; i < si->SVs.size(); i++) {
        Singular_V& sv = si->SVs.at(i);
        glm::vec3 vertexPosition(mesh->V(0, sv.hid), mesh->V(1, sv.hid), mesh->V(2, sv.hid));
        pointVertices.push_back(vertexPosition);
        pointColors.push_back(glm::vec4(1,0,0,1));
    }

    for (size_t i = 0; i < si->SEs.size(); i++) {
        Singular_E& se = si->SEs.at(i);
        for (size_t edgeIndex = 0; edgeIndex < se.es_link.size(); edgeIndex++) {
            Hybrid_E& e = mesh->Es[se.es_link.at(edgeIndex)];
            for (uint32_t v_id : e.vs) {
                glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                lineVertices.push_back(vertexPosition);
                lineColors.push_back(glm::vec4(1,0,0,1));
            }
        }
    }
}

void HexMesh::getBaseComplexDataWireframe(
        std::vector<glm::vec3>& lineVertices,
        std::vector<glm::vec4>& lineColors,
        std::vector<glm::vec3>& pointVertices,
        std::vector<glm::vec4>& pointColors,
        bool drawRegularLines) {
    rebuildInternalRepresentationIfNecessary();
    if (!frame) computeBaseComplexMeshFrame();
    Mesh* mesh = baseComplexMesh;

    for (Frame_V& fv : frame->FVs) {
        if (!drawRegularLines && !fv.singular) {
            continue;
        }

        glm::vec4 vertexColor;
        if (fv.singular) {
            vertexColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        } else {
            vertexColor = glm::vec4(0.0f, 0.2f, 1.0f, 1.0f);
        }

        glm::vec3 vertexPosition(mesh->V(0, fv.hid), mesh->V(1, fv.hid), mesh->V(2, fv.hid));
        pointVertices.push_back(vertexPosition);
        pointColors.push_back(vertexColor);
    }

    for (Frame_E& fe : frame->FEs) {
        if (!drawRegularLines && !fe.singular) {
            continue;
        }

        glm::vec4 vertexColor;
        if (fe.singular) {
            vertexColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        } else {
            vertexColor = glm::vec4(0.0f, 0.2f, 1.0f, 1.0f);
        }

        for (size_t lineEdgeIndex = 0; lineEdgeIndex < fe.es_link.size(); lineEdgeIndex++) {
            uint32_t edgeIndex = fe.es_link.at(lineEdgeIndex);
            Hybrid_E& edge = mesh->Es[edgeIndex];
            for (size_t edgeVertexIndex = 0; edgeVertexIndex < edge.vs.size(); edgeVertexIndex++) {
                uint32_t vertexIndex = edge.vs.at(edgeVertexIndex);
                glm::vec3 vertexPosition(mesh->V(0, vertexIndex), mesh->V(1, vertexIndex), mesh->V(2, vertexIndex));
                lineVertices.push_back(vertexPosition);
                lineColors.push_back(vertexColor);
            }
        }
    }
}

void HexMesh::getBaseComplexDataSurface(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec3>& normals,
        std::vector<glm::vec4>& colors,
        bool cullInterior) {
    rebuildInternalRepresentationIfNecessary();
    if (!frame) computeBaseComplexMeshFrame();
    Mesh* mesh = baseComplexMesh;
    sgl::XorshiftRandomGenerator random(10203);

    const int vertexIndices[12] = {
            0, 1, 2, 0, 2, 3,
            0, 2, 1, 0, 3, 2,
    };

    std::vector<glm::vec3> quadBuffer;
    for (size_t partitionIndex = 0; partitionIndex < frame->FHs.size(); partitionIndex++) {
        Frame_H& fh = frame->FHs.at(partitionIndex);

        // Get a random partition color (random seed is fixed).
        float randomVal = random.getRandomFloatBetween(0.0f, 360.0f);
        glm::vec3 hsvVec(randomVal, 1.0f, 0.5f);
        glm::vec3 rgbVec = glm::rgbColor(hsvVec);
        glm::vec4 vertexColor(rgbVec.x, rgbVec.y, rgbVec.z, 1.0f);

        // Iterate over all boundary faces.
        for (uint32_t ff_id : fh.fs) {
            Frame_F& ff = frame->FFs[ff_id];
            for (uint32_t f_id : ff.ffs_net) {
                Hybrid_F& f = mesh->Fs[f_id];
                if (f.boundary || !cullInterior) {
                    assert(f.vs.size() == 4);
                    for (uint32_t v_id : f.vs) {
                        glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                        quadBuffer.push_back(vertexPosition);
                    }
                    glm::vec3 v0 = quadBuffer.at(1) - quadBuffer.at(0);
                    glm::vec3 v1 = quadBuffer.at(2) - quadBuffer.at(0);
                    glm::vec3 vertexNormal = glm::normalize(glm::cross(v0, v1));

                    uint32_t offset = vertices.size();
                    for (int i = 0; i < 12; i++) {
                        indices.push_back(offset + vertexIndices[i]);
                    }
                    for (size_t i = 0; i < f.vs.size(); i++) {
                        vertices.push_back(quadBuffer.at(i));
                        normals.push_back(vertexNormal);
                        colors.push_back(vertexColor);
                    }
                    quadBuffer.clear();
                }
            }
        }
    }
}


#define PARAM_IDX_LIST(u, v, w) ((u) + ((v) + (w)*numVertices[1])*numVertices[0])
#define PARAM_IDX_VEC(v) ((v).x + ((v).y + (v).z*numVertices[1])*numVertices[0])

/**
 * A parametrized, curvilinear grid line either in u, v or w direction.
 */
struct ParameterLine {
    std::vector<glm::vec3> points;
};

/**
 * A parametrized, curvilinear grid. The extracted grid lines in u, v and w direction and the number of vertices in each
 * direction are stored.
 */
struct ParametrizedGrid {
    int numVertices[3];
    std::vector<ParameterLine> wLine, vLine, uLine;

    std::vector<uint32_t> gridVertexIds;
    std::vector<glm::vec3> gridPoints;
    inline glm::vec3& getPoint(int u, int v, int w) {
        return gridPoints.at(PARAM_IDX_LIST(u, v, w));
    }
};

/**
 * Traverses a base-complex frame edge starting from a vertex zero. It stops as soon as another frame corner vertex is
 * visited. The passed callback function is called when a new vertex is visited (excluding vertex zero).
 * @param mesh The hexahedral mesh.
 * @param fh The frame component.
 * @param fe The frame edge.
 * @param zero_v The zero vertex of the parametrization to start at.
 * @param visitCallback The callback to call when a new vertex is visited (excluding vertex zero).
 * The vertex ID and the counter variable are passed to the callback.
     * @return False if an error occured. True otherwise.
 */
bool traverseFrameEdge(Mesh& mesh, Frame_H& fh, Frame_E& fe, Hybrid_V& zero_v,
        std::function<bool(uint32_t, int)> visitCallback) {
    uint32_t last_v_id = zero_v.id;
    Hybrid_V* last_v = &zero_v;
    Hybrid_E* last_e = nullptr;
    std::sort(fe.es_link.begin(), fe.es_link.end());

    // Finde edge containing zero_v.
    for (uint32_t edgeIndex = 0; edgeIndex < fe.es_link.size(); edgeIndex++) {
        Hybrid_E& e = mesh.Es[fe.es_link.at(edgeIndex)];
        if (std::find(e.vs.begin(), e.vs.end(), last_v_id) != e.vs.end()) {
            last_e = &e;
            break;
        }
    }
    if (last_e == nullptr) {
        sgl::Logfile::get()->writeError(
                "Error in traverseFrameEdge: First edge not found.");
        return false;
    }

    // Traverse the frame edge until another frame vertex is hit.
    int i = 1;
    while (true) {
        uint32_t next_v_id;
        Hybrid_V* next_v;
        Hybrid_E* next_e = nullptr;

        // Find the next vertex
        if (last_v_id == last_e->vs.at(0)) {
            next_v_id = last_e->vs.at(1);
        } else {
            next_v_id = last_e->vs.at(0);
        }
        next_v = &mesh.Vs[next_v_id];


        if (!visitCallback(next_v_id, i)) {
            break;
        }

        // Find the neighbor edge the next vertex is part of.
        std::vector<uint32_t> neighborEdges;
        std::sort(next_v->neighbor_es.begin(), next_v->neighbor_es.end());
        std::set_intersection(
                fe.es_link.begin(), fe.es_link.end(),
                next_v->neighbor_es.begin(), next_v->neighbor_es.end(),
                std::back_inserter(neighborEdges));

        for (uint32_t edgeIndex = 0; edgeIndex < neighborEdges.size(); edgeIndex++) {
            Hybrid_E& neighbor_e = mesh.Es[neighborEdges.at(edgeIndex)];
            if (std::find(neighbor_e.vs.begin(), neighbor_e.vs.end(), last_v_id) == neighbor_e.vs.end()) {
                next_e = &neighbor_e;
                break;
            }
        }
        if (next_e == nullptr) {
            sgl::Logfile::get()->writeError(
                    "Error in traverseFrameEdge: Next edge not found.");
            return false;
        }

        last_v_id = next_v_id;
        last_e = next_e;
        i++;
    }

    return true;
}

bool HexMesh::indexShared(
        uint32_t idx0, uint32_t idx1, uint32_t idxShared, uint32_t* partitionParam,
        std::unordered_set<uint32_t>& visitedVertices) {
    uint32_t v_id_0 = partitionParam[idx0];
    uint32_t v_id_1 = partitionParam[idx1];
    Hybrid_V& v_0 = baseComplexMesh->Vs[v_id_0];
    Hybrid_V& v_1 = baseComplexMesh->Vs[v_id_1];
    assert(v_id_0 != v_id_1);

    // Find the common, not yet visited neighbor.
    std::vector<uint32_t> sharedNeighborVerticesAll, sharedNeighborVerticesUnvisited;
    std::sort(v_0.neighbor_vs.begin(), v_0.neighbor_vs.end());
    std::sort(v_1.neighbor_vs.begin(), v_1.neighbor_vs.end());
    std::set_intersection(
            v_0.neighbor_vs.begin(), v_0.neighbor_vs.end(),
            v_1.neighbor_vs.begin(), v_1.neighbor_vs.end(),
            std::back_inserter(sharedNeighborVerticesAll));
    for (uint32_t neighborVertexIndex : sharedNeighborVerticesAll) {
        if (visitedVertices.find(neighborVertexIndex) == visitedVertices.end()) {
            sharedNeighborVerticesUnvisited.push_back(neighborVertexIndex);
        }
    }

    if (sharedNeighborVerticesUnvisited.size() != 1) {
        sgl::Logfile::get()->writeError(
                "Error in HexMesh::indexShared: Invalid number of unvisited neighbors.");
        delete[] partitionParam;
        return false;
    }
    visitedVertices.insert(sharedNeighborVerticesUnvisited.at(0));
    // Add the neighbor to the parametrization.
    partitionParam[idxShared] = sharedNeighborVerticesUnvisited.at(0);
    return true;
}

std::vector<ParametrizedGrid> HexMesh::computeBaseComplexParametrizedGrid() {
    Mesh* mesh = baseComplexMesh;
    std::unordered_set<uint32_t> visitedVertices;
    std::vector<ParametrizedGrid> gridPartitions;

    // Generate a set of grid lines for each curvilinear grid base-complex partition.
    for (size_t partitionIndex = 0; partitionIndex < frame->FHs.size(); partitionIndex++) {
        Frame_H &fh = frame->FHs.at(partitionIndex);
        visitedVertices.clear();

        // TODO: Support also partitions topologically equivalent to a torus instead of a cube?
        if (fh.vs.size() != 8) {
            sgl::Logfile::get()->writeError(
                    "Error in HexMesh::computeBaseComplexParametrizedGrid: Partition is not cubic.");
            return gridPartitions;
        }

        // Pick a singular corner point from the partition.
        Frame_V &zero_fv = frame->FVs.at(fh.vs[0]);
        Hybrid_V &zero_v = mesh->Vs.at(zero_fv.hid);

        // Get all neighboring frame edges in the partition spanning the three parametrization unit directions.
        std::vector<uint32_t> neighborFEsInPartition;
        std::sort(fh.es.begin(), fh.es.end());
        std::sort(zero_fv.neighbor_fes.begin(), zero_fv.neighbor_fes.end());
        std::set_intersection(
                fh.es.begin(), fh.es.end(), // All frame edges in the partition
                zero_fv.neighbor_fes.begin(), zero_fv.neighbor_fes.end(), // Neighboring frame edges to vertex zero
                std::back_inserter(neighborFEsInPartition));
        if (neighborFEsInPartition.size() != 3) {
            sgl::Logfile::get()->writeError(
                    "Error in HexMesh::computeBaseComplexParametrizedGrid: "
                    "Number of frame edges meeting in corner not 3.");
            return gridPartitions;
        }

        // Create a set of all corner vertex IDs.
        std::unordered_set<uint32_t> cornerVs;
        for (uint32_t cornerFrameId : fh.vs) {
            cornerVs.insert(frame->FVs[cornerFrameId].hid);
        }


        // Determine the dimensions of the grid in the three unit directions.
        int numVertices[3];
        for (int dim = 0; dim < 3; dim++) {
            Frame_E &fe = frame->FEs.at(neighborFEsInPartition.at(dim));
            traverseFrameEdge(
                    *mesh, fh, fe, zero_v,
                    [&](uint32_t v_id, uint32_t i) {
                        if (cornerVs.find(v_id) != cornerVs.end()) {
                            numVertices[dim] = i + 1;
                            return false;
                        }
                        return true;
                    });
        }

        // Allocate the partition parametrization look-up array.
        uint32_t *partitionParam = new uint32_t[numVertices[0] * numVertices[1] * numVertices[2]];

        // Number the origin point.
        partitionParam[PARAM_IDX_LIST(0, 0, 0)] = zero_v.id;
        visitedVertices.insert(zero_v.id);

        // Number the three unit directions.
        for (int dim = 0; dim < 3; dim++) {
            Frame_E &fe = frame->FEs.at(neighborFEsInPartition.at(dim));
            traverseFrameEdge(
                    *mesh, fh, fe, zero_v,
                    [&](uint32_t v_id, uint32_t i) {
                        glm::ivec3 idx(0, 0, 0);
                        idx[dim] = i;
                        partitionParam[PARAM_IDX_VEC(idx)] = v_id;
                        visitedVertices.insert(v_id);
                        if (cornerVs.find(v_id) != cornerVs.end()) {
                            return false;
                        }
                        return true;
                    });
        }

        // Number the vertices on the three boundary faces containing the origin.
        for (int v = 1; v < numVertices[1]; v++) {
            for (int u = 1; u < numVertices[0]; u++) {
                if (!indexShared(
                        PARAM_IDX_LIST(u - 1, v, 0), PARAM_IDX_LIST(u, v - 1, 0), PARAM_IDX_LIST(u, v, 0),
                        partitionParam, visitedVertices)) {
                    return gridPartitions;
                }
            }
        }
        for (int w = 1; w < numVertices[2]; w++) {
            for (int u = 1; u < numVertices[0]; u++) {
                if (!indexShared(
                        PARAM_IDX_LIST(u - 1, 0, w), PARAM_IDX_LIST(u, 0, w - 1), PARAM_IDX_LIST(u, 0, w),
                        partitionParam, visitedVertices)) {
                    return gridPartitions;
                }
            }
        }
        for (int w = 1; w < numVertices[2]; w++) {
            for (int v = 1; v < numVertices[1]; v++) {
                if (!indexShared(
                        PARAM_IDX_LIST(0, v - 1, w), PARAM_IDX_LIST(0, v, w - 1), PARAM_IDX_LIST(0, v, w),
                        partitionParam, visitedVertices)) {
                    return gridPartitions;
                }
            }
        }

        // Number the inner vertices in the interior of the partition.
        for (int w = 1; w < numVertices[2]; w++) {
            for (int v = 1; v < numVertices[1]; v++) {
                for (int u = 1; u < numVertices[0]; u++) {
                    if (!indexShared(
                            PARAM_IDX_LIST(u - 1, v, w), PARAM_IDX_LIST(u, v - 1, w), PARAM_IDX_LIST(u, v, w),
                            partitionParam, visitedVertices)) {
                        return gridPartitions;
                    }
                }
            }
        }

        // Get the list of lists in w, v, and u direction.
        ParametrizedGrid grid;
        for (int dim = 0; dim < 3; dim++) {
            grid.numVertices[dim] = numVertices[dim];
        }
        grid.wLine.resize(numVertices[0] * numVertices[1]);
        grid.vLine.resize(numVertices[0] * numVertices[2]);
        grid.uLine.resize(numVertices[1] * numVertices[2]);
        for (int w = 0; w < numVertices[2]; w++) {
            for (int v = 0; v < numVertices[1]; v++) {
                for (int u = 0; u < numVertices[0]; u++) {
                    uint32_t v_id = partitionParam[PARAM_IDX_LIST(u, v, w)];
                    glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                    grid.uLine.at(v + w * numVertices[1]).points.push_back(vertexPosition);
                    grid.vLine.at(u + w * numVertices[0]).points.push_back(vertexPosition);
                    grid.wLine.at(u + v * numVertices[0]).points.push_back(vertexPosition);
                }
            }
        }
        grid.gridPoints.resize(numVertices[0] * numVertices[1] * numVertices[2]);
        grid.gridVertexIds.resize(numVertices[0] * numVertices[1] * numVertices[2]);
        for (int w = 0; w < numVertices[2]; w++) {
            for (int v = 0; v < numVertices[1]; v++) {
                for (int u = 0; u < numVertices[0]; u++) {
                    uint32_t v_id = partitionParam[PARAM_IDX_LIST(u, v, w)];
                    glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                    grid.gridPoints.at(PARAM_IDX_LIST(u, v, w)) = vertexPosition;
                    grid.gridVertexIds.at(PARAM_IDX_LIST(u, v, w)) = v_id;
                }
            }
        }
        gridPartitions.push_back(grid);

        // Cleanup.
        delete[] partitionParam;
    }

    return gridPartitions;
}


/**
 * Helper function for getColoredPartitionLines.
 * @param u Parameter #0.
 * @param v Parameter #1.
 * @param w Parameter #2.
 * @param numVertices Number of vertices in each of the three parameter domain directions.
 * @return The color with (R, G, B, A) = (u/u_max, v/v_max, w/w_max, 1).
 */
inline glm::vec4 parametersToColor(int u, int v, int w, int *numVertices) {
    return glm::vec4(
            float(u) / (numVertices[0]-1),
            float(v) / (numVertices[1]-1),
            float(w) / (numVertices[2]-1),
            1.0f);
}

void HexMesh::getColoredPartitionLines(
        std::vector<glm::vec3>& lineVertices,
        std::vector<glm::vec4>& lineColors) {
    rebuildInternalRepresentationIfNecessary();
    if (!frame) computeBaseComplexMeshFrame();
    std::vector<ParametrizedGrid> gridPartitions = computeBaseComplexParametrizedGrid();

    for (ParametrizedGrid& grid : gridPartitions) {
        // Add the grid lines to the rendering data.
        for (int w = 0; w < grid.numVertices[2]; w++) {
            for (int v = 0; v < grid.numVertices[1]; v++) {
                for (int u = 0; u < grid.numVertices[0] - 1; u++) {
                    glm::vec3 vertexPosition0 = grid.uLine.at(v + w*grid.numVertices[1]).points.at(u);
                    glm::vec3 vertexPosition1 = grid.uLine.at(v + w*grid.numVertices[1]).points.at(u+1);
                    lineVertices.push_back(vertexPosition0);
                    lineVertices.push_back(vertexPosition1);
                    lineColors.push_back(parametersToColor(u,   v, w, grid.numVertices));
                    lineColors.push_back(parametersToColor(u+1, v, w, grid.numVertices));
                }
            }
        }
        for (int w = 0; w < grid.numVertices[2]; w++) {
            for (int u = 0; u < grid.numVertices[0]; u++) {
                for (int v = 0; v < grid.numVertices[1] - 1; v++) {
                    glm::vec3 vertexPosition0 = grid.vLine.at(u + w*grid.numVertices[0]).points.at(v);
                    glm::vec3 vertexPosition1 = grid.vLine.at(u + w*grid.numVertices[0]).points.at(v+1);
                    lineVertices.push_back(vertexPosition0);
                    lineVertices.push_back(vertexPosition1);
                    lineColors.push_back(parametersToColor(u, v,   w, grid.numVertices));
                    lineColors.push_back(parametersToColor(u, v+1, w, grid.numVertices));
                }
            }
        }
        for (int v = 0; v < grid.numVertices[1]; v++) {
            for (int u = 0; u < grid.numVertices[0]; u++) {
                for (int w = 0; w < grid.numVertices[2] - 1; w++) {
                    glm::vec3 vertexPosition0 = grid.wLine.at(u + v*grid.numVertices[0]).points.at(w);
                    glm::vec3 vertexPosition1 = grid.wLine.at(u + v*grid.numVertices[0]).points.at(w+1);
                    lineVertices.push_back(vertexPosition0);
                    lineVertices.push_back(vertexPosition1);
                    lineColors.push_back(parametersToColor(u, v, w,   grid.numVertices));
                    lineColors.push_back(parametersToColor(u, v, w+1, grid.numVertices));
                }
            }
        }
    }
}


/**
 * Helper function for getLodRepresentation. Assigns LOD values recursively in an 1D array by bisection.
 * @param lodValues The LOD value array.
 * @param minIndex The minimum index handled in this recursion step (inclusive).
 * @param maxIndex The maximum index handled in this recursion step (inclusive).
 * @param recursionNumber The recursion index. Starting at 1 and incremented each recursion.
 */
void assignSubdivisionLodValues(std::vector<float>& lodValues, int minIndex, int maxIndex, int recursionNumber) {
    if (maxIndex - minIndex < 0) {
        return;
    }

    int midIndex = (minIndex + maxIndex) / 2;
    lodValues.at(midIndex) = float(recursionNumber);
    assignSubdivisionLodValues(lodValues, minIndex, midIndex - 1, recursionNumber + 1);
    assignSubdivisionLodValues(lodValues, midIndex + 1, maxIndex, recursionNumber + 1);
}

void HexMesh::getLodLineRepresentation(
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        std::vector<float> &lineLodValues,
        bool previewColors) {
    rebuildInternalRepresentationIfNecessary();
    if (!frame) computeBaseComplexMeshFrame();

    // Get a list of all parametrized base-complex grids.
    std::vector<ParametrizedGrid> gridPartitions = computeBaseComplexParametrizedGrid();

    // Create the three LOD base arrays for each parametrization direction for all grids.
    std::vector<std::vector<std::vector<float>>> lodValuesAllGrids;
    lodValuesAllGrids.resize(gridPartitions.size());
    #pragma omp parallel for
    for (size_t gridIdx = 0; gridIdx < gridPartitions.size(); gridIdx++) {
        std::vector<std::vector<float>>& lodValuesAllDirections = lodValuesAllGrids.at(gridIdx);
        lodValuesAllDirections.resize(3);
        ParametrizedGrid& grid = gridPartitions.at(gridIdx);
        for (int dim = 0; dim < 3; dim++) {
            std::vector<float>& lodValues = lodValuesAllDirections.at(dim);
            lodValues.resize(grid.numVertices[dim], 0.0f);
            assignSubdivisionLodValues(lodValues, 0, int(lodValues.size()) - 1, 1);
            lodValues.front() = 0;
            lodValues.back() = 0;
        }
    }

    // We want to normalize the LOD values to the range [0, 1]. First, compute the maximum value.
    float maxValue = 1.0f;
    #pragma omp parallel for reduction(max: maxValue)
    for (size_t gridIdx = 0; gridIdx < gridPartitions.size(); gridIdx++) {
        std::vector<std::vector<float>>& lodValuesAllDirections = lodValuesAllGrids.at(gridIdx);
        for (int dim = 0; dim < 3; dim++) {
            std::vector<float>& lodValues = lodValuesAllDirections.at(dim);
            for (size_t i = 0; i < lodValues.size(); i++) {
                maxValue = std::max(maxValue, lodValues.at(i));
            }
        }
    }

    // Now, normalize the values by division.
    #pragma omp parallel for
    for (size_t gridIdx = 0; gridIdx < gridPartitions.size(); gridIdx++) {
        std::vector<std::vector<float>>& lodValuesAllDirections = lodValuesAllGrids.at(gridIdx);
        for (int dim = 0; dim < 3; dim++) {
            std::vector<float>& lodValues = lodValuesAllDirections.at(dim);
            for (size_t i = 0; i < lodValues.size(); i++) {
                lodValues.at(i) /= maxValue;
            }
        }
    }

    // Now, add all grid lines with the corresponding LOD values to the rendering data.
    // For combining LOD values from two dimension, the maximum operator is used in order to thin more lines out.
    for (size_t gridIdx = 0; gridIdx < gridPartitions.size(); gridIdx++) {
        ParametrizedGrid& grid = gridPartitions.at(gridIdx);
        std::vector<std::vector<float>>& lodValuesAllDirections = lodValuesAllGrids.at(gridIdx);

        // Add the grid lines to the rendering data.
        for (int w = 0; w < grid.numVertices[2]; w++) {
            for (int v = 0; v < grid.numVertices[1]; v++) {
                float lodValue = std::max(lodValuesAllDirections.at(1).at(v), lodValuesAllDirections.at(2).at(w));
                for (int u = 0; u < grid.numVertices[0] - 1; u++) {
                    glm::vec3 vertexPosition0 = grid.uLine.at(v + w*grid.numVertices[1]).points.at(u);
                    glm::vec3 vertexPosition1 = grid.uLine.at(v + w*grid.numVertices[1]).points.at(u+1);
                    lineVertices.push_back(vertexPosition0);
                    lineVertices.push_back(vertexPosition1);
                    lineLodValues.push_back(lodValue);
                    lineLodValues.push_back(lodValue);
                }
            }
        }
        for (int w = 0; w < grid.numVertices[2]; w++) {
            for (int u = 0; u < grid.numVertices[0]; u++) {
                float lodValue = std::max(lodValuesAllDirections.at(0).at(u), lodValuesAllDirections.at(2).at(w));
                for (int v = 0; v < grid.numVertices[1] - 1; v++) {
                    glm::vec3 vertexPosition0 = grid.vLine.at(u + w*grid.numVertices[0]).points.at(v);
                    glm::vec3 vertexPosition1 = grid.vLine.at(u + w*grid.numVertices[0]).points.at(v+1);
                    lineVertices.push_back(vertexPosition0);
                    lineVertices.push_back(vertexPosition1);
                    lineLodValues.push_back(lodValue);
                    lineLodValues.push_back(lodValue);
                }
            }
        }
        for (int v = 0; v < grid.numVertices[1]; v++) {
            for (int u = 0; u < grid.numVertices[0]; u++) {
                float lodValue = std::max(lodValuesAllDirections.at(0).at(u), lodValuesAllDirections.at(1).at(v));
                for (int w = 0; w < grid.numVertices[2] - 1; w++) {
                    glm::vec3 vertexPosition0 = grid.wLine.at(u + v*grid.numVertices[0]).points.at(w);
                    glm::vec3 vertexPosition1 = grid.wLine.at(u + v*grid.numVertices[0]).points.at(w+1);
                    lineVertices.push_back(vertexPosition0);
                    lineVertices.push_back(vertexPosition1);
                    lineLodValues.push_back(lodValue);
                    lineLodValues.push_back(lodValue);
                }
            }
        }
    }

    lineColors.reserve(lineLodValues.size());
    for (size_t i = 0; i < lineLodValues.size(); i += 2) {
        float lodValue = lineLodValues.at(i) * maxValue;
        glm::vec4 vertexColor(0.6f, 0.0f, 0.0f, 1.0f);
        if (lodValue > 0.0001) {
            float interpolationFactor = (lodValue - 1.0f) / (maxValue - 1.0f);
            vertexColor = glm::vec4(
                    glm::mix(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.8f, 0.8f, 0.8f), interpolationFactor), 1.0f);
        }
        lineColors.push_back(vertexColor);
        lineColors.push_back(vertexColor);
    }
}


/**
 * Represents an octree for subdividing a base-complex partition into a coarser and coarser levels.
 * It is used to get a partly-refined representation of the mesh close to a focus point by culling octree nodes that
 * are very far away to the focus point.
 */
struct OctreeNode {
    OctreeNode(glm::ivec3 minRange, glm::ivec3 maxRange) : minRange(minRange), maxRange(maxRange) {}
    glm::ivec3 minRange; ///< Minimum parametrized grid vertex index in the octree node (inclusive).
    glm::ivec3 maxRange; ///< Maximum parametrized grid vertex index in the octree node (inclusive).
    float minDist = FLT_MAX; ///< The (minimum) distance to the focus region.
    std::vector<OctreeNode> children;
};

/**
 * Encodes an edge defined by two vertex IDs in an order-independent way.
 * @param ptIdx0 The vertex ID of the first point forming the edge.
 * @param ptIdx1 The vertex ID of the second point forming the edge.
 * @return
 */
inline uint64_t makeEdge(uint32_t ptIdx0, uint32_t ptIdx1) {
    // Pack the two IDs into one 64-bit value, making sure the smaller one is always in the least significant bytes.
    uint32_t ptA = std::min(ptIdx0, ptIdx1);
    uint32_t ptB = std::max(ptIdx0, ptIdx1);
    return uint64_t(ptA) | (uint64_t(ptB) << 32ull);
}

void addChildIfNonEmpty(OctreeNode& parent, const OctreeNode& child) {
    // Only add if we have at least one cell (i.e., two vertices in all directions).
    if (glm::any(glm::lessThan(child.maxRange - child.minRange, glm::ivec3(1)))) {
        return;
    }
    parent.children.push_back(child);
}

void subdivideEmptyOctree(OctreeNode& octree, int level, int& numLevels) {
    numLevels = std::max(numLevels, level + 1);

    glm::ivec3 minRange = octree.minRange;
    glm::ivec3 maxRange = octree.maxRange;
    glm::ivec3 midRange = (minRange + maxRange) / 2;
    // Only add up to 8 children if we are not at a leaf node with one element.
    if (glm::any(glm::greaterThan(maxRange - minRange, glm::ivec3(1)))) {
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(minRange.x, minRange.y, minRange.z),
                glm::ivec3(midRange.x, midRange.y, midRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(midRange.x, minRange.y, minRange.z),
                glm::ivec3(maxRange.x, midRange.y, midRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(minRange.x, midRange.y, minRange.z),
                glm::ivec3(midRange.x, maxRange.y, midRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(midRange.x, midRange.y, minRange.z),
                glm::ivec3(maxRange.x, maxRange.y, midRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(minRange.x, minRange.y, midRange.z),
                glm::ivec3(midRange.x, midRange.y, maxRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(midRange.x, minRange.y, midRange.z),
                glm::ivec3(maxRange.x, midRange.y, maxRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(minRange.x, midRange.y, midRange.z),
                glm::ivec3(midRange.x, maxRange.y, maxRange.z)));
        addChildIfNonEmpty(octree, OctreeNode(
                glm::ivec3(midRange.x, midRange.y, midRange.z),
                glm::ivec3(maxRange.x, maxRange.y, maxRange.z)));
    }

    for (OctreeNode& child : octree.children) {
        subdivideEmptyOctree(child, level + 1, numLevels);
    }
}

void computeOctreeDistances(OctreeNode& octree, ParametrizedGrid& grid, std::vector<float>& gridPointDistances) {
    // If we are at a leaf node: Compute the distance to the focus point as the minimum distance of all corner points.
    if (octree.children.size() == 0) {
        assert(glm::all(glm::equal(octree.maxRange - octree.minRange, glm::ivec3(1))));
        int* numVertices = grid.numVertices;
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.minRange.x, octree.minRange.y, octree.minRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.maxRange.x, octree.minRange.y, octree.minRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.minRange.x, octree.maxRange.y, octree.minRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.maxRange.x, octree.maxRange.y, octree.minRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.minRange.x, octree.minRange.y, octree.maxRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.maxRange.x, octree.minRange.y, octree.maxRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.minRange.x, octree.maxRange.y, octree.maxRange.z))));
        octree.minDist = std::min(
                octree.minDist, gridPointDistances.at(PARAM_IDX_VEC(glm::ivec3(
                        octree.maxRange.x, octree.maxRange.y, octree.maxRange.z))));
        return;
    }

    // Compute the distance of a parent node to the focus point as the minimum distance of all child nodes.
    for (OctreeNode& child : octree.children) {
        computeOctreeDistances(child, grid, gridPointDistances);
        octree.minDist = std::min(octree.minDist, child.minDist);
    }
}

void HexMesh::addEdgeToLodRenderData(
        const glm::ivec3 ptIdx0, const glm::ivec3 ptIdx1, ParametrizedGrid& grid,
        std::vector<glm::vec3>& lineVertices, std::vector<glm::vec4>& lineColors,
        std::unordered_set<uint64_t>& addedEdgeSet, int level, int numLevels) {
    int* numVertices = grid.numVertices;
    Mesh* mesh = this->baseComplexMesh;
    uint32_t vertexId0 = grid.gridVertexIds.at(PARAM_IDX_VEC(ptIdx0));
    uint32_t vertexId1 = grid.gridVertexIds.at(PARAM_IDX_VEC(ptIdx1));
    uint64_t vertexPair = makeEdge(vertexId0, vertexId1);
    if (addedEdgeSet.find(vertexPair) != addedEdgeSet.end()) {
        return;
    }
    addedEdgeSet.insert(vertexPair);
    glm::vec3 vertexPosition0(mesh->V(0, vertexId0), mesh->V(1, vertexId0), mesh->V(2, vertexId0));
    glm::vec3 vertexPosition1(mesh->V(0, vertexId1), mesh->V(1, vertexId1), mesh->V(2, vertexId1));

    glm::vec4 vertexColor(1.0f, 0.0f, 0.0f, 1.0f);
    if (level > 0) {
        float interpolationFactor = 0.0f;
        if (numLevels > 2) {
            interpolationFactor = (level - 1.0f) / (numLevels - 2.0f);
        }
        vertexColor = glm::vec4(glm::mix(
                glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.8f, 0.8f, 0.8f), interpolationFactor), 1.0f);
    }

    lineVertices.push_back(vertexPosition0);
    lineVertices.push_back(vertexPosition1);
    lineColors.push_back(vertexColor);
    lineColors.push_back(vertexColor);
}

void HexMesh::getListOfOctreeEdges(
        OctreeNode& octree, ParametrizedGrid& grid,
        std::vector<glm::vec3>& lineVertices, std::vector<glm::vec4>& lineColors,
        std::unordered_set<uint64_t>& addedEdgeSet, float focusRadius, int level, int numLevels) {
    // Iterate over all border edges of the octree node's LOD level.
    for (int x = octree.minRange.x; x < octree.maxRange.x; x++) {
        addEdgeToLodRenderData(
                glm::ivec3(x, octree.minRange.y, octree.minRange.z),
                glm::ivec3(x + 1, octree.minRange.y, octree.minRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(x, octree.maxRange.y, octree.minRange.z),
                glm::ivec3(x + 1, octree.maxRange.y, octree.minRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(x, octree.minRange.y, octree.maxRange.z),
                glm::ivec3(x + 1, octree.minRange.y, octree.maxRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(x, octree.maxRange.y, octree.maxRange.z),
                glm::ivec3(x + 1, octree.maxRange.y, octree.maxRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
    }
    for (int y = octree.minRange.y; y < octree.maxRange.y; y++) {
        addEdgeToLodRenderData(
                glm::ivec3(octree.minRange.x, y, octree.minRange.z),
                glm::ivec3(octree.minRange.x, y + 1, octree.minRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(octree.maxRange.x, y, octree.minRange.z),
                glm::ivec3(octree.maxRange.x, y + 1, octree.minRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(octree.minRange.x, y, octree.maxRange.z),
                glm::ivec3(octree.minRange.x, y + 1, octree.maxRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(octree.maxRange.x, y, octree.maxRange.z),
                glm::ivec3(octree.maxRange.x, y + 1, octree.maxRange.z),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
    }
    for (int z = octree.minRange.z; z < octree.maxRange.z; z++) {
        addEdgeToLodRenderData(
                glm::ivec3(octree.minRange.x, octree.minRange.y, z),
                glm::ivec3(octree.minRange.x, octree.minRange.y, z + 1),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(octree.maxRange.x, octree.minRange.y, z),
                glm::ivec3(octree.maxRange.x, octree.minRange.y, z + 1),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(octree.minRange.x, octree.maxRange.y, z),
                glm::ivec3(octree.minRange.x, octree.maxRange.y, z + 1),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
        addEdgeToLodRenderData(
                glm::ivec3(octree.maxRange.x, octree.maxRange.y, z),
                glm::ivec3(octree.maxRange.x, octree.maxRange.y, z + 1),
                grid, lineVertices, lineColors, addedEdgeSet, level, numLevels);
    }

    // Only add the refined edges of the child nodes at finer LOD levels if they are in the focus region.
    if (octree.minDist > focusRadius) {
        return;
    }

    for (OctreeNode& child : octree.children) {
        getListOfOctreeEdges(
                child, grid, lineVertices, lineColors, addedEdgeSet,
                focusRadius, level + 1, numLevels);
    }
}

void HexMesh::getLodLineRepresentationClosest(
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        const glm::vec3& focusPoint,
        float focusRadius) {
    rebuildInternalRepresentationIfNecessary();
    if (!frame) computeBaseComplexMeshFrame();

    // Edges only need to be considered once, thus remember which we have already added.
    std::unordered_set<uint64_t> addedEdgeSet;

    // Get a list of all parametrized base-complex grids.
    std::vector<ParametrizedGrid> gridPartitions = computeBaseComplexParametrizedGrid();
    for (size_t gridIdx = 0; gridIdx < gridPartitions.size(); gridIdx++) {
        ParametrizedGrid& grid = gridPartitions.at(gridIdx);
        int* numVertices = grid.numVertices;
        int numVerticesTotal = numVertices[0] * numVertices[1] * numVertices[2];

        // Compute the distances of all grid points to the focus point.
        std::vector<float> gridPointDistances;
        gridPointDistances.resize(numVertices[0] * numVertices[1] * numVertices[2]);
        for (int vertexIdx = 0; vertexIdx < numVerticesTotal; vertexIdx++) {
            gridPointDistances.at(vertexIdx) = glm::length(focusPoint - grid.gridPoints.at(vertexIdx));
        }

        /*
         * Construct the octree, including the distance of all levels of subdivion to the focus point.
         * Then, add the edges of the appropriate levels to the list of rendered lines.
         */
        OctreeNode octree(
                glm::ivec3(0, 0, 0),
                glm::ivec3(numVertices[0] - 1, numVertices[1] - 1, numVertices[2] - 1));
        int numLevels = 0;
        subdivideEmptyOctree(octree, 0, numLevels);
        computeOctreeDistances(octree, grid, gridPointDistances);
        getListOfOctreeEdges(
                octree, grid, lineVertices, lineColors, addedEdgeSet, focusRadius, 0, numLevels);
    }
}



void HexMesh::getCompleteWireframeData(
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    std::unordered_set<uint32_t> singularEdgeIDs;
    for (Singular_E& se : si->SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIDs.insert(e_id);
        }
    }

    for (Hybrid_E& e : mesh->Es) {
        bool isSingular = singularEdgeIDs.find(e.id) != singularEdgeIDs.end();
        for (uint32_t v_id : e.vs) {
            glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            lineVertices.push_back(vertexPosition);
            if (isSingular) {
                lineColors.push_back(singularColor);
            } else {
                lineColors.push_back(regularColor);
            }
        }
    }
}


Hybrid_E* HexMesh::pickNextUnvisitedNeighbor(std::unordered_set<uint32_t>& visitedEdgeIds, Hybrid_E& e, uint32_t v_id) {
    Hybrid_E* neighbor_e = nullptr;
    Hybrid_V& v = baseComplexMesh->Vs.at(v_id);
    for (uint32_t neighbor_e_id : v.neighbor_es) {
        if (neighbor_e_id != e.id && visitedEdgeIds.find(neighbor_e_id) == visitedEdgeIds.end()) {
            neighbor_e = &baseComplexMesh->Es.at(neighbor_e_id);
            break;
        }
    }
    return neighbor_e;
}

void HexMesh::getCompleteWireframeTubeData(
        std::vector<std::vector<glm::vec3>>& lineCentersList,
        std::vector<std::vector<glm::vec4>>& lineColorsList,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    std::unordered_set<uint32_t> visitedEdgeIds;
    std::vector<glm::vec3> lineCenters;
    std::vector<glm::vec4> lineColors;
    for (Singular_E& se : si->SEs) {
        if (se.es_link.size() < 1) continue;

        uint32_t last_v_id, v_id;

        // Find the shared vertex between the first two edges.
        if (se.es_link.size() == 1) {
            v_id = se.es_link.at(0);
        } else {
            std::vector<uint32_t>& verticesEdge0 = mesh->Es.at(se.es_link.at(0)).vs;
            std::vector<uint32_t>& verticesEdge1 = mesh->Es.at(se.es_link.at(1)).vs;
            std::vector<uint32_t> sharedVertices;
            std::sort(verticesEdge0.begin(), verticesEdge0.end());
            std::sort(verticesEdge1.begin(), verticesEdge1.end());
            std::set_intersection(
                    verticesEdge0.begin(), verticesEdge0.end(),
                    verticesEdge1.begin(), verticesEdge1.end(),
                    std::back_inserter(sharedVertices));
            assert(sharedVertices.size() == 1);
            v_id = verticesEdge0.at(0) == sharedVertices.at(0) ? verticesEdge0.at(1) : verticesEdge0.at(0);
        }

        // Add the first vertex
        glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
        lineCenters.push_back(vertexPosition);
        lineColors.push_back(singularColor);

        for (uint32_t e_id : se.es_link) {
            Hybrid_E& e = mesh->Es.at(e_id);
            last_v_id = v_id;
            if (e.vs.at(0) == last_v_id) {
                v_id = e.vs.at(1);
            } else {
                v_id = e.vs.at(0);
            }

            glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            lineCenters.push_back(vertexPosition);
            lineColors.push_back(singularColor);
            visitedEdgeIds.insert(e_id);
        }

        lineCentersList.push_back(lineCenters);
        lineColorsList.push_back(lineColors);
        lineCenters.clear();
        lineColors.clear();
    }

    // Perform greedy matching of pairs of edges and add the regular edges.
    for (Hybrid_E& e : mesh->Es) {
        if (visitedEdgeIds.find(e.id) != visitedEdgeIds.end())
            continue;

        uint32_t v0_id = e.vs.at(0);
        uint32_t v1_id = e.vs.at(1);

        lineColors.push_back(regularColor);
        lineColors.push_back(regularColor);
        visitedEdgeIds.insert(e.id);


        Hybrid_E* current_e;
        uint32_t last_v_id, v_id;

        v_id = e.vs.at(1);
        current_e = pickNextUnvisitedNeighbor(visitedEdgeIds, e, v_id);
        if (!current_e) {
            v_id = e.vs.at(0);
            current_e = pickNextUnvisitedNeighbor(visitedEdgeIds, e, v_id);
            lineCenters.push_back(
                    glm::vec3(mesh->V(0, v1_id), mesh->V(1, v1_id), mesh->V(2, v1_id)));
            lineCenters.push_back(
                    glm::vec3(mesh->V(0, v0_id), mesh->V(1, v0_id), mesh->V(2, v0_id)));
        } else {
            lineCenters.push_back(
                    glm::vec3(mesh->V(0, v0_id), mesh->V(1, v0_id), mesh->V(2, v0_id)));
            lineCenters.push_back(
                    glm::vec3(mesh->V(0, v1_id), mesh->V(1, v1_id), mesh->V(2, v1_id)));
        }

        while (current_e) {
            visitedEdgeIds.insert(current_e->id);
            last_v_id = v_id;

            if (current_e->vs.at(0) == last_v_id) {
                v_id = current_e->vs.at(1);
            } else {
                v_id = current_e->vs.at(0);
            }

            glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            lineCenters.push_back(vertexPosition);
            lineColors.push_back(regularColor);

            current_e = pickNextUnvisitedNeighbor(visitedEdgeIds, *current_e, v_id);
        }
        lineCentersList.push_back(lineCenters);
        lineColorsList.push_back(lineColors);
        lineCenters.clear();
        lineColors.clear();
    }
}

void HexMesh::getCompleteVertexData(
        std::vector<glm::vec3> &pointVertices,
        std::vector<glm::vec4> &pointColors,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    std::unordered_set<uint32_t> vertexOnSingularEdgeIds;

    for (size_t i = 0; i < si->SEs.size(); i++) {
        Singular_E& se = si->SEs.at(i);
        for (size_t edgeIndex = 0; edgeIndex < se.es_link.size(); edgeIndex++) {
            Hybrid_E& e = mesh->Es[se.es_link.at(edgeIndex)];
            for (uint32_t v_id : e.vs) {
                if (vertexOnSingularEdgeIds.find(v_id) == vertexOnSingularEdgeIds.end()) {
                    vertexOnSingularEdgeIds.insert(v_id);
                }
            }
        }
    }

    for (Hybrid_V& v : mesh->Vs) {
        glm::vec3 vertexPosition(mesh->V(0, v.id), mesh->V(1, v.id), mesh->V(2, v.id));
        pointVertices.push_back(vertexPosition);
        if (vertexOnSingularEdgeIds.find(v.id) == vertexOnSingularEdgeIds.end()) {
            pointColors.push_back(regularColor);
        } else {
            pointColors.push_back(singularColor);
        }
    }
}

void HexMesh::getVertexTubeData(
        std::vector<glm::vec3> &pointVertices,
        std::vector<glm::vec4> &pointColors,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    std::unordered_set<uint32_t> excludedVertexIds;

    for (Singular_E& se : si->SEs) {
        for (uint32_t i = 1; i < se.vs_link.size() - 1; i++) {
            uint32_t v_id = se.vs_link.at(i);
            excludedVertexIds.insert(v_id);
        }
    }

    std::unordered_set<uint32_t> vertexOnSingularEdgeIds;

    for (Hybrid_V& v : mesh->Vs) {
        if (excludedVertexIds.find(v.id) != excludedVertexIds.end()) {
            continue;
        }

        glm::vec3 vertexPosition(mesh->V(0, v.id), mesh->V(1, v.id), mesh->V(2, v.id));
        pointVertices.push_back(vertexPosition);
        if (vertexOnSingularEdgeIds.find(v.id) == vertexOnSingularEdgeIds.end()) {
            pointColors.push_back(regularColor);
        } else {
            pointColors.push_back(singularColor);
        }
    }
}


void HexMesh::getSurfaceDataBarycentric(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec4>& vertexColors,
        std::vector<glm::vec3>& barycentricCoordinates,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    std::unordered_set<uint32_t> singularEdgeIDs;
    for (Singular_E& se : si->SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIDs.insert(e_id);
        }
    }

    size_t indexOffset = 0;
    for (Hybrid_F& f : mesh->Fs) {
        for (uint32_t v_id : f.vs) {
            glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            vertexPositions.push_back(vertexPosition);
        }

        /**
         * 1             2
         *  | - - - - - |
         *  | \         |
         *  |   \       |
         *  |     \     |
         *  |       \   |
         *  |         \ |
         *  | - - - - - |
         * 0             3
         */
        indices.push_back(indexOffset + 0);
        indices.push_back(indexOffset + 3);
        indices.push_back(indexOffset + 1);
        indices.push_back(indexOffset + 2);
        indices.push_back(indexOffset + 1);
        indices.push_back(indexOffset + 3);

        glm::vec4 vertexColor(regularColor);
        vertexColors.push_back(vertexColor);
        vertexColors.push_back(vertexColor);
        vertexColors.push_back(vertexColor);
        vertexColors.push_back(vertexColor);

        barycentricCoordinates.push_back(glm::vec3(1,0,0));
        barycentricCoordinates.push_back(glm::vec3(0,1,0));
        barycentricCoordinates.push_back(glm::vec3(1,0,0));
        barycentricCoordinates.push_back(glm::vec3(0,0,1));

        indexOffset += 4;
    }
}


void HexMesh::getSurfaceDataWireframeFaces(
        std::vector<uint32_t>& indices,
        std::vector<HexahedralCellFace>& hexahedralCellFaces,
        bool onlyBoundary,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    Mesh* mesh = baseComplexMesh;
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    std::unordered_set<uint32_t> singularEdgeIds;
    for (Singular_E& se : si->SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIds.insert(e_id);
        }
    }

    if (!onlyBoundary) {
        hexahedralCellFaces.resize(mesh->Fs.size());
    }

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Fs.size(); i++) {
        Hybrid_F& f = mesh->Fs.at(i);
        HexahedralCellFace* hexahedralCellFace;
        if (onlyBoundary) {
            if (f.boundary) {
                hexahedralCellFaces.push_back(HexahedralCellFace());
                hexahedralCellFace = &hexahedralCellFaces.back();
            } else {
                continue;
            }
        } else {
            hexahedralCellFace = &hexahedralCellFaces.at(i);
        }

        assert(f.vs.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t v_id = f.vs.at(j);
            glm::vec4 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
            hexahedralCellFace->vertexPositions[j] = vertexPosition;
        }

        /**
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
        indices.push_back(indexOffset + 0);
        indices.push_back(indexOffset + 3);
        indices.push_back(indexOffset + 1);
        indices.push_back(indexOffset + 2);
        indices.push_back(indexOffset + 1);
        indices.push_back(indexOffset + 3);

        assert(f.es.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t e_id = f.es.at(j);
            if (singularEdgeIds.find(e_id) != singularEdgeIds.end()) {
                hexahedralCellFace->lineColors[j] = singularColor;
            } else {
                hexahedralCellFace->lineColors[j] = regularColor;
            }
        }

        indexOffset += 4;
    }
}
