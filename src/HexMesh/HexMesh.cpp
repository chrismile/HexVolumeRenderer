/*
 * This file is the interface between the HexaMesh classes and HexVolumeRenderer.
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

#include "HexMesh.hpp"

#include "HexaLab/hex_quality.h"
#include "HexaLab/hex_quality_color_maps.h"
#include "HexaLab/mesh.h"
#include "HexaLab/mesh_navigator.h"

#include "BaseComplex/base_complex.h"

HexMesh::~HexMesh() {
    if (hexaLabApp != nullptr) {
        delete hexaLabApp;
        hexaLabApp = nullptr;
    }
    if (baseComplexMesh != nullptr) {
        delete baseComplexMesh;
        baseComplexMesh = nullptr;
    }
}

void HexMesh::setHexMeshData(const std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices) {
    if (hexaLabApp != nullptr) {
        delete hexaLabApp;
        hexaLabApp = nullptr;
    }
    if (baseComplexMesh != nullptr) {
        delete baseComplexMesh;
        baseComplexMesh = nullptr;
    }
    hexaLabApp = new HexaLab::App(transferFunctionWindow);
    std::vector<HexaLab::Index> indices;
    for (uint32_t& idx : cellIndices) {
        indices.push_back(idx);
    }
    hexaLabApp->import_mesh(vertices, indices);
    setQualityMeasure(qualityMeasure);

    baseComplexMesh = new Mesh;
    computeBaseComplexMesh(vertices, cellIndices);

    dirty = true;
}

void HexMesh::computeBaseComplexMesh(const std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices) {
    Mesh& mesh = *baseComplexMesh;
    const uint32_t numVertices = vertices.size();
    const uint32_t numCells = cellIndices.size() / 8;

    mesh.type = Mesh_type::Hex;
    mesh.V.resize(3, numVertices);
    mesh.V.setZero();
    mesh.Vs.resize(numVertices);

    for (uint32_t i = 0; i < numVertices; i++) {
        const glm::vec3& vertexPosition = vertices.at(i);
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
    for (uint32_t i = 0; i < numCells; i++) {
        h.vs.resize(8);
        h.id = i;
        for (int vertIdx = 0; vertIdx < 8; vertIdx++) {
            uint32_t vertexIndex = cellIndices.at(i*8 + vertIdx);
            h.vs[vertIdx] = vertexIndex;
            mesh.Vs[vertexIndex].neighbor_hs.push_back(h.id);
        }
        mesh.Hs[h.id] = h;
    }

    build_connectivity(mesh);
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

void HexMesh::getSurfaceData(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec3>& normals,
        std::vector<glm::vec4>& colors) {
    if (dirty) {
        hexaLabApp->update_models();
        dirty = false;
    }
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
    if (dirty) {
        hexaLabApp->update_models();
        dirty = false;
    }
    vertices = hexaLabApp->get_visible_model()->wireframe_vert_pos;
    colors = hexaLabApp->get_visible_model()->wireframe_vert_color;
}

void HexMesh::getVolumeData(
        std::vector<uint32_t>& indices,
        std::vector<glm::vec3>& vertices,
        std::vector<glm::vec3>& normals,
        std::vector<glm::vec4>& colors) {
    if (dirty) {
        hexaLabApp->update_models();
        dirty = false;
    }
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
    if (dirty) {
        hexaLabApp->update_models();
        dirty = false;
    }

    Mesh& mesh = *baseComplexMesh;

    base_complex bc;
    Singularity si;
    build_connectivity(mesh);
    bc.singularity_structure(si, mesh);

    for (size_t i = 0; i < si.SVs.size(); i++) {
        Singular_V& singularV = si.SVs.at(i);
        glm::vec3 vertexPosition(mesh.V(0, singularV.hid), mesh.V(1, singularV.hid), mesh.V(2, singularV.hid));
        pointVertices.push_back(vertexPosition);
        pointColors.push_back(glm::vec4(1,0,0,1));
    }

    for (size_t i = 0; i < si.SEs.size(); i++) {
        Singular_E& singularE = si.SEs.at(i);
        for (size_t lineVertexIndex = 0; lineVertexIndex < singularE.vs_link.size(); lineVertexIndex++) {
            uint32_t vertexIndex = singularE.vs_link.at(lineVertexIndex);
            glm::vec3 vertexPosition(mesh.V(0, vertexIndex), mesh.V(1, vertexIndex), mesh.V(2, vertexIndex));
            lineVertices.push_back(vertexPosition);
            lineColors.push_back(glm::vec4(1,0,0,1));
        }
    }
}

void HexMesh::getBaseComplexDataWireframe(
        std::vector<glm::vec3>& lineVertices,
        std::vector<glm::vec4>& lineColors,
        std::vector<glm::vec3>& pointVertices,
        std::vector<glm::vec4>& pointColors) {
    if (dirty) {
        hexaLabApp->update_models();
        dirty = false;
    }

    Mesh& mesh = *baseComplexMesh;

    base_complex bc;
    Singularity si;
    Frame frame;
    build_connectivity(mesh);
    bc.singularity_structure(si, mesh);
    bc.base_complex_extraction(si, frame, mesh);

    for (Frame_V& frameVertex : frame.FVs) {
        glm::vec4 vertexColor;
        if (frameVertex.what_type == 0) {
            vertexColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        } else {
            vertexColor = glm::vec4(0.0f, 0.2f, 1.0f, 1.0f);
        }

        glm::vec3 vertexPosition(mesh.V(0, frameVertex.hid), mesh.V(1, frameVertex.hid), mesh.V(2, frameVertex.hid));
        pointVertices.push_back(vertexPosition);
        pointColors.push_back(glm::vec4(1,0,0,1));
    }

    for (Frame_E& frameEdge : frame.FEs) {
        glm::vec4 vertexColor;
        if (frameEdge.singular) {
            vertexColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        } else {
            vertexColor = glm::vec4(0.0f, 0.2f, 1.0f, 1.0f);
        }

        for (size_t lineVertexIndex = 0; lineVertexIndex < frameEdge.vs_link.size(); lineVertexIndex++) {
            uint32_t vertexIndex = frameEdge.vs_link.at(lineVertexIndex);
            glm::vec3 vertexPosition(mesh.V(0, vertexIndex), mesh.V(1, vertexIndex), mesh.V(2, vertexIndex));
            lineVertices.push_back(vertexPosition);
            lineColors.push_back(vertexColor);
        }
    }
}

void HexMesh::getBaseComplexDataSurface(
        std::vector<glm::vec3>& triangleVertices,
        std::vector<glm::vec4>& vertexColors) {
    ;
}


void HexMesh::getLodRepresentation(
        std::vector<glm::vec3>& lineVertices,
        std::vector<uint32_t>& lineLodValues) {
    if (dirty) {
        hexaLabApp->update_models();
        dirty = false;
    }
    hexaLabApp->build_lod_representation(lineVertices, lineLodValues);
}
