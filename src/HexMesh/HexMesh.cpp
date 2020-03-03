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

HexMesh::~HexMesh() {
    if (hexaLabApp != nullptr) {
        delete hexaLabApp;
        hexaLabApp = nullptr;
    }
}

void HexMesh::setHexMeshData(const std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices) {
    if (hexaLabApp != nullptr) {
        delete hexaLabApp;
        hexaLabApp = nullptr;
    }
    hexaLabApp = new HexaLab::App(transferFunctionWindow);
    std::vector<HexaLab::Index> indices;
    for (uint32_t& idx : cellIndices) {
        indices.push_back(idx);
    }
    hexaLabApp->import_mesh(vertices, indices);
    setQualityMeasure(qualityMeasure);
    dirty = true;
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
    hexaLabApp->build_singularity_model(lineVertices, lineColors, pointVertices, pointColors);
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
