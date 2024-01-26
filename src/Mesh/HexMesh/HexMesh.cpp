/*
 * This file is the interface between HexVolumeRenderer, HexaLab classes and
 * the code from Robust Hexahedral Re-Meshing (see README).
 * Some functions from HexaLab are used here, which are covered by the MIT license.
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
#include <unordered_map>

#include <glm/detail/setup.hpp>
#if GLM_VERSION_MAJOR == 1 && GLM_VERSION_MINOR == 0 && GLM_VERSION_PATCH == 0
#include <limits>
template<class T> T epsilon() { return std::numeric_limits<T>::epsilon(); }
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/color_space.hpp>

#include <Utils/File/Logfile.hpp>
#include <Utils/Random/Xorshift.hpp>

#include "QualityMeasure/hex_quality.h"
#include "QualityMeasure/hex_quality_color_maps.h"

#include "Renderers/Helpers/HexahedronVolume.hpp"
#include "Renderers/LOD/LodSheetGeneration.hpp"
#include "../BaseComplex/base_complex.h"

#include "EdgeKey.hpp"
#include "HexMesh.hpp"

const glm::vec4 HexMesh::glowColorRegular = glm::vec4(0.0f, 0.5f, 0.2f, 1.0f);
const glm::vec4 HexMesh::glowColorSingular = glm::vec4(0.8f, 0.1f, 0.1f, 1.0f);
const glm::vec4 HexMesh::outlineColorRegular = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
const glm::vec4 HexMesh::outlineColorSingular = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

// See: https://stackoverflow.com/questions/2513505/how-to-get-available-memory-c-g
#ifdef linux
#include <unistd.h>
size_t getUsedSystemMemoryBytes()
{
    size_t totalNumPages = sysconf(_SC_PHYS_PAGES);
    size_t availablePages = sysconf(_SC_AVPHYS_PAGES);
    size_t pageSizeBytes = sysconf(_SC_PAGE_SIZE);
    return (totalNumPages - availablePages) * pageSizeBytes;
}
#endif
#ifdef windows
#include <windows.h>
size_t getUsedSystemMemoryBytes()
{
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    GlobalMemoryStatusEx(&status);
    return status.ullTotalPhys - status.ullAvailPhys;
}
#endif

HexMesh::HexMesh(sgl::TransferFunctionWindow &transferFunctionWindow, RayMeshIntersection& rayMeshIntersection)
        : transferFunctionWindow(transferFunctionWindow), rayMeshIntersection(rayMeshIntersection) {
}

HexMesh::~HexMesh() {
    if (mesh != nullptr) {
        delete mesh;
        mesh = nullptr;
        delete si;
        si = nullptr;
        delete frame;
        frame = nullptr;
    }
}

void HexMesh::setHexMeshData(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices, bool loadMeshRepresentation) {
    if (mesh != nullptr) {
        delete mesh;
        mesh = nullptr;
    }
    if (si != nullptr) {
        delete si;
        si = nullptr;
    }
    if (frame != nullptr) {
        delete frame;
        frame = nullptr;
    }

    this->vertices = vertices;
    this->cellIndices = cellIndices;
    meshNumCells = cellIndices.size() / 8ull;
    meshNumVertices = vertices.size();
    cellFilteringList.resize(meshNumCells);
    cellQualityMeasureList.resize(meshNumCells);

    if (loadMeshRepresentation) {
        computeBaseComplexMesh(vertices, cellIndices);
    }

    cellVolumes.clear();
    faceAreas.clear();

    if (mesh) {
        sgl::Logfile::get()->writeInfo(std::string() + "Number of mesh vertices: " + std::to_string(mesh->Vs.size()));
        sgl::Logfile::get()->writeInfo(std::string() + "Number of mesh edges: " + std::to_string(mesh->Es.size()));
        sgl::Logfile::get()->writeInfo(std::string() + "Number of mesh faces: " + std::to_string(mesh->Fs.size()));
        sgl::Logfile::get()->writeInfo(std::string() + "Number of mesh cells: " + std::to_string(mesh->Hs.size()));
    } else {
        sgl::Logfile::get()->writeInfo(std::string() + "Number of mesh vertices: " + std::to_string(vertices.size()));
        sgl::Logfile::get()->writeInfo(std::string() + "Number of mesh cells: " + std::to_string(cellIndices.size()/8ull));
    }

    dirty = true;
}

void HexMesh::addManualVertexAttribute(const std::vector<float>& vertexAttributes, const std::string& attributeName) {
    float minAttribute = FLT_MAX;
    float maxAttribute = -FLT_MAX;
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) reduction(min: minAttribute) reduction(max: maxAttribute) \
    shared(vertexAttributes)
#endif
    for (size_t i = 0; i < vertexAttributes.size(); i++) {
        minAttribute = std::min(minAttribute, vertexAttributes.at(i));
        maxAttribute = std::max(maxAttribute, vertexAttributes.at(i));
    }

    manualVertexAttributesMinMax.emplace_back(glm::vec2(minAttribute, maxAttribute));
    manualVertexAttributesList.push_back(vertexAttributes);
    manualVertexAttributesNames.push_back(attributeName);

    useManualVertexAttribute = true;
    manualVertexAttributeIdx = 0;
    manualVertexAttributes = &manualVertexAttributesList.at(manualVertexAttributeIdx);

    recomputeHistogram();
}

void HexMesh::addManualCellAttribute(const std::vector<float>& cellAttributes, const std::string& attributeName) {
    float minAttribute = FLT_MAX;
    float maxAttribute = -FLT_MAX;
#if _OPENMP >= 201107
#pragma omp parallel for default(none) reduction(min: minAttribute) reduction(max: maxAttribute) \
    shared(cellAttributes)
#endif
    for (size_t i = 0; i < cellAttributes.size(); i++) {
        minAttribute = std::min(minAttribute, cellAttributes.at(i));
        maxAttribute = std::max(maxAttribute, cellAttributes.at(i));
    }

    manualCellAttributesMinMax.emplace_back(glm::vec2(minAttribute, maxAttribute));
    manualCellAttributesList.push_back(cellAttributes);
    manualCellAttributesNames.push_back(attributeName);

    useManualCellAttribute = true;
    manualCellAttributeIdx = 0;
    manualCellAttributes = &manualCellAttributesList.at(manualCellAttributeIdx);

    cellQualityMeasureList = cellAttributes;

    recomputeHistogram();
}

/**
 * Vertex and edge IDs:
 *
 *      3 +----------------+ 2
 *       /|       1       /|
 *   11 / |           10 / |
 *     /  |             /  |
 *    /   | 4          /   | 7
 * 7 +----------------+ 6  |
 *   |    |   2       |    |
 *   |    |           |    |
 *   |    |      0    |    |
 *   |  0 +-----------|----+ 1
 * 5 |   /          6 |   /
 *   |  / 8           |  / 9
 *   | /              | /
 *   |/      3        |/
 * 4 +----------------+ 5
 */
void buildCellEdgeList(Mesh &mesh) {
    EdgeMap edgeMap;
    for (Hybrid_E& e : mesh.Es) {
        edgeMap.insert(std::make_pair(EdgeKey(e.vs.at(0), e.vs.at(1)), e.id));
    }

    for (Hybrid& h : mesh.Hs) {
        assert(h.es.empty());
        h.es.resize(12);
        h.es.at(0) = edgeMap.find(EdgeKey(h.vs.at(0), h.vs.at(1)))->second;
        h.es.at(1) = edgeMap.find(EdgeKey(h.vs.at(2), h.vs.at(3)))->second;
        h.es.at(2) = edgeMap.find(EdgeKey(h.vs.at(6), h.vs.at(7)))->second;
        h.es.at(3) = edgeMap.find(EdgeKey(h.vs.at(4), h.vs.at(5)))->second;

        h.es.at(4) = edgeMap.find(EdgeKey(h.vs.at(0), h.vs.at(3)))->second;
        h.es.at(5) = edgeMap.find(EdgeKey(h.vs.at(4), h.vs.at(7)))->second;
        h.es.at(6) = edgeMap.find(EdgeKey(h.vs.at(5), h.vs.at(6)))->second;
        h.es.at(7) = edgeMap.find(EdgeKey(h.vs.at(1), h.vs.at(2)))->second;

        h.es.at(8) = edgeMap.find(EdgeKey(h.vs.at(0), h.vs.at(4)))->second;
        h.es.at(9) = edgeMap.find(EdgeKey(h.vs.at(1), h.vs.at(5)))->second;
        h.es.at(10) = edgeMap.find(EdgeKey(h.vs.at(2), h.vs.at(6)))->second;
        h.es.at(11) = edgeMap.find(EdgeKey(h.vs.at(3), h.vs.at(7)))->second;
    }
}

void HexMesh::computeBaseComplexMesh(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices) {
    const uint32_t numVertices = vertices.size();
    const uint32_t numCells = cellIndices.size() / 8;

    mesh = new Mesh;
    si = new Singularity;

    mesh->type = Mesh_type::Hex;
    mesh->V.resize(3, numVertices);
    mesh->V.setZero();
    mesh->Vs.resize(numVertices);

    for (uint32_t i = 0; i < numVertices; i++) {
        const glm::vec3 &vertexPosition = vertices.at(i);
        mesh->V(0, i) = vertexPosition.x;
        mesh->V(1, i) = vertexPosition.y;
        mesh->V(2, i) = vertexPosition.z;

        Hybrid_V v;
        v.id = i;
        v.boundary = false;
        mesh->Vs[i] = v;
    }

    mesh->Hs.resize(numCells);
    Hybrid h;
    h.vs.resize(8);
    for (uint32_t i = 0; i < numCells; i++) {
        h.id = i;
        for (int vertIdx = 0; vertIdx < 8; vertIdx++) {
            uint32_t vertexIndex = cellIndices.at(i * 8 + vertIdx);
            h.vs[vertIdx] = vertexIndex;
            mesh->Vs[vertexIndex].neighbor_hs.push_back(h.id);
        }
        mesh->Hs[h.id] = h;
    }

    build_connectivity(*mesh);
    buildCellEdgeList(*mesh);
    base_complex bc;
    bc.singularity_structure(*si, *mesh);

    singularEdgeIds.clear();
    for (Singular_E& se : si->SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIds.insert(e_id);
        }
    }
}

void HexMesh::computeBaseComplexMeshFrame() {
    assert(frame == nullptr);
    frame = new Frame;
    base_complex bc;
    bc.base_complex_extraction(*si, *frame, *mesh);

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



void HexMesh::updateVertexPositions(const std::vector<glm::vec3>& vertices) {
    if (mesh) {
        // Update the base-complex mesh.
        for (size_t i = 0; i < vertices.size(); i++) {
            Hybrid_V& v = mesh->Vs.at(i);
            const glm::vec3& vertPos = vertices.at(i);
            for (int j = 0; j < 3; j++) {
                mesh->V(j, v.id) = vertPos[j];
            }
        }
    }

    for (size_t i = 0; i < vertices.size(); i++) {
        this->vertices.at(i) = vertices.at(i);
    }

    setQualityMeasure(qualityMeasure);

    dirty = true;
}

void HexMesh::onTransferFunctionMapRebuilt() {
    //dirty = true;
}

void HexMesh::markCell(uint32_t h_id) {
    cellFilteringList.at(h_id) = true;
}

bool HexMesh::isCellMarked(uint32_t h_id) {
    return cellFilteringList.at(h_id);
}

void HexMesh::unmark() {
    for (size_t i = 0; i < cellFilteringList.size(); i++) {
        cellFilteringList.at(i) = false;
    }
    dirty = true;
}

void HexMesh::setQualityMeasure(QualityMeasure qualityMeasure) {
    this->qualityMeasure = qualityMeasure;
    HexaLab::QualityMeasureEnum hexaLabQualityMeasure;
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

    HexaLab::quality_measure_fun* qualityFunctor = get_quality_measure_fun (hexaLabQualityMeasure);
    void* arg = nullptr;
    float avgVolume = 0.0f;
    if (hexaLabQualityMeasure == HexaLab::QualityMeasureEnum::RSS
            || hexaLabQualityMeasure == HexaLab::QualityMeasureEnum::SHAS
            || hexaLabQualityMeasure == HexaLab::QualityMeasureEnum::SHES) {
        avgVolume = getAverageCellVolume();
        arg = &avgVolume;
    }

    std::vector<float> cellQualityList;
    cellQualityList.resize(meshNumCells);
    qualityMin = FLT_MAX;
    qualityMax = -FLT_MAX;
    qualityMinNormalized = FLT_MAX;
    qualityMaxNormalized = -FLT_MAX;
    glm::vec3 v[8];

    if (mesh) {
#if _OPENMP >= 201107
        #pragma omp parallel for reduction(min: qualityMin) reduction(max: qualityMax) private(v) \
                shared(cellQualityList, qualityFunctor, arg) default(none)
#endif
        for (size_t i = 0; i < meshNumCells; i++) {
            Hybrid& h = mesh->Hs.at(i);
            assert(h.vs.size() == 8);
            for (size_t j = 0; j < h.vs.size(); j++) {
                uint32_t v_id = h.vs.at(j);
                v[j] = glm::vec3(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            }
            float quality = qualityFunctor(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], arg);
            qualityMin = std::min(qualityMin, quality);
            qualityMax = std::max(qualityMax, quality);
            cellQualityList.at(i) = quality;
        }
#if _OPENMP >= 201107
        #pragma omp parallel for reduction(min: qualityMinNormalized) reduction(max: qualityMaxNormalized) \
                shared(cellQualityList, hexaLabQualityMeasure) default(none)
#endif
        for (size_t i = 0; i < meshNumCells; i++) {
            cellQualityMeasureList.at(i) = 1.0f - HexaLab::normalize_quality_measure(
                    hexaLabQualityMeasure, cellQualityList.at(i), qualityMin, qualityMax);
            qualityMinNormalized = std::min(qualityMinNormalized, cellQualityMeasureList.at(i));
            qualityMaxNormalized = std::max(qualityMaxNormalized, cellQualityMeasureList.at(i));
        }
    } else {
#if _OPENMP >= 201107
        #pragma omp parallel for reduction(min: qualityMin) reduction(max: qualityMax) private(v) \
                shared(cellQualityList, qualityFunctor, arg) default(none)
#endif
        for (size_t i = 0; i < meshNumCells; i++) {
            for (size_t j = 0; j < 8; j++) {
                uint32_t v_id = cellIndices.at(i * 8 + j);
                v[j] = glm::vec3(vertices.at(v_id));
            }
            float quality = qualityFunctor(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], arg);
            qualityMin = std::min(qualityMin, quality);
            qualityMax = std::max(qualityMax, quality);
            cellQualityList.at(i) = quality;
        }
#if _OPENMP >= 201107
        #pragma omp parallel for reduction(min: qualityMinNormalized) reduction(max: qualityMaxNormalized) \
                shared(cellQualityList, hexaLabQualityMeasure) default(none)
#endif
        for (size_t i = 0; i < meshNumCells; i++) {
            cellQualityMeasureList.at(i) = 1.0f - HexaLab::normalize_quality_measure(
                    hexaLabQualityMeasure, cellQualityList.at(i), qualityMin, qualityMax);
            qualityMinNormalized = std::min(qualityMinNormalized, cellQualityMeasureList.at(i));
            qualityMaxNormalized = std::max(qualityMaxNormalized, cellQualityMeasureList.at(i));
        }
    }
    std::cout << "Quality: " << HexaLab::get_quality_name(hexaLabQualityMeasure)
            << ", range: [" << qualityMin << ", " << qualityMax << "], normalized range: "
            << qualityMinNormalized << ", " << qualityMaxNormalized << "]" << std::endl;

    recomputeHistogram();
    dirty = true;
}

void HexMesh::recomputeHistogram() {
    if (useManualVertexAttribute) {
        transferFunctionWindow.computeHistogram(*manualVertexAttributes, 0.0f, 1.0f);
    } else if (useManualCellAttribute) {
        transferFunctionWindow.computeHistogram(*manualCellAttributes, 0.0f, 1.0f);
    } else {
        //transferFunctionWindow.computeHistogram(cellQualityMeasureList, qualityMinNormalized, qualityMaxNormalized);
        transferFunctionWindow.computeHistogram(cellQualityMeasureList, 0.0f, 1.0f);
    }
}

float HexMesh::getCellAttribute(uint32_t h_id) {
    return cellQualityMeasureList.at(h_id);
}

float HexMesh::getCellAttributeManualVertexAttributes(uint32_t h_id) {
    float cellAttribute = 0.0f;
    Hybrid& h = mesh->Hs.at(h_id);
    for (uint32_t v_id : h.vs) {
        cellAttribute += manualVertexAttributes->at(v_id);
    }
    cellAttribute /= h.vs.size();
    return cellAttribute;
}

bool HexMesh::isBaseComplexMeshLoaded() {
    return mesh != nullptr;
}

Mesh& HexMesh::getBaseComplexMesh() {
    return *mesh;
}

Singularity& HexMesh::getBaseComplexMeshSingularity(){
    return *si;
}

Frame& HexMesh::getBaseComplexMeshFrame(){
    if (!frame) computeBaseComplexMeshFrame();
    return *frame;
}

void HexMesh::rebuildInternalRepresentationIfNecessary() {
    if (!mesh) {
        computeBaseComplexMesh(vertices, cellIndices);
    }

    if (dirty) {
        updateMeshTriangleIntersectionDataStructure();
        dirty = false;
    }
}

void HexMesh::updateMeshTriangleIntersectionDataStructure() {
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertices;

    // Add all hexahedral mesh vertices to the triangle mesh vertex data.
    vertices.reserve(mesh->Vs.size());
    for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
        glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
        vertices.push_back(vertexPosition);
    }

    // Add all triangle indices.
    triangleIndices.reserve(mesh->Fs.size() * 12);
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        assert(f.vs.size() == 4);
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

size_t HexMesh::getNumberOfSingularEdges() {
    return singularEdgeIds.size();
}

size_t HexMesh::getNumberOfSingularEdges(bool boundary, uint32_t valence) {
    size_t counter = 0;
    for (uint32_t e_id : singularEdgeIds) {
        Hybrid_E& e = mesh->Es.at(e_id);
        if (e.boundary == boundary && e.neighbor_hs.size() == valence) {
            counter++;
        }
    }
    return counter;
}

std::unordered_set<uint32_t>& HexMesh::getSingularEdgeIds() {
    return singularEdgeIds;
}


float HexMesh::getFaceArea(uint32_t f_id) {
    glm::vec3 facePointsArray[4];
    Hybrid_F& f = mesh->Fs.at(f_id);
    assert(f.vs.size() == 4u);
    for (size_t i = 0; i < f.vs.size(); i++) {
        uint32_t v_id = f.vs.at(i);
        facePointsArray[i] = glm::vec3(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
    }
    float faceArea = computeQuadrilateralFaceArea_Barycenter(facePointsArray);
    return faceArea;
}

float HexMesh::getFaceIdsAreaSum(const std::vector<uint32_t>& f_ids) {
    if (faceAreas.empty()) {
        computeAllFaceAreas();
    }

    float areaSum = 0.0f;
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) reduction(+: areaSum) shared(f_ids, faceAreas)
#endif
    for (size_t i = 0; i < f_ids.size(); i++) {
        areaSum += faceAreas.at(f_ids.at(i));
    }
    //for (uint32_t f_id : f_ids) {
    //    areaSum += faceAreas.at(f_id);
    //}
    return areaSum;
}

void HexMesh::computeAllFaceAreas() {
    faceAreas.resize(mesh->Fs.size());
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) shared(faceAreas, mesh)
#endif
    for (uint32_t f_id = 0; f_id < mesh->Fs.size(); f_id++) {
        faceAreas.at(f_id) = getFaceArea(f_id);
    }
}

float HexMesh::getCellVolume(uint32_t h_id) {
    glm::vec3 cellPointsArray[8];
    Hybrid& h = mesh->Hs.at(h_id);
    assert(h.vs.size() == 8u);
    for (size_t i = 0; i < h.vs.size(); i++) {
        uint32_t v_id = h.vs.at(i);
        cellPointsArray[i] = glm::vec3(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
    }
    float cellVolume = computeHexahedralCellVolume_TetrakisHexahedron(cellPointsArray);
    return cellVolume;
}

float HexMesh::getCellIdsVolumeSum(const std::vector<uint32_t>& h_ids) {
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    float volumeSum = 0.0f;
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) reduction(+: volumeSum) shared(h_ids, cellVolumes)
#endif
    for (size_t i = 0; i < h_ids.size(); i++) {
        volumeSum += cellVolumes.at(h_ids.at(i));
    }
    //for (uint32_t h_id : h_ids) {
    //    volumeSum += cellVolumes.at(h_id);
    //}
    return volumeSum;
}

void HexMesh::computeAllCellVolumes() {
    cellVolumes.resize(mesh->Hs.size());
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) shared(cellVolumes, mesh)
#endif
    for (uint32_t h_id = 0; h_id < mesh->Hs.size(); h_id++) {
        cellVolumes.at(h_id) = getCellVolume(h_id);
    }
}

float HexMesh::getTotalCellVolume() {
    rebuildInternalRepresentationIfNecessary();
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    float totalVolume = 0.0f;
    for (uint32_t i = 0; i < mesh->Hs.size(); i++) {
        totalVolume += cellVolumes.at(i);
    }
    return totalVolume;
}

float HexMesh::getAverageCellVolume() {
    rebuildInternalRepresentationIfNecessary();
    return getTotalCellVolume() / float(mesh->Hs.size());
}

float HexMesh::interpolateCellAttributePerVertex(uint32_t v_id, const std::vector<float>& cellVolumes) {
    Hybrid_V& v = mesh->Vs.at(v_id);

    float volumeSum = 0.0f;
    for (uint32_t h_id : v.neighbor_hs) {
        volumeSum += cellVolumes.at(h_id);
    }

    float vertexAttribute;
    if (qualityMeasure == QUALITY_MEASURE_JACOBIAN || qualityMeasure == QUALITY_MEASURE_SCALED_JACOBIAN) {
        // sum(V_i) / sum(V_i/J_i)
        float volumeWeightedInverseAttributeSum = 0.0f;
        for (uint32_t h_id : v.neighbor_hs) {
            float cellVolume = cellVolumes.at(h_id);
            float cellAttribute = getCellAttribute(h_id);
            volumeWeightedInverseAttributeSum += cellVolume / cellAttribute;
        }
        vertexAttribute = volumeSum / volumeWeightedInverseAttributeSum;
        // sum(V_i/J_i) / sum(V_i)
        /*float volumeWeightedInverseAttributeSum = 0.0f;
        for (uint32_t h_id : v.neighbor_hs) {
            float cellVolume = cellVolumes.at(h_id);
            float cellAttribute = getCellAttribute(h_id)(h_id);
            volumeWeightedInverseAttributeSum += cellVolume / cellAttribute;
        }
        vertexAttribute = volumeWeightedInverseAttributeSum / volumeSum;*/
        /*vertexAttribute = (1.0f / float(v.neighbor_hs.size())) * volumeSum / volumeWeightedInverseAttributeSum;*/
    } else {
        // sum(V_i * J_i) / sum(V_i)
        float volumeWeightedAttributeSum = 0.0f;
        for (uint32_t h_id : v.neighbor_hs) {
            float cellVolume = cellVolumes.at(h_id);
            float cellAttribute = getCellAttribute(h_id);
            volumeWeightedAttributeSum += cellVolume * cellAttribute;
        }
        vertexAttribute = volumeWeightedAttributeSum / volumeSum;
    }
    return vertexAttribute;
}

float HexMesh::maximumCellAttributePerVertex(uint32_t v_id) {
    Hybrid_V& v = mesh->Vs.at(v_id);

    float maximumCellAttributeValue = 0.0f;
    for (uint32_t h_id : v.neighbor_hs) {
        maximumCellAttributeValue = std::max(maximumCellAttributeValue, getCellAttribute(h_id));
    }
    return maximumCellAttributeValue;
}

float HexMesh::interpolateCellAttributePerEdge(uint32_t e_id, const std::vector<float>& cellVolumes) {
    Hybrid_E& e = mesh->Es.at(e_id);

    float volumeSum = 0.0f;
    for (uint32_t h_id : e.neighbor_hs) {
        volumeSum += cellVolumes.at(h_id);
    }

    float edgeAttribute;
    if (qualityMeasure == QUALITY_MEASURE_JACOBIAN || qualityMeasure == QUALITY_MEASURE_SCALED_JACOBIAN) {
        // sum(V_i) / sum(V_i/J_i)
        float volumeWeightedInverseAttributeSum = 0.0f;
        for (uint32_t h_id : e.neighbor_hs) {
            float cellVolume = cellVolumes.at(h_id);
            float cellAttribute = getCellAttribute(h_id);
            volumeWeightedInverseAttributeSum += cellVolume / cellAttribute;
        }
        edgeAttribute = volumeSum / volumeWeightedInverseAttributeSum;
        // sum(V_i/J_i) / sum(V_i)
        /*float volumeWeightedInverseAttributeSum = 0.0f;
        for (uint32_t h_id : e.neighbor_hs) {
            float cellVolume = cellVolumes.at(h_id);
            float cellAttribute = getCellAttribute(h_id)(h_id);
            volumeWeightedInverseAttributeSum += cellVolume / cellAttribute;
        }
        edgeAttribute = volumeWeightedInverseAttributeSum / volumeSum;*/
        /*edgeAttribute = (1.0f / float(v.neighbor_hs.size())) * volumeSum / volumeWeightedInverseAttributeSum;*/
    } else {
        // sum(V_i * J_i) / sum(V_i)
        float volumeWeightedAttributeSum = 0.0f;
        for (uint32_t h_id : e.neighbor_hs) {
            float cellVolume = cellVolumes.at(h_id);
            float cellAttribute = getCellAttribute(h_id);
            volumeWeightedAttributeSum += cellVolume * cellAttribute;
        }
        edgeAttribute = volumeWeightedAttributeSum / volumeSum;
    }
    return edgeAttribute;
}

float HexMesh::maximumCellAttributePerEdge(uint32_t e_id) {
    Hybrid_E& e = mesh->Es.at(e_id);

    float maximumCellAttributeValue = 0.0f;
    for (uint32_t h_id : e.neighbor_hs) {
        maximumCellAttributeValue = std::max(
                maximumCellAttributeValue, getCellAttribute(h_id));
    }
    return maximumCellAttributeValue;
}


std::vector<glm::vec3> HexMesh::getFilteredVertices(bool removeFilteredCells) {
    if (!removeFilteredCells) {
        return vertices;
    }

    rebuildInternalRepresentationIfNecessary();

    std::unordered_set<uint32_t> usedVertexIndices;
    for (size_t i = 0; i < mesh->Hs.size(); i++) {
        Hybrid& h = mesh->Hs.at(i);
        for (size_t j = 0; j < h.fs.size(); j++) {
            Hybrid_F& f = mesh->Fs.at(h.fs.at(j));
            if (!f.boundary && !std::any_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }
            if (removeFilteredCells && std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }

            assert(f.neighbor_hs.size() >= 1 && f.neighbor_hs.size() <= 2);
            bool invertWinding = f.neighbor_hs.at(0) != h.id;

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                usedVertexIndices.insert(v_id);
            }
        }
    }

    std::vector<glm::vec3> vertexPositions;
    for (uint32_t v_id : usedVertexIndices) {
        glm::vec3 vertexPosition(
                mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
        vertexPositions.push_back(vertexPosition);
    }

    return vertexPositions;
}

sgl::AABB3 HexMesh::getModelBoundingBox(bool removeFilteredCells) {
    std::vector<glm::vec3>* vertices;
    std::vector<glm::vec3> verticesLocal;
    if (removeFilteredCells) {
        verticesLocal = getFilteredVertices(removeFilteredCells);
        vertices = &verticesLocal;
    } else {
        vertices = &this->vertices;
    }

    float minPosX = std::numeric_limits<float>::max();
    float minPosY = std::numeric_limits<float>::max();
    float minPosZ = std::numeric_limits<float>::max();
    float maxPosX = std::numeric_limits<float>::lowest();
    float maxPosY = std::numeric_limits<float>::lowest();
    float maxPosZ = std::numeric_limits<float>::lowest();

#if _OPENMP >= 201107
    #pragma omp parallel for default(none) reduction(min: minPosX) reduction(min: minPosY) reduction(min: minPosZ) \
    reduction(max: maxPosX) reduction(max: maxPosY) reduction(max: maxPosZ) shared(vertices)
#endif
    for (size_t i = 0; i < vertices->size(); i++) {
        const glm::vec3& pos = vertices->at(i);
        minPosX = std::min(minPosX, pos.x);
        minPosY = std::min(minPosY, pos.y);
        minPosZ = std::min(minPosZ, pos.z);
        maxPosX = std::max(maxPosX, pos.x);
        maxPosY = std::max(maxPosY, pos.y);
        maxPosZ = std::max(maxPosZ, pos.z);
    }

    sgl::AABB3 modelBoundingBox(glm::vec3(minPosX, minPosY, minPosZ), glm::vec3(maxPosX, maxPosY, maxPosZ));

    return modelBoundingBox;
}


glm::vec4 HexMesh::edgeColorMap(bool isSingular, bool isBoundary, int valence) {
    if (!isSingular) {
        return outlineColorRegular;
    }

    if (isBoundary) {
        if (valence == 1) {
            return glm::vec4(1.0f, 0.0f, 0.0f, 1.0f); // red
        } else if (valence == 3) {
            return glm::vec4(1.0f, 0.20f, 0.0f, 1.0f); // orange-red
        }
    } else {
        if (valence == 3) {
            return glm::vec4(1.0f, 0.20f, 0.0f, 1.0f); // orange-red
        } else if (valence == 5) {
            return glm::vec4(1.0f, 0.65f, 0.0f, 1.0f); // orange-yellow
        }
    }

    return glm::vec4(0.45f, 0.0f, 0.5f, 1.0f); // purple
}

void HexMesh::getSurfaceData(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<float>& vertexAttributes,
        bool removeFilteredCells) {
    rebuildInternalRepresentationIfNecessary();

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Hs.size(); i++) {
        Hybrid& h = mesh->Hs.at(i);
        float cellAttribute = getCellAttribute(h.id);
        if (removeFilteredCells && isCellMarked(h.id)) {
            continue;
        }

        for (size_t j = 0; j < h.fs.size(); j++) {
            Hybrid_F& f = mesh->Fs.at(h.fs.at(j));
            if ((!removeFilteredCells && !f.boundary) || (removeFilteredCells
                    && !f.boundary && !std::any_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            }))) {
                continue;
            }

            assert(f.neighbor_hs.size() >= 1 && f.neighbor_hs.size() <= 2);
            bool invertWinding = f.neighbor_hs.at(0) != h.id;

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                glm::vec3 vertexPosition(
                        mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                vertexPositions.push_back(vertexPosition);
                if (!useManualVertexAttribute) {
                    vertexAttributes.push_back(cellAttribute);
                } else {
                    // Use manually specified attributes.
                    vertexAttributes.push_back(manualVertexAttributes->at(v_id));
                }
            }

            size_t oldTriangleIndicesSize = triangleIndices.size();
            if (!invertWinding) {
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
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
            }
            if (invertWinding) {
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
            }

            for (size_t j = 0; j < 2; j++) {
                glm::vec3 v[3];
                for (int k = 0; k < 3; k++) {
                    v[k] = vertexPositions.at(triangleIndices.at(oldTriangleIndicesSize + j*3 + k));
                }
                glm::vec3 vertexNormal = glm::normalize(glm::cross(v[1] - v[0], v[2] - v[0]));
                vertexNormals.push_back(vertexNormal);
                vertexNormals.push_back(vertexNormal);
            }

            indexOffset += 4;
        }
    }
}

void HexMesh::getSurfaceData(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        bool removeFilteredCells) {
    rebuildInternalRepresentationIfNecessary();

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Hs.size(); i++) {
        Hybrid& h = mesh->Hs.at(i);
        for (size_t j = 0; j < h.fs.size(); j++) {
            Hybrid_F& f = mesh->Fs.at(h.fs.at(j));
            if (!f.boundary && !std::any_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }
            if (removeFilteredCells && std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }

            assert(f.neighbor_hs.size() >= 1 && f.neighbor_hs.size() <= 2);
            bool invertWinding = f.neighbor_hs.at(0) != h.id;

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                glm::vec3 vertexPosition(
                        mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                vertexPositions.push_back(vertexPosition);
            }

            if (!invertWinding) {
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
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
            }
            if (invertWinding) {
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
            }

            indexOffset += 4;
        }
    }
}

void HexMesh::getWireframeData(
        std::vector<glm::vec3>& lineVertices,
        std::vector<glm::vec4>& lineColors) {
    rebuildInternalRepresentationIfNecessary();

    // Add all hexahedral mesh vertices to the triangle mesh vertex data.
    lineVertices.reserve(mesh->Vs.size());
    for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
        glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
        lineVertices.push_back(vertexPosition);
    }

    // Add all triangle indices.
    for (Hybrid_E& e : mesh->Es) {
        if (std::all_of(e.neighbor_hs.begin(), e.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        glm::vec4 lineColor = singularEdgeIds.find(e.id) == singularEdgeIds.end()
                ? glm::vec4(0.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        assert(e.vs.size() == 2);
        for (uint32_t v_id : e.vs) {
            lineVertices.push_back(glm::vec3(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id)));
            lineColors.push_back(lineColor);
        }
    }
}

void HexMesh::getVolumeData_Faces(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<float>& vertexAttributes) {
    rebuildInternalRepresentationIfNecessary();

    std::vector<glm::vec3> quadBuffer;
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        bool neighborIsMarked = std::any_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        });

        assert(f.vs.size() == 4);

        for (uint32_t v_id : f.vs) {
            glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            quadBuffer.push_back(vertexPosition);
        }
        glm::vec3 v0 = quadBuffer.at(1) - quadBuffer.at(0);
        glm::vec3 v1 = quadBuffer.at(2) - quadBuffer.at(0);
        glm::vec3 vertexNormal = glm::normalize(glm::cross(v0, v1));
        quadBuffer.clear();

        if (!isCellMarked(f.neighbor_hs.at(0))) {
            size_t idxStart = vertexPositions.size();
            float vertexAttribute;
            // Compute attribute data.
            if (!useManualVertexAttribute) {
                vertexAttribute = getCellAttribute(f.neighbor_hs.at(0));
            } else {
                vertexAttribute = getCellAttributeManualVertexAttributes(f.neighbor_hs.at(0));
            }
            for (uint32_t v_id : f.vs) {
                vertexAttributes.push_back(vertexAttribute);
                glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                vertexPositions.push_back(vertexPosition);
                vertexNormals.push_back(vertexNormal);
            }

            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+1);
            triangleIndices.push_back(idxStart+0);
            triangleIndices.push_back(idxStart+3);
            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+0);
        }

        if (!isCellMarked(f.neighbor_hs.at(0)) || (!f.boundary && isCellMarked(f.neighbor_hs.at(1)))) {
            size_t idxStart = vertexPositions.size();
            float vertexAttribute;
            // Compute attribute data.
            if (!useManualVertexAttribute) {
                vertexAttribute = getCellAttribute(f.neighbor_hs.at(f.boundary ? 0 : 1));
            } else {
                vertexAttribute = getCellAttributeManualVertexAttributes(f.neighbor_hs.at(f.boundary ? 0 : 1));
            }
            for (uint32_t v_id : f.vs) {
                vertexAttributes.push_back(vertexAttribute);
                glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                vertexPositions.push_back(vertexPosition);
                vertexNormals.push_back(-vertexNormal);
            }

            triangleIndices.push_back(idxStart+0);
            triangleIndices.push_back(idxStart+1);
            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+0);
            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+3);
        }
    }
}

void HexMesh::getVolumeData_Volume(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<float>& vertexAttributes) {
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        assert(f.vs.size() == 4);

        if (!isCellMarked(f.neighbor_hs.at(0))) {
            size_t idxStart = vertexPositions.size();
            float vertexAttribute;
            // Compute attribute data.
            if (!useManualVertexAttribute) {
                vertexAttribute = getCellAttribute(f.neighbor_hs.at(0));
            } else {
                vertexAttribute = getCellAttributeManualVertexAttributes(f.neighbor_hs.at(0));
            }
            for (uint32_t v_id : f.vs) {
                vertexAttributes.push_back(vertexAttribute);
                glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                vertexPositions.push_back(vertexPosition);
            }

            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+1);
            triangleIndices.push_back(idxStart+0);
            triangleIndices.push_back(idxStart+3);
            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+0);
        }

        if (!f.boundary && !isCellMarked(f.neighbor_hs.at(1))) {
            size_t idxStart = vertexPositions.size();
            float vertexAttribute;
            // Compute attribute data.
            if (!useManualVertexAttribute) {
                vertexAttribute = getCellAttribute(f.neighbor_hs.at(1));
            } else {
                vertexAttribute = getCellAttributeManualVertexAttributes(f.neighbor_hs.at(1));
            }
            for (uint32_t v_id : f.vs) {
                vertexAttributes.push_back(vertexAttribute);
                glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                vertexPositions.push_back(vertexPosition);
            }

            triangleIndices.push_back(idxStart+0);
            triangleIndices.push_back(idxStart+1);
            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+0);
            triangleIndices.push_back(idxStart+2);
            triangleIndices.push_back(idxStart+3);
        }
    }
}

void HexMesh::getVolumeData_FacesShared(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<float>& vertexAttributes,
        bool useVolumeWeighting) {
    rebuildInternalRepresentationIfNecessary();

    // Compute all cell volumes.
    if (useVolumeWeighting && cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    // Add all hexahedral mesh vertices to the triangle mesh vertex data.
    for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
        glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
        vertexPositions.push_back(vertexPosition);
    }
    if (!useManualVertexAttribute) {
        if (useVolumeWeighting) {
            for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
                vertexAttributes.push_back(interpolateCellAttributePerVertex(v_id, cellVolumes));
            }
        } else {
            for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
                vertexAttributes.push_back(maximumCellAttributePerVertex(v_id));
            }
        }
    } else {
        // Use manually specified attributes.
        for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
            vertexAttributes.push_back(this->manualVertexAttributes->at(v_id));
        }
    }

    // Add all triangle indices.
    triangleIndices.reserve(mesh->Fs.size() * 12);
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        assert(f.vs.size() == 4);
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
}

void HexMesh::getVolumeData_VolumeShared(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<float>& vertexAttributes) {
    rebuildInternalRepresentationIfNecessary();

    // Compute all cell volumes.
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    // Add all hexahedral mesh vertices to the triangle mesh vertex data.
    for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
        float vertexAttribute;
        if (!useManualVertexAttribute) {
            vertexAttribute = interpolateCellAttributePerVertex(v_id, cellVolumes);
        } else {
            vertexAttribute = this->manualVertexAttributes->at(v_id);
        }
        vertexAttributes.push_back(vertexAttribute);
        glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
        vertexPositions.push_back(vertexPosition);
    }

    // Add all triangle indices.
    triangleIndices.reserve(mesh->Fs.size() * 12);
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        assert(f.vs.size() == 4);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[1]);
        triangleIndices.push_back(f.vs[0]);
        triangleIndices.push_back(f.vs[3]);
        triangleIndices.push_back(f.vs[2]);
        triangleIndices.push_back(f.vs[0]);

        if (!f.boundary) {
            triangleIndices.push_back(f.vs[0]);
            triangleIndices.push_back(f.vs[1]);
            triangleIndices.push_back(f.vs[2]);
            triangleIndices.push_back(f.vs[0]);
            triangleIndices.push_back(f.vs[2]);
            triangleIndices.push_back(f.vs[3]);
        }
    }
}

void HexMesh::getSingularityData(
        std::vector<glm::vec3>& lineVertices,
        std::vector<glm::vec4>& lineColors,
        std::vector<glm::vec3>& pointVertices,
        std::vector<glm::vec4>& pointColors) {
    rebuildInternalRepresentationIfNecessary();

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
            int edgeValence = int(e.neighbor_hs.size());
            glm::vec4 vertexColor = edgeColorMap(true, e.boundary, edgeValence);
            for (uint32_t v_id : e.vs) {
                glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
                lineVertices.push_back(vertexPosition);
                lineColors.push_back(vertexColor);
                //lineColors.push_back(glm::vec4(1,0,0,1));
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
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec4>& vertexColors,
        bool cullInterior) {
    rebuildInternalRepresentationIfNecessary();
    if (!frame) computeBaseComplexMeshFrame();
    //sgl::XorshiftRandomGenerator random(10203);
    sgl::XorshiftRandomGenerator random(time(nullptr));

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

                    uint32_t offset = vertexPositions.size();
                    for (int i = 0; i < 12; i++) {
                        triangleIndices.push_back(offset + vertexIndices[i]);
                    }
                    for (size_t i = 0; i < f.vs.size(); i++) {
                        vertexPositions.push_back(quadBuffer.at(i));
                        vertexNormals.push_back(vertexNormal);
                        vertexColors.push_back(vertexColor);
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
    Hybrid_V& v_0 = mesh->Vs[v_id_0];
    Hybrid_V& v_1 = mesh->Vs[v_id_1];
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
#if _OPENMP >= 200805
    #pragma omp parallel for
#endif
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
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(max: maxValue)
#endif
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
#if _OPENMP >= 200805
    #pragma omp parallel for
#endif
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
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    for (Hybrid_E& e : mesh->Es) {
        bool isSingular = singularEdgeIds.find(e.id) != singularEdgeIds.end();
        int edgeValence = int(e.neighbor_hs.size());
        glm::vec4 vertexColor = edgeColorMap(isSingular, e.boundary, edgeValence);
        for (uint32_t v_id : e.vs) {
            glm::vec3 vertexPosition(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
            lineVertices.push_back(vertexPosition);
            lineColors.push_back(vertexColor);
        }
    }
}


Hybrid_E* HexMesh::pickNextUnvisitedNeighbor(std::unordered_set<uint32_t>& visitedEdgeIds, Hybrid_E& e, uint32_t v_id) {
    Hybrid_E* neighbor_e = nullptr;
    Hybrid_V& v = mesh->Vs.at(v_id);
    for (uint32_t neighbor_e_id : v.neighbor_es) {
        if (neighbor_e_id != e.id && visitedEdgeIds.find(neighbor_e_id) == visitedEdgeIds.end()) {
            neighbor_e = &mesh->Es.at(neighbor_e_id);
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
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec4>& vertexColors,
        std::vector<glm::vec3>& barycentricCoordinates,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();
    const glm::vec4 regularColor = useGlowColors ? glowColorRegular : outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? glowColorSingular : outlineColorSingular;

    size_t indexOffset = 0;
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

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
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);

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
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFace>& hexahedralCellFaces,
        bool onlyBoundary,
        bool useGlowColors) {
    rebuildInternalRepresentationIfNecessary();

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Fs.size(); i++) {
        Hybrid_F& f = mesh->Fs.at(i);
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        if (onlyBoundary) {
            if (!f.boundary && !std::any_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }
        }

        hexahedralCellFaces.push_back(HexahedralCellFace());
        HexahedralCellFace& hexahedralCellFace = hexahedralCellFaces.back();

        assert(f.vs.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t v_id = f.vs.at(j);
            glm::vec4 vertexPosition(
                    mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
            hexahedralCellFace.vertexPositions[j] = vertexPosition;
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
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);

        assert(f.es.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t e_id = f.es.at(j);
            Hybrid_E& e = mesh->Es.at(e_id);
            int edgeValence = int(e.neighbor_hs.size());
            glm::vec4 vertexColor = edgeColorMap(singularEdgeIds.find(e_id) != singularEdgeIds.end(), e.boundary, edgeValence);
            hexahedralCellFace.lineColors[j] = vertexColor;
        }

        indexOffset += 4;
    }
}

void HexMesh::getSurfaceDataWireframeFaces(
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFace>& hexahedralCellFaces,
        const std::vector<uint32_t>& faceIds,
        bool useSingularEdgeColorMap) {
    rebuildInternalRepresentationIfNecessary();

    size_t indexOffset = 0;
    for (size_t i = 0; i < faceIds.size(); i++) {
        Hybrid_F& f = mesh->Fs.at(faceIds.at(i));
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        hexahedralCellFaces.push_back(HexahedralCellFace());
        HexahedralCellFace& hexahedralCellFace = hexahedralCellFaces.back();

        assert(f.vs.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t v_id = f.vs.at(j);
            glm::vec4 vertexPosition(
                    mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
            hexahedralCellFace.vertexPositions[j] = vertexPosition;
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
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);

        assert(f.es.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t e_id = f.es.at(j);
            Hybrid_E& e = mesh->Es.at(e_id);
            glm::vec4 vertexColor(0.0f, 0.0f, 0.0f, 1.0f);
            if (useSingularEdgeColorMap) {
                int edgeValence = int(e.neighbor_hs.size());
                vertexColor = edgeColorMap(singularEdgeIds.find(e_id) != singularEdgeIds.end(), e.boundary, edgeValence);
            }
            hexahedralCellFace.lineColors[j] = vertexColor;
        }

        indexOffset += 4;
    }
}


uint32_t HexMesh::packEdgeSingularityInformation(uint32_t e_id) {
    Hybrid_E& e = mesh->Es.at(e_id);
    uint32_t isSingular = singularEdgeIds.find(e_id) != singularEdgeIds.end() ? 1 : 0;
    uint32_t isBoundary = e.boundary ? 1 : 0;
    uint32_t edgeValence = e.neighbor_hs.size();
    return isSingular | (isBoundary << 1) | (edgeValence << 2);
}

/*void HexMesh::getSurfaceDataWireframeFacesUnified_AttributePerCell(
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFaceUnified>& hexahedralCellFaces,
        int& maxLodValue, LodSettings lodSettings) {
    rebuildInternalRepresentationIfNecessary();

    // Compute the per-edge LOD values between 0 and 1.
    if (edgeLodValues.empty() || this->lodSettings != lodSettings) {
        edgeLodValues.clear();
        maxLodValue = 0;
        generateSheetLevelOfDetailEdgeStructure(this, edgeLodValues, &maxLodValue, lodSettings);
        this->lodSettings = lodSettings;
    }

    // Compute all cell volumes.
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    // Compute all edge attributes.
    std::vector<float> edgeAttributes(mesh->Es.size());
    for (uint32_t e_id = 0; e_id < mesh->Es.size(); e_id++) {
        float edgeAttribute = maximumCellAttributePerEdge(e_id);
        edgeAttributes.at(e_id) = edgeAttribute;
    }

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Hs.size(); i++) {
        Hybrid& h = mesh->Hs.at(i);
        for (size_t j = 0; j < h.fs.size(); j++) {
            Hybrid_F& f = mesh->Fs.at(h.fs.at(j));
            if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }

            assert(f.neighbor_hs.size() >= 1 && f.neighbor_hs.size() <= 2);
            bool invertWinding = f.neighbor_hs.at(0) != h.id;

            hexahedralCellFaces.push_back(HexahedralCellFaceUnified());
            HexahedralCellFaceUnified& hexahedralCellFace = hexahedralCellFaces.back();

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                glm::vec4 vertexPosition(
                        mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
                hexahedralCellFace.vertexPositions[j] = vertexPosition;
                hexahedralCellFace.vertexAttributes[j] = getCellAttribute(h.id);
            }

            if (!invertWinding) {
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
                 /
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
            }
            if (invertWinding || f.boundary) {
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
            }

            assert(f.es.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t e_id = f.es.at(j);
                Hybrid_E& e = mesh->Es.at(e_id);
                hexahedralCellFace.edgeAttributes[j] = edgeAttributes.at(e_id);
                hexahedralCellFace.edgeLodValues[j] = edgeLodValues.at(e_id);
                //hexahedralCellFace.edgeSingularityInformationList[j] = packEdgeSingularityInformation(e_id);
            }

            indexOffset += 4;
        }
    }
}*/

void HexMesh::getSurfaceDataWireframeFacesUnified_AttributePerVertex(
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFaceUnified>& hexahedralCellFaces,
        std::vector<HexahedralCellVertexUnified>& hexahedralCellVertices,
        std::vector<HexahedralCellEdgeUnified>& hexahedralCellEdges,
        std::vector<glm::uvec2>& hexahedralCellFacesCellLinks,
        std::vector<float>& hexahedralCells,
        bool showFocusFaces, int& maxLodValue, bool useVolumeWeighting, LodSettings lodSettings) {
    rebuildInternalRepresentationIfNecessary();

    // Compute the per-edge LOD values between 0 and 1.
    if (edgeLodValues.empty() || this->lodSettings != lodSettings) {
        edgeLodValues.clear();
        maxLodValue = 0;
        generateSheetLevelOfDetailEdgeStructure(this, edgeLodValues, &maxLodValue, lodSettings);
        this->lodSettings = lodSettings;
    }

    // Compute all cell volumes.
    if (useVolumeWeighting && cellVolumes.empty()) {
        computeAllCellVolumes();
    }


    // 1. Get vertex data (first: position).
    hexahedralCellVertices.reserve(mesh->Vs.size());
    for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
        hexahedralCellVertices.push_back(HexahedralCellVertexUnified());
        HexahedralCellVertexUnified& hexahedralCellVertex = hexahedralCellVertices.back();
        hexahedralCellVertex.vertexPosition = glm::vec3(mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id));
    }

    // 1.2 Get vertex attributes.
    if (!useManualVertexAttribute) {
        if (useVolumeWeighting) {
            for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
                HexahedralCellVertexUnified& hexahedralCellVertex = hexahedralCellVertices.at(v_id);
                hexahedralCellVertex.vertexAttribute = interpolateCellAttributePerVertex(v_id, cellVolumes);
            }
        } else {
            for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
                HexahedralCellVertexUnified& hexahedralCellVertex = hexahedralCellVertices.at(v_id);
                hexahedralCellVertex.vertexAttribute = maximumCellAttributePerVertex(v_id);
            }
        }
    } else {
        // Use manually specified attributes.
        for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
            HexahedralCellVertexUnified& hexahedralCellVertex = hexahedralCellVertices.at(v_id);
            hexahedralCellVertex.vertexAttribute = this->manualVertexAttributes->at(v_id);
        }
    }


    // 2. Edge data.
    hexahedralCellEdges.reserve(mesh->Es.size());
    if (!useManualVertexAttribute) {
        // Compute all edge attributes.
        for (uint32_t e_id = 0; e_id < mesh->Es.size(); e_id++) {
            hexahedralCellEdges.push_back(HexahedralCellEdgeUnified());
            HexahedralCellEdgeUnified& hexahedralCellEdge = hexahedralCellEdges.back();
            hexahedralCellEdge.edgeAttribute = maximumCellAttributePerEdge(e_id);
            hexahedralCellEdge.edgeLodValue = edgeLodValues.at(e_id);
        }
    } else {
        // Use manually specified attributes.
        for (uint32_t e_id = 0; e_id < mesh->Es.size(); e_id++) {
            Hybrid_E& e = mesh->Es.at(e_id);
            hexahedralCellEdges.push_back(HexahedralCellEdgeUnified());
            HexahedralCellEdgeUnified& hexahedralCellEdge = hexahedralCellEdges.back();
            hexahedralCellEdge.edgeAttribute =
                    (this->manualVertexAttributes->at(e.vs.at(0))
                     + this->manualVertexAttributes->at(e.vs.at(1))) * 0.5f;
            hexahedralCellEdge.edgeLodValue = edgeLodValues.at(e_id);
            //hexahedralCellEdge.edgeSingularityInformation = packEdgeSingularityInformation(e_id);
        }
    }


    // 3. Cell data.
    if (showFocusFaces) {
        hexahedralCells.reserve(mesh->Hs.size());
        for (uint32_t h_id = 0; h_id < mesh->Hs.size(); h_id++) {
            hexahedralCells.push_back(getCellAttribute(h_id));
        }
    }


    // 4. Face data.
    size_t indexOffset = 0;
    hexahedralCellFaces.reserve(mesh->Fs.size());
    if (showFocusFaces) {
        hexahedralCellFacesCellLinks.reserve(mesh->Fs.size());
    }
    for (size_t f_id = 0; f_id < mesh->Fs.size(); f_id++) {
        Hybrid_F& f = mesh->Fs.at(f_id);
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        hexahedralCellFaces.push_back(HexahedralCellFaceUnified());
        HexahedralCellFaceUnified& hexahedralCellFace = hexahedralCellFaces.back();
        hexahedralCellFace.vertexIdx[0];
        hexahedralCellFace.edgeIdx[0];

        if (showFocusFaces) {
            glm::uvec2 cellLinks = glm::uvec2(0xFFFFFFFF, 0xFFFFFFFF);
            assert(f.neighbor_hs.size() == 1 || f.neighbor_hs.size() == 2);
            for (size_t i = 0; i < f.neighbor_hs.size(); i++) {
                cellLinks[i] = f.neighbor_hs.at(i);
            }
            hexahedralCellFacesCellLinks.push_back(cellLinks);

        }

        assert(f.vs.size() == 4);
        for (size_t i = 0; i < 4; i++) {
            hexahedralCellFace.vertexIdx[i] = f.vs.at(i);
        }

        assert(f.es.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            hexahedralCellFace.edgeIdx[j] = f.es.at(j);
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
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);

        indexOffset += 4;
    }
}

void HexMesh::getSurfaceDataWireframeFacesUnified_AttributePerCell_Volume2(
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFaceUnified_Volume2>& hexahedralCellFaces,
        int& maxLodValue) {
    rebuildInternalRepresentationIfNecessary();

    // Compute the per-edge LOD values between 0 and 1.
    LodSettings lodSettings;
    if (edgeLodValues.empty() || this->lodSettings != lodSettings) {
        edgeLodValues.clear();
        maxLodValue = 0;
        generateSheetLevelOfDetailEdgeStructure(this, edgeLodValues, &maxLodValue, lodSettings);
        this->lodSettings = lodSettings;
    }

    // Compute all cell volumes.
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    // Compute all edge attributes.
    std::vector<float> edgeAttributes(mesh->Es.size());
    for (uint32_t e_id = 0; e_id < mesh->Es.size(); e_id++) {
        float edgeAttribute = maximumCellAttributePerEdge(e_id);
        edgeAttributes.at(e_id) = edgeAttribute;
    }

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Hs.size(); i++) {
        Hybrid& h = mesh->Hs.at(i);
        for (size_t j = 0; j < h.fs.size(); j++) {
            Hybrid_F& f = mesh->Fs.at(h.fs.at(j));
            if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return isCellMarked(h_id);
            })) {
                continue;
            }

            assert(f.neighbor_hs.size() >= 1 && f.neighbor_hs.size() <= 2);
            bool invertWinding = f.neighbor_hs.at(0) != h.id;

            hexahedralCellFaces.push_back(HexahedralCellFaceUnified_Volume2());
            HexahedralCellFaceUnified_Volume2& hexahedralCellFace = hexahedralCellFaces.back();
            hexahedralCellFace.bitfield[0] = hexahedralCellFace.bitfield[1] = hexahedralCellFace.bitfield[2]
                    = hexahedralCellFace.bitfield[3] = 0u;

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                glm::vec4 vertexPosition(
                        mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
                hexahedralCellFace.vertexPositions[j] = vertexPosition;
                hexahedralCellFace.vertexAttributes[j] = getCellAttribute(h.id);
            }

            if (!invertWinding) {
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
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
            }
            if (invertWinding || f.boundary) {
                if (f.boundary) {
                    indexOffset += 4;
                }
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
            }

            assert(f.es.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t e_id = f.es.at(j);
                Hybrid_E& e = mesh->Es.at(e_id);
                hexahedralCellFace.edgeAttributes[j] = edgeAttributes.at(e_id);
                hexahedralCellFace.edgeLodValues[j] = edgeLodValues.at(e_id);
                hexahedralCellFace.edgeSingularityInformationList[j] = packEdgeSingularityInformation(e_id);
            }

            indexOffset += 4;

            if (f.boundary) {
                hexahedralCellFaces.push_back(HexahedralCellFaceUnified_Volume2());
                HexahedralCellFaceUnified_Volume2& hexahedralCellBackface = hexahedralCellFaces.back();
                hexahedralCellBackface.bitfield[0] = hexahedralCellBackface.bitfield[1] = hexahedralCellBackface.bitfield[2]
                        = hexahedralCellBackface.bitfield[3] = 1u;

                assert(f.vs.size() == 4);
                for (size_t j = 0; j < 4; j++) {
                    uint32_t v_id = f.vs.at(j);
                    glm::vec4 vertexPosition(
                            mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
                    hexahedralCellBackface.vertexPositions[j] = vertexPosition;
                    hexahedralCellBackface.vertexAttributes[j] = getCellAttribute(h.id);
                }

                assert(f.es.size() == 4);
                for (size_t j = 0; j < 4; j++) {
                    uint32_t e_id = f.es.at(j);
                    Hybrid_E& e = mesh->Es.at(e_id);
                    hexahedralCellBackface.edgeAttributes[j] = edgeAttributes.at(e_id);
                    hexahedralCellBackface.edgeLodValues[j] = edgeLodValues.at(e_id);
                    hexahedralCellBackface.edgeSingularityInformationList[j] = packEdgeSingularityInformation(e_id);
                }
            }
        }
    }
}

void HexMesh::getSurfaceDataWireframeFacesUnified_AttributePerVertex_Volume2(
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFaceUnified_Volume2>& hexahedralCellFaces,
        int& maxLodValue) {
    rebuildInternalRepresentationIfNecessary();

    // Compute the per-edge LOD values between 0 and 1.
    LodSettings lodSettings;
    if (edgeLodValues.empty() || this->lodSettings != lodSettings) {
        edgeLodValues.clear();
        maxLodValue = 0;
        generateSheetLevelOfDetailEdgeStructure(this, edgeLodValues, &maxLodValue, lodSettings);
        this->lodSettings = lodSettings;
    }

    // Compute all cell volumes.
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    // Compute all vertex attributes.
    std::vector<float> vertexAttributes(mesh->Vs.size());
    for (uint32_t v_id = 0; v_id < mesh->Vs.size(); v_id++) {
        float vertexAttribute = interpolateCellAttributePerVertex(v_id, cellVolumes);
        vertexAttributes.at(v_id) = vertexAttribute;
    }

    // Compute all edge attributes.
    std::vector<float> edgeAttributes(mesh->Es.size());
    for (uint32_t e_id = 0; e_id < mesh->Es.size(); e_id++) {
        float edgeAttribute = maximumCellAttributePerEdge(e_id);
        edgeAttributes.at(e_id) = edgeAttribute;
    }

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Fs.size(); i++) {
        Hybrid_F& f = mesh->Fs.at(i);
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        hexahedralCellFaces.push_back(HexahedralCellFaceUnified_Volume2());
        HexahedralCellFaceUnified_Volume2& hexahedralCellFace = hexahedralCellFaces.back();
        hexahedralCellFace.bitfield[0] = hexahedralCellFace.bitfield[1] = hexahedralCellFace.bitfield[2]
                = hexahedralCellFace.bitfield[3] = 0u;

        assert(f.vs.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t v_id = f.vs.at(j);
            glm::vec4 vertexPosition(
                    mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
            hexahedralCellFace.vertexPositions[j] = vertexPosition;
            hexahedralCellFace.vertexAttributes[j] = vertexAttributes.at(v_id);
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
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);

        // Backface
        if (f.boundary) {
            indexOffset += 4;
        }
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);

        indexOffset += 4;

        assert(f.es.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t e_id = f.es.at(j);
            Hybrid_E& e = mesh->Es.at(e_id);
            hexahedralCellFace.edgeAttributes[j] = edgeAttributes.at(e_id);
            hexahedralCellFace.edgeLodValues[j] = edgeLodValues.at(e_id);
            hexahedralCellFace.edgeSingularityInformationList[j] = packEdgeSingularityInformation(e_id);
        }

        if (f.boundary) {
            hexahedralCellFaces.push_back(HexahedralCellFaceUnified_Volume2());
            HexahedralCellFaceUnified_Volume2& hexahedralCellBackface = hexahedralCellFaces.back();
            hexahedralCellBackface.bitfield[0] = hexahedralCellBackface.bitfield[1] = hexahedralCellBackface.bitfield[2]
                    = hexahedralCellBackface.bitfield[3] = 1u;

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                glm::vec4 vertexPosition(
                        mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
                hexahedralCellBackface.vertexPositions[j] = vertexPosition;
                hexahedralCellBackface.vertexAttributes[j] = vertexAttributes.at(v_id);
            }

            assert(f.es.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t e_id = f.es.at(j);
                Hybrid_E& e = mesh->Es.at(e_id);
                hexahedralCellBackface.edgeAttributes[j] = edgeAttributes.at(e_id);
                hexahedralCellBackface.edgeLodValues[j] = edgeLodValues.at(e_id);
                hexahedralCellBackface.edgeSingularityInformationList[j] = packEdgeSingularityInformation(e_id);
            }
        }
    }
}

void HexMesh::getVolumeData_DepthComplexity(
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions) {
    rebuildInternalRepresentationIfNecessary();

    for (Hybrid_V& v : mesh->Vs) {
        glm::vec3 vertexPosition(mesh->V(0, v.id), mesh->V(1, v.id), mesh->V(2, v.id));
        vertexPositions.push_back(vertexPosition);
    }

    size_t indexOffset = 0;
    for (Hybrid_F& f : mesh->Fs) {
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        assert(f.vs.size() == 4);
        uint32_t v_ids[4];
        for (size_t j = 0; j < 4; j++) {
            v_ids[j] = f.vs.at(j);
        }

        triangleIndices.push_back(v_ids[0]);
        triangleIndices.push_back(v_ids[3]);
        triangleIndices.push_back(v_ids[1]);
        triangleIndices.push_back(v_ids[2]);
        triangleIndices.push_back(v_ids[1]);
        triangleIndices.push_back(v_ids[3]);

        indexOffset += 4;
    }
}

void HexMesh::getSurfaceDataWireframeFacesLineDensityControl(
        std::vector<uint32_t>& triangleIndices,
        std::vector<HexahedralCellFaceLineDensityControl>& hexahedralCellFaces,
        int& maxLodValue) {
    rebuildInternalRepresentationIfNecessary();

    // Compute the per-edge LOD values between 0 and 1.
    LodSettings lodSettings;
    if (edgeLodValues.empty() || this->lodSettings != lodSettings) {
        edgeLodValues.clear();
        maxLodValue = 0;
        generateSheetLevelOfDetailEdgeStructure(this, edgeLodValues, &maxLodValue, lodSettings);
        this->lodSettings = lodSettings;
    }

    // Compute all cell volumes.
    if (cellVolumes.empty()) {
        computeAllCellVolumes();
    }

    // Compute all edge attributes.
    std::vector<float> edgeAttributes(mesh->Es.size());
    for (uint32_t e_id = 0; e_id < mesh->Es.size(); e_id++) {
        float edgeAttribute = interpolateCellAttributePerEdge(e_id, cellVolumes);
        edgeAttributes.at(e_id) = edgeAttribute;
    }

    size_t indexOffset = 0;
    for (size_t i = 0; i < mesh->Fs.size(); i++) {
        Hybrid_F& f = mesh->Fs.at(i);
        if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
            return isCellMarked(h_id);
        })) {
            continue;
        }

        hexahedralCellFaces.push_back(HexahedralCellFaceLineDensityControl());
        HexahedralCellFaceLineDensityControl& hexahedralCellFace = hexahedralCellFaces.back();

        assert(f.vs.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t v_id = f.vs.at(j);
            glm::vec4 vertexPosition(
                    mesh->V(0, v_id), mesh->V(1, v_id), mesh->V(2, v_id), 1.0f);
            hexahedralCellFace.vertexPositions[j] = vertexPosition;
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
        triangleIndices.push_back(indexOffset + 0);
        triangleIndices.push_back(indexOffset + 3);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 2);
        triangleIndices.push_back(indexOffset + 1);
        triangleIndices.push_back(indexOffset + 3);

        assert(f.es.size() == 4);
        for (size_t j = 0; j < 4; j++) {
            uint32_t e_id = f.es.at(j);
            Hybrid_E& e = mesh->Es.at(e_id);
            hexahedralCellFace.edgeAttributes[j] = edgeAttributes.at(e_id);
            hexahedralCellFace.edgeLodValues[j] = edgeLodValues.at(e_id);
            hexahedralCellFace.edgeSingularityInformationList[j] = packEdgeSingularityInformation(e_id);
        }

        indexOffset += 4;
    }
}
