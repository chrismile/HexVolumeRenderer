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

#ifndef GENERALMAP_GENERALMAP_HPP
#define GENERALMAP_GENERALMAP_HPP

#include <vector>
#include <memory>

#include <glm/vec3.hpp>

#include <Math/Geometry/AABB3.hpp>

#include "QualityMeasure/QualityMeasure.hpp"
#include "Renderers/TransferFunctionWindow.hpp"
#include "HexaLab/builder.h"
#include "HexaLab/app.h"

class HexMesh;
class Mesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

class HexMesh {
public:
    HexMesh(TransferFunctionWindow &transferFunctionWindow) : transferFunctionWindow(transferFunctionWindow) {}
    ~HexMesh();
    void setHexMeshData(const std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices);
    void setQualityMeasure(QualityMeasure qualityMeasure);
    void onTransferFunctionMapRebuilt();
    void unmark();
    bool isDirty() { return dirty; }

    // For filters
    HexaLab::Mesh& getMesh();

    void getSurfaceData(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec3>& normals,
            std::vector<glm::vec4>& colors);
    void getWireframeData(
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec4>& colors);
    void getVolumeData(
            std::vector<uint32_t>& indices,
            std::vector<glm::vec3>& vertices,
            std::vector<glm::vec3>& normals,
            std::vector<glm::vec4>& colors);
    void getSingularityData(
            std::vector<glm::vec3>& lineVertices,
            std::vector<glm::vec4>& lineColors,
            std::vector<glm::vec3>& pointVertices,
            std::vector<glm::vec4>& pointColors);
    void getBaseComplexDataWireframe(
            std::vector<glm::vec3>& lineVertices,
            std::vector<glm::vec4>& lineColors,
            std::vector<glm::vec3>& pointVertices,
            std::vector<glm::vec4>& pointColors);
    void getBaseComplexDataSurface(
            std::vector<glm::vec3>& triangleVertices,
            std::vector<glm::vec4>& vertexColors);
    void getLodRepresentation(
            std::vector<glm::vec3>& lineVertices,
            std::vector<uint32_t>& lineLodValues);

private:
    void computeBaseComplexMesh(const std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices);
    void recomputeHistogram();
    QualityMeasure qualityMeasure = QUALITY_MEASURE_SCALED_JACOBIAN;
    TransferFunctionWindow &transferFunctionWindow;
    bool dirty = false;

    // HexaLab data
    HexaLab::App* hexaLabApp = nullptr;
    Mesh* baseComplexMesh = nullptr;
    HexaLab::QualityMeasureEnum hexaLabQualityMeasure;
};


#endif //GENERALMAP_GENERALMAP_HPP
