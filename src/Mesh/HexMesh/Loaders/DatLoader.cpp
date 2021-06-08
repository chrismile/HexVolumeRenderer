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

#include <cassert>

#include <Utils/File/LineReader.hpp>

#include "DatLoader.hpp"

#define PT_IDXn(x, y, z) ((x) + ((y) + (z) * (numCellsY + 1)) * (numCellsX + 1))

bool DatCartesianGridLoader::loadHexahedralMeshFromFile(
        const std::string& filename,
        std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices,
        std::vector<glm::vec3>& deformations, std::vector<float>& attributeList,
        bool& isPerVertexData) {
    sgl::LineReader lineReader(filename);

    std::vector<uint32_t> numCellsLine = lineReader.readVectorLine<uint32_t>(3);
    uint32_t numCellsX = numCellsLine.at(0);
    uint32_t numCellsY = numCellsLine.at(1);
    uint32_t numCellsZ = numCellsLine.at(2);
    uint32_t numCellsTotal = numCellsX * numCellsY * numCellsZ;

    // Add the vertices.
    for (uint32_t z = 0; z < numCellsZ + 1; z++) {
        for (uint32_t y = 0; y < numCellsY + 1; y++) {
            for (uint32_t x = 0; x < numCellsX + 1; x++) {
                vertices.emplace_back(glm::vec3(x, y, z));
            }
        }
    }

    // Add the cell IDs.
    for (uint32_t z = 0; z < numCellsZ; z++) {
        for (uint32_t y = 0; y < numCellsY; y++) {
            for (uint32_t x = 0; x < numCellsX; x++) {
                cellIndices.emplace_back(PT_IDXn(x+0, y+0, z+0));
                cellIndices.emplace_back(PT_IDXn(x+1, y+0, z+0));
                cellIndices.emplace_back(PT_IDXn(x+1, y+1, z+0));
                cellIndices.emplace_back(PT_IDXn(x+0, y+1, z+0));
                cellIndices.emplace_back(PT_IDXn(x+0, y+0, z+1));
                cellIndices.emplace_back(PT_IDXn(x+1, y+0, z+1));
                cellIndices.emplace_back(PT_IDXn(x+1, y+1, z+1));
                cellIndices.emplace_back(PT_IDXn(x+0, y+1, z+1));
            }
        }
    }

    attributeList.reserve(numCellsTotal);
    for (uint32_t cellIdx = 0; cellIdx < numCellsTotal; cellIdx++) {
        float cellAttribute = lineReader.readScalarLine<float>();
        attributeList.push_back(cellAttribute);
    }
    isPerVertexData = false; //< Per-cell data!

    return true;
}


std::vector<std::vector<float>> loadDatData(const std::string& filename) {
    sgl::LineReader lineReader(filename);
    std::vector<std::vector<float>> attributesList;
    std::vector<float> lineData;
    int lineId = 0;
    size_t numAttributes = 0;
    while (lineReader.isLineLeft()) {
        lineReader.readVectorLine(lineData);
        if (lineId == 0) {
            numAttributes = lineData.size();
            attributesList.resize(numAttributes);
        }
        assert(numAttributes == lineData.size());
        for (size_t attrIdx = 0; attrIdx < numAttributes; attrIdx++) {
            attributesList.at(attrIdx).push_back(lineData.at(attrIdx));
        }
        lineId++;
    }
    return attributesList;
}
