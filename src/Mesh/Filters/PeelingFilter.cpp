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

#include <unordered_set>
#include <limits>
#include <Utils/CircularQueue.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Mesh/BaseComplex/global_types.h"
#include "PeelingFilter.hpp"

void PeelingFilter::onMeshLoaded(HexMeshPtr meshIn) {
    if (!meshIn->isBaseComplexMeshLoaded()) {
        return;
    }

    cellDepths.clear();
    peelingDepth = 0;
    maxPeelingDepth = 0;

    CircularQueue<uint32_t> openList(32);

    Mesh& mesh = meshIn->getBaseComplexMesh();
    cellDepths.resize(mesh.Hs.size(), std::numeric_limits<int>::max());
    for (Hybrid& h : mesh.Hs) {
        if (std::any_of(h.fs.begin(), h.fs.end(), [this, &mesh](uint32_t f_id) {
            return mesh.Fs.at(f_id).boundary;
        })) {
            cellDepths.at(h.id) = 0;
            openList.push_back(h.id);
        }
    }

    while (!openList.is_empty()) {
        uint32_t h_id = openList.pop_front();
        Hybrid& h = mesh.Hs.at(h_id);
        int cellDepth = cellDepths.at(h_id);
        for (uint32_t f_id : h.fs) {
            Hybrid_F& f = mesh.Fs.at(f_id);
            if (!f.boundary) {
                uint32_t neighbor_h_id = f.neighbor_hs.at(0) == h_id ? f.neighbor_hs.at(1) : f.neighbor_hs.at(0);
                int neighborDepth = cellDepths.at(neighbor_h_id);
                if (neighborDepth > cellDepth + 1) {
                    openList.push_back(neighbor_h_id);
                    cellDepths.at(neighbor_h_id) = cellDepth + 1;
                    maxPeelingDepth = std::max(maxPeelingDepth, cellDepth + 1);
                }
            }
        }
    }

    cellDepths.shrink_to_fit();
}

void PeelingFilter::filterMesh(HexMeshPtr meshIn) {
    if (cellDepths.empty()) {
        onMeshLoaded(meshIn);
    }

    output = meshIn;
    Mesh& mesh = meshIn->getBaseComplexMesh();
    for (Hybrid& h : mesh.Hs) {
        if (cellDepths.at(h.id) < peelingDepth) {
            meshIn->markCell(h.id);
        }
    }
    dirty = false;
}

void PeelingFilter::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3025, 1671, 800, 110);
    if (ImGui::Begin("Peeling Filter", &showFilterWindow)) {
        if (ImGui::SliderInt("Depth", &peelingDepth, 0, maxPeelingDepth)) {
            dirty = true;
        }
    }
    ImGui::End();
}
