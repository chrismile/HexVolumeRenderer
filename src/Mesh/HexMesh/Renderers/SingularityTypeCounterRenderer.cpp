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

#include "SingularityTypeCounterRenderer.hpp"
#include "Mesh/BaseComplex/global_types.h"

SingularityTypeCounterRenderer::SingularityTypeCounterRenderer(
        SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
}

void SingularityTypeCounterRenderer::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    Mesh& mesh = meshIn->getBaseComplexMesh();
    Singularity& si = meshIn->getBaseComplexMeshSingularity();

    numCircleSEs = 0;
    numOpenSEs = 0;
    irregularBoundaryEdgeMap.clear();
    irregularInnerEdgeMap.clear();

    std::unordered_set<uint32_t> singularEdgeIds;
    for (Singular_E& se : si.SEs) {
        if (se.circle) {
            numCircleSEs++;
        } else {
            numOpenSEs++;
        }
        for (uint32_t e_id : se.es_link) {
            if (singularEdgeIds.find(e_id) != singularEdgeIds.end()) {
                continue;
            }

            Hybrid_E& e = mesh.Es.at(e_id);
            unsigned int edgeValence = e.neighbor_hs.size();
            std::map<unsigned int, unsigned int>* irregularEdgeMap;
            if (e.boundary) {
                irregularEdgeMap = &irregularBoundaryEdgeMap;
            } else {
                irregularEdgeMap = &irregularInnerEdgeMap;
            }
            auto it = irregularEdgeMap->find(edgeValence);
            if (it != irregularEdgeMap->end()) {
                it->second++;
            } else {
                irregularEdgeMap->insert(std::make_pair(edgeValence, 1u));
            }
            singularEdgeIds.insert(e_id);
        }
    }

    dirty = false;
    reRender = true;
}

void SingularityTypeCounterRenderer::render() {
}

void SingularityTypeCounterRenderer::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        ImGui::Text("#Circle SEs: %u", numCircleSEs);
        ImGui::Text("#Open SEs: %u", numOpenSEs);
        ImGui::Separator();
        for (auto it = irregularBoundaryEdgeMap.begin(); it != irregularBoundaryEdgeMap.end(); it++) {
            ImGui::Text("#Boundary valence %u edges: %u", it->first, it->second);
        }
        for (auto it = irregularInnerEdgeMap.begin(); it != irregularInnerEdgeMap.end(); it++) {
            ImGui::Text("#Inner valence %u edges: %u", it->first, it->second);
        }
    }
    ImGui::End();
}
