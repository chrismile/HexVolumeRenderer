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

#include <ImGui/ImGuiWrapper.hpp>

#include "Mesh/BaseComplex/global_types.h"
#include "PlaneFilter.hpp"

void PlaneFilter::filterMesh(HexMeshPtr meshIn) {
    output = meshIn;
    Mesh& mesh = meshIn->getBaseComplexMesh();
    glm::vec3 normalizedDirection = glm::normalize(direction);

    float minOffset = std::numeric_limits<float>::max();
    float maxOffset = std::numeric_limits<float>::lowest();
    for (Hybrid_V& v : mesh.Vs) {
        float offset = glm::dot(normalizedDirection,
                glm::vec3(mesh.V(0, v.id), mesh.V(1, v.id), mesh.V(2, v.id)));
        minOffset = std::min(minOffset, offset);
        maxOffset = std::max(maxOffset, offset);
    }
    float offset = minOffset + (maxOffset - minOffset) * filterRatio;

    for (Hybrid& h : mesh.Hs) {
        for (uint32_t v_id : h.vs) {
            if (glm::dot(normalizedDirection,
                    glm::vec3(mesh.V(0, v_id), mesh.V(1, v_id), mesh.V(2, v_id))) < offset) {
                meshIn->markCell(h.id);
                break;
            }
        }
    }
    dirty = false;
}

void PlaneFilter::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3025, 1508, 800, 160);
    if (ImGui::Begin("Plane Filter", &showFilterWindow)) {
        if (ImGui::SliderFloat("Slice", &filterRatio, 0.0f, 1.0f)) {
            dirty = true;
        }
        if (ImGui::InputFloat3("Direction", &direction.x)) {
            dirty = true;
        }
    }
    ImGui::End();
}
