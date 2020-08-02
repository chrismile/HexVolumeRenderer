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

#include "PlaneFilter.hpp"

void PlaneFilter::filterMesh(HexMeshPtr meshIn) {
    output = meshIn;
    HexaLab::Mesh& mesh = meshIn->getHexaLabMesh();
    glm::vec3 normalizedDirection = glm::normalize(direction);

    float minOffset = FLT_MAX;
    float maxOffset = -FLT_MAX;
    for (const HexaLab::Vert &vert : mesh.verts) {
        float offset = glm::dot(normalizedDirection, vert.position);
        minOffset = std::min(minOffset, offset);
        maxOffset = std::max(maxOffset, offset);
    }
    float offset = minOffset + (maxOffset - minOffset) * filterRatio;

    for (size_t i = 0; i < mesh.cells.size(); ++i) {
        HexaLab::Cell& cell = mesh.cells.at(i);
        HexaLab::MeshNavigator nav = mesh.navigate(cell);

        HexaLab::MeshNavigator cellNav = mesh.navigate(nav.face());
        for (int v = 0; v < 8; ++v) {
            if (v == 4) {
                nav = nav.rotate_on_cell().rotate_on_cell();
                cellNav = mesh.navigate(nav.face());
            }

            if (glm::dot(normalizedDirection, cellNav.vert().position) < offset) {
                mesh.mark(nav.cell());
                break;
            }
            cellNav = cellNav.rotate_on_face();
        }
    }
    dirty = false;
}

void PlaneFilter::renderGui() {
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
