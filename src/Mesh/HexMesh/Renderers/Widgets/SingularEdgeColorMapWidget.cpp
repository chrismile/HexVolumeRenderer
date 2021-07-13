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

#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>

#include "Mesh/BaseComplex/global_types.h"

#include "SingularEdgeColorMapWidget.hpp"

SingularEdgeColorMapWidget::SingularEdgeColorMapWidget() {
    updateSingularEdgeColorLookupTexture();
}

void SingularEdgeColorMapWidget::updateSingularEdgeColorLookupTexture() {
    const int NUM_VALENCE_LEVELS = 8; // Handle valence 1 to 8.
    glm::vec4 textureData[NUM_VALENCE_LEVELS*2];
    for (int isBoundary = 0; isBoundary <= 1; isBoundary++) {
        for (int valence = 1; valence <= NUM_VALENCE_LEVELS; valence++) {
            bool isSingular = isBoundary ? valence != 2 : valence != 4;
            if (singularEdgesColorByValence) {
                textureData[isBoundary * NUM_VALENCE_LEVELS + valence - 1] =
                        HexMesh::edgeColorMap(isSingular, isBoundary, valence);
            } else {
                textureData[isBoundary * NUM_VALENCE_LEVELS + valence - 1] =
                        isSingular ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)
                                   : glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            }
        }
    }

    if (singularEdgeColorLookupTexture) {
        singularEdgeColorLookupTexture->uploadPixelData(
                NUM_VALENCE_LEVELS, 2, textureData, sgl::PixelFormat(GL_RGBA, GL_FLOAT));
    } else {
        sgl::TextureSettings textureSettings;
        textureSettings.textureMinFilter = GL_NEAREST;
        textureSettings.textureMagFilter = GL_NEAREST;
        textureSettings.internalFormat = GL_RGBA32F;
        singularEdgeColorLookupTexture = sgl::TextureManager->createTexture(
                textureData, NUM_VALENCE_LEVELS, 2, sgl::PixelFormat(GL_RGBA, GL_FLOAT), textureSettings);
    }
}

void SingularEdgeColorMapWidget::generateSingularityStructureInformation(HexMeshPtr meshIn) {
    // Get the information about the singularity structure.
    singularEdgeMap.clear();
    Mesh& baseComplexMesh = meshIn->getBaseComplexMesh();
    Singularity& si = meshIn->getBaseComplexMeshSingularity();
    std::unordered_set<uint32_t> singularEdgeIds = meshIn->getSingularEdgeIds();
    for (uint32_t e_id : singularEdgeIds) {
        Hybrid_E& e = baseComplexMesh.Es.at(e_id);
        SingularityInformation singularityInformation(e.boundary, e.neighbor_hs.size());
        auto it = singularEdgeMap.find(singularityInformation);
        if (it != singularEdgeMap.end()) {
            it->second++;
        } else {
            singularEdgeMap.insert(std::make_pair(singularityInformation, 1u));
        }
    }
}

bool SingularEdgeColorMapWidget::renderGui() {
    bool reRender = false;

    if (ImGui::Begin("Singular Edge Color Map Widget", &showWindow)) {
        if (ImGui::Checkbox("Color Singular Edges by Valence", &singularEdgesColorByValence)) {
            updateSingularEdgeColorLookupTexture();
            reRender = true;
        }
        if (singularEdgesColorByValence && !singularEdgeMap.empty()) {
            ImGui::Text("Singularity Information:");
            ImGui::Columns(4, "ColorMapColumns");
            ImGui::Separator();
            ImGui::Text("Location"); ImGui::NextColumn();
            ImGui::Text("Valence"); ImGui::NextColumn();
            ImGui::Text("Edge Color"); ImGui::NextColumn();
            ImGui::Text("Occurrences"); ImGui::NextColumn();
            ImGui::Separator();
            int i = 0;
            for (auto& it : singularEdgeMap) {
                const SingularityInformation& singularityInformation = it.first;
                ImGui::Text(singularityInformation.isBoundary ? "Boundary" : "Interior"); ImGui::NextColumn();
                ImGui::Text("%u", singularityInformation.valence); ImGui::NextColumn();
                glm::vec4 color = HexMesh::edgeColorMap(
                        true, singularityInformation.isBoundary, singularityInformation.valence);
                std::string colorEditId = std::string() + "##color_" + std::to_string(i);
                ImGui::ColorEdit3(colorEditId.c_str(), &color.x,
                                  ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoPicker); ImGui::NextColumn();
                ImGui::Text("%u", it.second); ImGui::NextColumn();
                i++;
            }
            ImGui::Columns(1);
            ImGui::Separator();
        }
    }
    ImGui::End();

    return reRender;
}