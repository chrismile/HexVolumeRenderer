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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "Mesh/HexMesh/Renderers/Helpers/Sphere.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "Mesh/HexMesh/Renderers/LOD/LodSheetGeneration.hpp"
#include "LodLinePreviewRenderer_Sheets.hpp"

LodLinePreviewRenderer_Sheets::LodLinePreviewRenderer_Sheets(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    shaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeLod.Vertex", "WireframeLod.Geometry", "WireframeLod.Fragment.Preview"});
}

void LodLinePreviewRenderer_Sheets::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    hexMesh = meshIn;
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec4> colors;
    std::vector<float> lodValues;
    generateSheetLevelOfDetailLineStructureAndVertexData(
            meshIn.get(), vertices, colors, lodValues, nullptr, lodSettings);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertices.size()*sizeof(glm::vec3), vertices.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr colorBuffer = sgl::Renderer->createGeometryBuffer(
            colors.size()*sizeof(glm::vec4), colors.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            colorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);

    // Add the LOD value buffer.
    sgl::GeometryBufferPtr lodValueBuffer = sgl::Renderer->createGeometryBuffer(
            lodValues.size()*sizeof(float), lodValues.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            lodValueBuffer, "vertexLodValue", sgl::ATTRIB_FLOAT, 1);

    dirty = false;
    reRender = true;
}

void LodLinePreviewRenderer_Sheets::render() {
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform("maxLod", maxLod);

    // Render the LOD lines.
    shaderProgram->setUniform("lineWidth", lineWidth);
    sgl::Renderer->render(shaderAttributes);
}

void LodLinePreviewRenderer_Sheets::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::SliderFloat("Maximum LOD", &maxLod, 0.0f, 1.0f)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(
                "LOD Merge Factor", &lodSettings.lodMergeFactor, 0.999f, 4.0f, "%.3f")) {
            if (hexMesh) {
                uploadVisualizationMapping(hexMesh, false);
                reRender = true;
            }
        }
        if (ImGui::Checkbox("Use Volume and Area Measures", &lodSettings.useVolumeAndAreaMeasures)) {
            if (hexMesh) {
                uploadVisualizationMapping(hexMesh, false);
                reRender = true;
            }
        }
        if (ImGui::Checkbox("Use Weights for Merging", &lodSettings.useWeightsForMerging)) {
            if (hexMesh) {
                uploadVisualizationMapping(hexMesh, false);
                reRender = true;
            }
        }
    }
    ImGui::End();
}
