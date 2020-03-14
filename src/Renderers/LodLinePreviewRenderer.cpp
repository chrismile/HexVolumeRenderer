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
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "Helpers/Sphere.hpp"
#include "LodLinePreviewRenderer.hpp"

LodLinePreviewRenderer::LodLinePreviewRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    shaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeLod.Vertex", "WireframeLod.Geometry", "WireframeLod.Fragment.Preview"});
}

void LodLinePreviewRenderer::generateVisualizationMapping(HexMeshPtr meshIn) {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec4> colors;
    std::vector<float> lodValues;
    meshIn->getLodLineRepresentation(vertices, colors, lodValues, true);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertices.size()*sizeof(glm::vec3), (void*)&vertices.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr colorBuffer = sgl::Renderer->createGeometryBuffer(
            colors.size()*sizeof(glm::vec4), (void*)&colors.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            colorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);

    // Add the LOD value buffer.
    sgl::GeometryBufferPtr lodValueBuffer = sgl::Renderer->createGeometryBuffer(
            lodValues.size()*sizeof(float), (void*)&lodValues.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            lodValueBuffer, "vertexLodValue", sgl::ATTRIB_FLOAT, 1);

    dirty = false;
    reRender = true;
}

void LodLinePreviewRenderer::render() {
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform("maxLod", maxLod);

    // Render the LOD lines.
    shaderProgram->setUniform("lineWidth", lineWidth);
    sgl::Renderer->render(shaderAttributes);
}

void LodLinePreviewRenderer::renderGui() {
    if (ImGui::Begin("Line LOD Preview Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Maximum LOD", &maxLod, 0.0f, 1.0f)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Line Width", &lineWidth, 0.0001f, 0.002f, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();
}
