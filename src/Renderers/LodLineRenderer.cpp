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


#include "LodLineRenderer.hpp"

#include <Graphics/Renderer.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "LodLineRenderer.hpp"

LodLineRenderer::LodLineRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    shaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeLod.Vertex", "WireframeLod.Geometry", "WireframeLod.Fragment"});

    shaderProgramPoints = sgl::ShaderManager->getShaderProgram({"Point.Vertex", "Point.Geometry", "Point.Fragment"});
    pointShaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgramPoints);
    pointShaderAttributes->setVertexMode(sgl::VERTEX_MODE_POINTS);
    focusPointBuffer = sgl::Renderer->createGeometryBuffer(sizeof(glm::vec3), &focusPoint.x, sgl::VERTEX_BUFFER);
    pointShaderAttributes->addGeometryBuffer(
            focusPointBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    glm::vec4 pointColor(1.0f, 0.0f, 0.0f, 1.0f);
    sgl::GeometryBufferPtr pointColorBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec4), &pointColor.x, sgl::VERTEX_BUFFER);
    pointShaderAttributes->addGeometryBuffer(
            pointColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);
}

void LodLineRenderer::generateVisualizationMapping(HexMeshPtr meshIn) {
    std::vector<glm::vec3> vertices;
    std::vector<float> lodValues;
    meshIn->getLodRepresentation(vertices, lodValues);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertices.size()*sizeof(glm::vec3), (void*)&vertices.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the LOD value buffer.
    sgl::GeometryBufferPtr lodValueBuffer = sgl::Renderer->createGeometryBuffer(
            lodValues.size()*sizeof(float), (void*)&lodValues.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            lodValueBuffer, "vertexLodValue", sgl::ATTRIB_FLOAT, 1);

    // Update the position of the focus point.
    focusPointBuffer->subData(0, sizeof(glm::vec3), &focusPoint.x);

    dirty = false;
    reRender = true;
}

void LodLineRenderer::render() {
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform("focusPoint", focusPoint);
    shaderProgram->setUniform("maxDistance", maxDistance);
    shaderProgramPoints->setUniform("cameraPosition", sceneData.camera->getPosition());

    // Render the LOD lines.
    shaderProgram->setUniform("lineWidth", 0.001f);
    sgl::Renderer->render(shaderAttributes);

    // Render the focus point.
    shaderProgramPoints->setUniform("radius", 0.008f);
    sgl::Renderer->render(pointShaderAttributes);
}

void LodLineRenderer::renderGui() {
    if (ImGui::Begin("Line LOD Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Maximum Distance", &maxDistance, 0.0f, 1.5f)) {
            dirty = true;
        }
        if (ImGui::SliderFloat3("Focus Point", &focusPoint.x, -0.4f, 0.4f)) {
            dirty = true;
        }
    }
    ImGui::End();
}
