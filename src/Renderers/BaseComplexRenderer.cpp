/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2019, Christoph Neuhauser
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

#include <Graphics/Renderer.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "BaseComplexRenderer.hpp"

BaseComplexRenderer::BaseComplexRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    shaderProgram = sgl::ShaderManager->getShaderProgram({"Wireframe.Vertex", "Wireframe.Geometry", "Wireframe.Fragment"});
    shaderProgramPoints = sgl::ShaderManager->getShaderProgram({"Point.Vertex", "Point.Geometry", "Point.Fragment"});
}

void BaseComplexRenderer::generateVisualizationMapping(HexMeshPtr meshIn) {
    std::vector<glm::vec3> lineVertices, pointVertices;
    std::vector<glm::vec4> lineColors, pointColors;
    meshIn->getBaseComplexDataWireframe(lineVertices, lineColors, pointVertices, pointColors);

    lineShaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    lineShaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);
    pointShaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgramPoints);
    pointShaderAttributes->setVertexMode(sgl::VERTEX_MODE_POINTS);

    // Add the position buffer.
    sgl::GeometryBufferPtr linePositionBuffer = sgl::Renderer->createGeometryBuffer(
            lineVertices.size()*sizeof(glm::vec3), (void*)&lineVertices.front(), sgl::VERTEX_BUFFER);
    lineShaderAttributes->addGeometryBuffer(
            linePositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    sgl::GeometryBufferPtr pointPositionBuffer = sgl::Renderer->createGeometryBuffer(
            pointVertices.size()*sizeof(glm::vec3), (void*)&pointVertices.front(), sgl::VERTEX_BUFFER);
    pointShaderAttributes->addGeometryBuffer(
            pointPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr lineColorBuffer = sgl::Renderer->createGeometryBuffer(
            lineColors.size()*sizeof(glm::vec4), (void*)&lineColors.front(), sgl::VERTEX_BUFFER);
    lineShaderAttributes->addGeometryBuffer(
            lineColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);
    sgl::GeometryBufferPtr pointColorBuffer = sgl::Renderer->createGeometryBuffer(
            pointColors.size()*sizeof(glm::vec4), (void*)&pointColors.front(), sgl::VERTEX_BUFFER);
    pointShaderAttributes->addGeometryBuffer(
            pointColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);

    dirty = false;
    reRender = true;
}

void BaseComplexRenderer::render() {
    if (shaderProgram->hasUniform("cameraPosition")) {
        shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    if (shaderProgramPoints->hasUniform("cameraPosition")) {
        shaderProgramPoints->setUniform("cameraPosition", sceneData.camera->getPosition());
    }

    glDepthMask(GL_TRUE);
    shaderProgram->setUniform("lineWidth", 0.0015f);
    sgl::Renderer->render(lineShaderAttributes);
    glDepthMask(GL_FALSE);
    shaderProgramPoints->setUniform("radius", 0.002f);
    sgl::Renderer->render(pointShaderAttributes);
    glDepthMask(GL_TRUE);
}

void BaseComplexRenderer::renderGui() {
    ;
}