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

#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "SurfaceRenderer.hpp"

SurfaceRenderer::SurfaceRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    shaderProgram = sgl::ShaderManager->getShaderProgram({"MeshShader.Vertex.Attribute", "MeshShader.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");
}

void SurfaceRenderer::generateVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    meshIn->getSurfaceData(triangleIndices, vertexPositions, vertexNormals, vertexAttributes);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), (void*)&triangleIndices.front(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), (void*)&vertexNormals.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), (void*)&vertexAttributes.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            attributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);

    dirty = false;
    reRender = true;
}

void SurfaceRenderer::render() {
    shaderProgram->setUniform("useShading", int(useShading));
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    sgl::Renderer->render(shaderAttributes);
}

void SurfaceRenderer::renderGui() {
    if (ImGui::Begin("Volume Renderer (Faces)", &showRendererWindow)) {
        if (ImGui::Checkbox("Use Shading", &useShading)) {
            reRender = true;
        }
    }
    ImGui::End();
}
