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
#include <Graphics/OpenGL/RendererGL.hpp>

#include "SurfaceRenderer.hpp"

const glm::vec4 hullColor = glm::vec4(
        sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(0.5, 0.5, 0.5f)), 0.3f);

SurfaceRenderer::SurfaceRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Attribute", "MeshShader.Fragment"});
    shaderProgramHull = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain.PositiveDepthBias"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");
}

void SurfaceRenderer::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    meshIn->getSurfaceData(triangleIndices, vertexPositions, vertexNormals, vertexAttributes);

    shaderAttributesSurface = sgl::ShaderManager->createShaderAttributes(shaderProgramSurface);
    shaderAttributesSurface->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributesSurface->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSurface->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSurface->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSurface->addGeometryBuffer(
            attributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);



    // Get hull data.
    std::vector<uint32_t> triangleIndicesHull;
    std::vector<glm::vec3> vertexPositionsHull;
    std::vector<glm::vec3> vertexNormalsHull;
    std::vector<float> vertexAttributesHull;
    meshIn->getSurfaceData(
            triangleIndicesHull, vertexPositionsHull,
            vertexNormalsHull, vertexAttributesHull,
            false);

    shaderAttributesHull = sgl::ShaderManager->createShaderAttributes(shaderProgramHull);
    shaderAttributesHull->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBufferHull = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndicesHull.size(), triangleIndicesHull.data(), sgl::INDEX_BUFFER);
    shaderAttributesHull->setIndexGeometryBuffer(indexBufferHull, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBufferHull = sgl::Renderer->createGeometryBuffer(
            vertexPositionsHull.size()*sizeof(glm::vec3), vertexPositionsHull.data(), sgl::VERTEX_BUFFER);
    shaderAttributesHull->addGeometryBuffer(
            positionBufferHull, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBufferHull = sgl::Renderer->createGeometryBuffer(
            vertexNormalsHull.size()*sizeof(glm::vec3), vertexNormalsHull.data(), sgl::VERTEX_BUFFER);
    shaderAttributesHull->addGeometryBuffer(
            normalBufferHull, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    dirty = false;
    reRender = true;
}

void SurfaceRenderer::render() {
    shaderProgramSurface->setUniform("useShading", int(useShading));
    shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgramSurface->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    shaderProgramSurface->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    shaderProgramSurface->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    sgl::Renderer->render(shaderAttributesSurface);

    if (hullOpacity > 0.0f) {
        shaderProgramHull->setUniform("useShading", int(useShading));
        shaderProgramHull->setUniform("cameraPosition", sceneData.camera->getPosition());
        shaderProgramHull->setUniform("color", glm::vec4(hullColor.r, hullColor.g, hullColor.b, hullOpacity));
        glDepthMask(GL_FALSE);
        sgl::Renderer->render(shaderAttributesHull);
        glDepthMask(GL_TRUE);
    }
}

void SurfaceRenderer::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(0, 0, 540, 160);
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::Checkbox("Use Shading", &useShading)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Hull Opacity", &hullOpacity, 0.0f, 0.5f, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();
}
