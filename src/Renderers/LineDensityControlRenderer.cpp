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
#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Helpers/LineRenderingDefines.hpp"
#include "LOD/LodSheetGeneration.hpp"
#include "LineDensityControlRenderer.hpp"

/// Expected (average) depth complexity, i.e. width*height* this value = number of fragments that can be stored.
static int EXPECTED_DEPTH_COMPLEXITY = 90;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNodeAttributeTextures {
    float importanceAttribute; ///< Between 0 and 1.
    float depth; ///< The linear depth of the fragment (i.e., distance to the camera).
    uint directionQuantized; ///< First 16 bits encode x, second 16 bits encode y.
    uint next; ///< Next entry index in the per-pixel linked list (or -1 == 0xFFFFFFFFu).
};

LineDensityControlRenderer::LineDensityControlRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    lineDensityControlShader = sgl::ShaderManager->getShaderProgram(
            {"WireframeLineDensityControl.Vertex", "WireframeLineDensityControl.Fragment"});

    createAttributeTextureClearShader = sgl::ShaderManager->getShaderProgram(
            {"CreateAttributeTextureClear.Vertex", "CreateAttributeTextureClear.Fragment"});
    createAttributeTextureGatherShader = sgl::ShaderManager->getShaderProgram(
            {"CreateAttributeTextureResolve.Vertex", "CreateAttributeTextureResolve.Fragment"});
    createAttributeTextureResolveShader = sgl::ShaderManager->getShaderProgram(
            {"CreateAttributeTextureResolve.Vertex", "CreateAttributeTextureResolve.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates)
    createAttributeTextureResolveRenderData =
            sgl::ShaderManager->createShaderAttributes(createAttributeTextureResolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    createAttributeTextureResolveRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    createAttributeTextureClearRenderData =
            sgl::ShaderManager->createShaderAttributes(createAttributeTextureClearShader);
    geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    createAttributeTextureClearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
}

void LineDensityControlRenderer::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    fragmentBufferSize = EXPECTED_DEPTH_COMPLEXITY * width * height;
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNodeAttributeTextures) * fragmentBufferSize;
    if (fragmentBufferSize >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeBytes = (1ull << 32ull) - sizeof(LinkedListFragmentNodeAttributeTextures);
        fragmentBufferSize = fragmentBufferSizeBytes / sizeof(LinkedListFragmentNodeAttributeTextures);
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));
    }

    createAttributeTextureFragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    createAttributeTextureFragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * width * height;
    createAttributeTextureStartOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    createAttributeTextureStartOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    createAttributeTextureAtomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    createAttributeTextureAtomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);
}

void LineDensityControlRenderer::generateVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    /*std::vector<uint32_t> indices;
    std::vector<LodHexahedralCellFace> hexahedralCellFaces;
    generateSheetLevelOfDetailLineStructureAndVertexData(
            meshIn.get(), indices, hexahedralCellFaces);

    lineShaderAttributes = sgl::ShaderManager->createShaderAttributes(lineShaderProgram);
    lineShaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
    lineShaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(LodHexahedralCellFace), (void*)&hexahedralCellFaces.front(),
            sgl::SHADER_STORAGE_BUFFER);

    dirty = false;
    reRender = true;*/
}

void LineDensityControlRenderer::setUniformData() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    sgl::ShaderManager->bindShaderStorageBuffer(0, createAttributeTextureAtomicCounterBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, createAttributeTextureStartOffsetBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, createAttributeTextureAtomicCounterBuffer);

    createAttributeTextureGatherShader->setUniform("viewportW", width);
    createAttributeTextureGatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    createAttributeTextureGatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    createAttributeTextureResolveShader->setUniform("viewportW", width);
    createAttributeTextureClearShader->setUniform("viewportW", width);
}

void LineDensityControlRenderer::clear() {
    // In the clear and gather pass, we just want to write data to an SSBO.
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(createAttributeTextureClearRenderData);

    // Set atomic counter to zero.
    GLuint bufferID = static_cast<sgl::GeometryBufferGL*>(createAttributeTextureAtomicCounterBuffer.get())->getBuffer();
    GLubyte val = 0;
    glClearNamedBufferData(bufferID, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void LineDensityControlRenderer::gather() {
    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    sgl::Renderer->render(createAttributeTextureGatherRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void LineDensityControlRenderer::resolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    sgl::Renderer->render(createAttributeTextureResolveRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void LineDensityControlRenderer::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void LineDensityControlRenderer::renderGui() {
    if (ImGui::Begin("Line Density Control Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"\u03BB", &lambda, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"m", &factor_m, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"c", &factor_c, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"v", &factor_v, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"d", &factor_d, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();
}
