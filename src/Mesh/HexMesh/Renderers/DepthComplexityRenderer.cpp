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

#include <iostream>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/Timer.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "DepthComplexityRenderer.hpp"

DepthComplexityRenderer::DepthComplexityRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    gatherShader = sgl::ShaderManager->getShaderProgram(
            {"DepthComplexityGather.Vertex", "DepthComplexityGather.Fragment"});
    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"DepthComplexityResolve.Vertex", "DepthComplexityResolve.Fragment"});
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"DepthComplexityClear.Vertex", "DepthComplexityClear.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates)
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    blitRenderData->addGeometryBuffer(geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    clearRenderData->addGeometryBuffer(geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
}

void DepthComplexityRenderer::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    hexMesh = meshIn;

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    meshIn->getVolumeData_DepthComplexity_Slim(triangleIndices, vertexPositions);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    firstFrame = true;
    totalNumFragments = 0;
    usedLocations = 1;
    maxComplexity = 0;
    bufferSize = 1;
    intensity = 1.5f;

    dirty = false;
    reRender = true;
}

void DepthComplexityRenderer::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    size_t fragmentCounterBufferSizeBytes = sizeof(uint32_t) * width * height;
    fragmentCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentCounterBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentCounterBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);
}

void DepthComplexityRenderer::setUniformData() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();

    gatherShader->setUniform("viewportW", width);
    gatherShader->setShaderStorageBuffer(0, "FragmentCounterBuffer", fragmentCounterBuffer);

    resolveShader->setUniform("viewportW", width);
    resolveShader->setShaderStorageBuffer(0, "FragmentCounterBuffer", fragmentCounterBuffer);

    clearShader->setUniform("viewportW", width);
    clearShader->setShaderStorageBuffer(0, "FragmentCounterBuffer", fragmentCounterBuffer);
}

void DepthComplexityRenderer::clear() {
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearRenderData);

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void DepthComplexityRenderer::gather() {
    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);

    // We can use the stencil buffer to mask used pixels for the resolve pass.
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glStencilMask(0xFF);
    glClear(GL_STENCIL_BUFFER_BIT);
    glDisable(GL_CULL_FACE);

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    sgl::Renderer->render(shaderAttributes);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    glEnable(GL_CULL_FACE);
}

void DepthComplexityRenderer::resolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    glStencilFunc(GL_EQUAL, 1, 0xFF);
    glStencilMask(0x00);

    resolveShader->setUniform("color", renderColor);
    resolveShader->setUniform("numFragmentsMaxColor", numFragmentsMaxColor);
    sgl::Renderer->render(blitRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void DepthComplexityRenderer::render() {
    setUniformData();
    clear();
    gather();
    resolve();

    if (sceneData.performanceMeasurer != nullptr || sceneData.recordingMode) {
        computeStatistics(false);
    }
}

void DepthComplexityRenderer::renderGui() {
    if (ImGui::Begin(getWindowName(), &showWindow)) {
        std::string totalNumFragmentsString = sgl::numberToCommaString(totalNumFragments);
        ImGui::Text("Depth complexity: #fragments: %s", totalNumFragmentsString.c_str());
        ImGui::Text("avg used: %.2f, avg all: %.2f, max: %lu", ((float) totalNumFragments / usedLocations),
                    ((float) totalNumFragments / bufferSize), maxComplexity);

        if (ImGui::ColorEdit4("Coloring", (float*)&colorSelection, 0)) {
            sgl::Color newColor = sgl::colorFromFloat(colorSelection.x, colorSelection.y, colorSelection.z, 1.0f);
            renderColor = newColor;
            intensity = 0.0001f + 3*colorSelection.w;
            numFragmentsMaxColor = std::max(maxComplexity, uint64_t(4ull))/intensity;
            reRender = true;
        }
    }
    ImGui::End();
}

bool DepthComplexityRenderer::needsReRender() {
    // Update & print statistics if enough time has passed
    static float counterPrintFrags = 0.0f;
    counterPrintFrags += sgl::Timer->getElapsedSeconds();
    if (hexMesh && (counterPrintFrags > 1.0f || firstFrame)) {
        computeStatistics(true);
        counterPrintFrags = 0.0f;
        firstFrame = false;
        return true;
    }
    return false;
}

void DepthComplexityRenderer::computeStatistics(bool isReRender) {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();
    bufferSize = width * height;

    uint32_t *data = (uint32_t*)fragmentCounterBuffer->mapBuffer(sgl::BUFFER_MAP_READ_ONLY);

    // Local reduction variables necessary for older OpenMP implementations
    uint64_t totalNumFragments = 0;
    uint64_t usedLocations = 0;
    uint64_t maxComplexity = 0;
    uint64_t minComplexity = 0;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(+:totalNumFragments,usedLocations) reduction(max:maxComplexity) \
    reduction(min:minComplexity) schedule(static)
#endif
    for (int i = 0; i < bufferSize; i++) {
        totalNumFragments += data[i];
        if (data[i] > 0) {
            usedLocations++;
        }
        maxComplexity = std::max(maxComplexity, uint64_t(data[i]));
        minComplexity = std::min(minComplexity, uint64_t(data[i]));
    }
    if (totalNumFragments == 0) usedLocations = 1; // Avoid dividing by zero in code below
    this->totalNumFragments = totalNumFragments;
    this->usedLocations = usedLocations;
    this->maxComplexity = maxComplexity;

    fragmentCounterBuffer->unmapBuffer();

    bool performanceMeasureMode = sceneData.performanceMeasurer != nullptr;
    if ((performanceMeasureMode || sceneData.recordingMode) || firstFrame) {
        if (!isReRender) {
            firstFrame = false;
        }
        numFragmentsMaxColor = std::max(maxComplexity, uint64_t(4ull))/intensity;
    }

    if (performanceMeasureMode) {
        sceneData.performanceMeasurer->pushDepthComplexityFrame(
                minComplexity, maxComplexity,
                (float)totalNumFragments / usedLocations,
                (float)totalNumFragments / bufferSize, totalNumFragments);
    }

    if (totalNumFragments == 0) usedLocations = 1; // Avoid dividing by zero in code below
    std::cout << "Depth complexity: avg used: " << ((float)totalNumFragments / usedLocations)
              << ", avg all: " << ((float)totalNumFragments / bufferSize) << ", max: " << maxComplexity
              << ", #fragments: " << totalNumFragments << std::endl;
}
