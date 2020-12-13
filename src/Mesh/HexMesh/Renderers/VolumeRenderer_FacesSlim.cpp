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
#include <Utils/File/Logfile.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "VolumeRenderer_FacesSlim.hpp"

const char* const sortingModeStrings[] = {"Priority Queue", "Bubble Sort", "Insertion Sort", "Shell Sort", "Max Heap"};

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

// Choice of sorting algorithm
static int sortingAlgorithmMode = 0;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNode {
    // RGBA color of the node
    uint color;
    // Depth value of the fragment (in view space)
    float depth;
    // The index of the next node in "nodes" array
    uint next;
};

VolumeRenderer_FacesSlim::VolumeRenderer_FacesSlim(
        SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow),
          EdgeDetectionRenderer(sceneData, true) {
    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine("DEPTH_TYPE_UINT", "");

    sgl::ShaderManager->invalidateShaderCache();
    gatherShader = sgl::ShaderManager->getShaderProgram(
            {"MeshShader_NoShading.Vertex.Attribute", "MeshShader_NoShading.Fragment"});
    reloadResolveShader();
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClear.Vertex", "LinkedListClear.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates)
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    blitRenderData->addGeometryBuffer(geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    clearRenderData->addGeometryBuffer(geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    initializeEdgeDetection();
    onResolutionChanged();
}

VolumeRenderer_FacesSlim::~VolumeRenderer_FacesSlim() {
    sgl::ShaderManager->removePreprocessorDefine("DEPTH_TYPE_UINT");
}

void VolumeRenderer_FacesSlim::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    this->hexMesh = meshIn;
    updateLargeMeshMode();

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<float> vertexAttributes;
    meshIn->getVolumeData_FacesShared_Slim(triangleIndices, vertexPositions, vertexAttributes);

    size_t modelBufferSizeBytes =
            triangleIndices.size() * sizeof(uint32_t)
            + vertexPositions.size() * sizeof(glm::vec3)
            + vertexAttributes.size() * sizeof(float);
    sgl::Logfile::get()->writeInfo(
            std::string() + "GPU model buffer size MiB: "
            + std::to_string(modelBufferSizeBytes / 1024.0 / 1024.0));

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
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

    // Add the color buffer.
    sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), (void*)&vertexAttributes.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            attributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);

    reloadModelEdgeDetection(hexMesh);

    dirty = false;
    reRender = true;
}

void VolumeRenderer_FacesSlim::reallocateFragmentBuffer() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    fragmentBufferSize = size_t(expectedAvgDepthComplexity) * size_t(width) * size_t(height);
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNode) * fragmentBufferSize;
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeBytes = (1ull << 32ull) - sizeof(LinkedListFragmentNode);
        fragmentBufferSize = fragmentBufferSizeBytes / sizeof(LinkedListFragmentNode);
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));
    }

    if (sceneData.performanceMeasurer) {
        sceneData.performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }

    fragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);
}

void VolumeRenderer_FacesSlim::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity));

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4;
        sgl::ShaderManager->addPreprocessorDefine("STACK_SIZE", sgl::toString(stackSize));
    }

    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void VolumeRenderer_FacesSlim::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if (sceneData.performanceMeasurer && !timerDataIsWritten) {
        if (timer) {
            delete timer;
        }
        timer = new sgl::TimerGL;
        sceneData.performanceMeasurer->setClearViewTimer(timer);
    }
}

void VolumeRenderer_FacesSlim::setNewSettings(const SettingsMap& settings) {
    if (settings.hasValue("sortingAlgorithmMode")) {
        sortingAlgorithmMode = (SortingAlgorithmMode)settings.getIntValue("sortingAlgorithmMode");
        setSortingAlgorithmDefine();
        reloadResolveShader();
    }
}

void VolumeRenderer_FacesSlim::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (hexMesh->getNumCells() > 1e6) { // > 1m elements
        newMeshLargeMeshMode = MESH_SIZE_LARGE;
    }
    if (newMeshLargeMeshMode != largeMeshMode) {
        largeMeshMode = newMeshLargeMeshMode;
        expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][0];
        expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][1];
        reallocateFragmentBuffer();
        reloadResolveShader();
    }
}

void VolumeRenderer_FacesSlim::setSortingAlgorithmDefine() {
    if (sortingAlgorithmMode == 0) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "frontToBackPQ");
    } else if (sortingAlgorithmMode == 1) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "bubbleSort");
    } else if (sortingAlgorithmMode == 2) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "insertionSort");
    } else if (sortingAlgorithmMode == 3) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "shellSort");
    } else if (sortingAlgorithmMode == 4) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "heapSort");
    }
}

void VolumeRenderer_FacesSlim::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    windowWidth = width;
    windowHeight = height;

    reallocateFragmentBuffer();

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * width * height;
    startOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);

    reloadTexturesEdgeDetection();
}

void VolumeRenderer_FacesSlim::setUniformData() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();

    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, startOffsetBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShader->setUniform("viewportW", width);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShader->setUniformOptional("cameraPosition", sceneData.camera->getPosition());
    gatherShader->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShader->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    gatherShader->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);

    resolveShader->setUniform("viewportW", width);
    resolveShader->setShaderStorageBuffer(0, "FragmentBuffer", fragmentBuffer);
    resolveShader->setShaderStorageBuffer(1, "StartOffsetBuffer", startOffsetBuffer);

    clearShader->setUniform("viewportW", width);
    clearShader->setShaderStorageBuffer(1, "StartOffsetBuffer", startOffsetBuffer);

    setUniformDataEdgeDetection();
}

void VolumeRenderer_FacesSlim::clear() {
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearRenderData);

    // Set atomic counter to zero.
    GLuint bufferID = static_cast<sgl::GeometryBufferGL*>(atomicCounterBuffer.get())->getBuffer();
    GLubyte val = 0;
    glClearNamedBufferData(bufferID, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void VolumeRenderer_FacesSlim::gather() {
    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);

    // We can use the stencil buffer to mask used pixels for the resolve pass.
    if (useStencilBuffer) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glClear(GL_STENCIL_BUFFER_BIT);
    }

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    glDisable(GL_CULL_FACE);
    sgl::Renderer->render(shaderAttributes);
    glEnable(GL_CULL_FACE);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void VolumeRenderer_FacesSlim::resolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    if (useStencilBuffer) {
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
    }

    sgl::Renderer->render(blitRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    renderEdgeDetectionContours();

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void VolumeRenderer_FacesSlim::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void VolumeRenderer_FacesSlim::renderGui() {
    if (ImGui::Begin("Pseudo Volume Renderer", &showRendererWindow)) {
        if (ImGui::Combo(
                "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
            setSortingAlgorithmDefine();
            reloadResolveShader();
            reRender = true;
        }
        reRender = renderGuiEdgeDetection() || reRender;
    }
    ImGui::End();
}
