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
#include <Graphics/OpenGL/SystemGL.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "Helpers/SortingVendorFix.hpp"
#include "VolumeRenderer_FacesSlim.hpp"

const char* const sortingModeStrings[] = {"Priority Queue", "Bubble Sort", "Insertion Sort", "Shell Sort", "Max Heap"};

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

// Choice of sorting algorithm
static int sortingAlgorithmMode = 0;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNode {
    // RGBA color of the node
    uint32_t color;
    // Depth value of the fragment (in view space)
    float depth;
    // The index of the next node in "nodes" array
    uint32_t next;
};

VolumeRenderer_FacesSlim::VolumeRenderer_FacesSlim(
        SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow),
          EdgeDetectionRenderer(sceneData, true) {
    /*
     * In Vulkan, we could use the minimum out of physicalDeviceVulkan11Properties.maxMemoryAllocationSize and
     * physicalDeviceProperties.limits.maxStorageBufferRange. On NVIDIA hardware, this seems to be 4GiB - 1B, on AMD
     * hardware it seems to be 2GiB. We will just assume OpenGL allows allocations of size 4GiB.
     */
    maxStorageBufferSize = (1ull << 32ull) - 1ull;
    double availableMemoryFactor = 28.0 / 32.0;
    maxDeviceMemoryBudget = size_t(double(sgl::SystemGL::get()->getFreeMemoryBytes()) * availableMemoryFactor);
    // We only have binding {1,2,3,4,5}, so limit the maximum budget.
    maxDeviceMemoryBudget = std::min(maxDeviceMemoryBudget, maxStorageBufferSize * 5);

    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine("DEPTH_TYPE_UINT", "");

    sgl::ShaderManager->invalidateShaderCache();
    reloadGatherShader(false);
    reloadResolveShader();
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClear.Vertex", "LinkedListClear.Fragment"});

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

    initializeEdgeDetection();
    onResolutionChanged();
}

VolumeRenderer_FacesSlim::~VolumeRenderer_FacesSlim() {
    sgl::ShaderManager->removePreprocessorDefine("DEPTH_TYPE_UINT");
    if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
        sgl::ShaderManager->removePreprocessorDefine("FRAGMENT_BUFFER_ARRAY");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGMENT_BUFFERS");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGS_PER_BUFFER");
    }
}

void VolumeRenderer_FacesSlim::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    this->hexMesh = meshIn;
    updateLargeMeshMode();

    if (isNewMesh) {
        multiVarAttrIdx = 0;
    }

    if ((meshIn->hasMultiVarData() && useMultiVarData && !isMultiVarData)
            || (!meshIn->hasMultiVarData() && useMultiVarData && isMultiVarData)) {
        isMultiVarData = meshIn->hasMultiVarData();
        reloadGatherShader(false);
    } else {
        isMultiVarData = meshIn->hasMultiVarData();
    }

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<float> vertexAttributes;
    meshIn->getVolumeData_FacesShared_Slim(triangleIndices, vertexPositions, vertexAttributes);

    size_t modelBufferSizeBytes =
            triangleIndices.size() * sizeof(uint32_t)
            + vertexPositions.size() * sizeof(glm::vec3)
            + vertexAttributes.size() * sizeof(float);
    if (useMultiVarData && isMultiVarData) {
        modelBufferSizeBytes += vertexAttributes.size() * sizeof(float);
    }
    sgl::Logfile::get()->writeInfo(
            std::string() + "GPU model buffer size MiB: "
            + std::to_string(modelBufferSizeBytes / 1024.0 / 1024.0));
    sgl::Logfile::get()->writeInfo("Number of triangles: " + std::to_string(triangleIndices.size() / 3));

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

    // Add the color buffer.
    if (!useMultiVarData || !isMultiVarData) {
        sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
                vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);
        shaderAttributes->addGeometryBuffer(
                attributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);
    } else {
        std::vector<float> interpolatedCellAttributes = hexMesh->getInterpolatedCellAttributeVertexData();
        std::vector<float> manualVertexAttributes = hexMesh->getManualVertexAttributeDataNormalized(multiVarAttrIdx);
        sgl::GeometryBufferPtr attributeBuffer0 = sgl::Renderer->createGeometryBuffer(
                interpolatedCellAttributes.size()*sizeof(float), interpolatedCellAttributes.data(),
                sgl::VERTEX_BUFFER);
        sgl::GeometryBufferPtr attributeBuffer1 = sgl::Renderer->createGeometryBuffer(
                manualVertexAttributes.size()*sizeof(float), manualVertexAttributes.data(),
                sgl::VERTEX_BUFFER);
        shaderAttributes->addGeometryBuffer(
                attributeBuffer0, "vertexAttribute0", sgl::ATTRIB_FLOAT, 1);
        shaderAttributes->addGeometryBuffer(
                attributeBuffer1, "vertexAttribute1", sgl::ATTRIB_FLOAT, 1);
    }

    reloadModelEdgeDetection(hexMesh);

    dirty = false;
    reRender = true;
}

void VolumeRenderer_FacesSlim::reallocateFragmentBuffer() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    fragmentBufferSize = size_t(expectedAvgDepthComplexity) * size_t(width) * size_t(height);
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNode) * fragmentBufferSize;

    // Delete old data first (-> refcount 0)
    fragmentBuffers = {};
    fragmentBuffer = {};

    // We only need buffer arrays when the maximum allocation is larger than our budget.
    bool reloadShaders = false;
    if (maxDeviceMemoryBudget < maxStorageBufferSize) {
        fragmentBufferMode = FragmentBufferMode::BUFFER;
        reloadShaders = true;
    }
    size_t maxSingleBufferAllocation = std::min(maxDeviceMemoryBudget, maxStorageBufferSize);

    // https://www.khronos.org/registry/OpenGL/extensions/NVX/NVX_gpu_memory_info.txt
    if (sgl::SystemGL::get()->isGLExtensionAvailable("GL_NVX_gpu_memory_info")) {
        GLint memKilobytes = 0;
        glGetIntegerv(GL_GPU_MEMORY_INFO_DEDICATED_VIDMEM_NVX, &memKilobytes);
        double dedicatedMemGiB = memKilobytes * 1000.0 / 1024.0 / 1024.0 / 1024.0;
        double numMillionCells = hexMesh ? hexMesh->getNumCells() * 1e-6 : 0.0;
        std::cout << "Dedicated mem: " << dedicatedMemGiB << " GiB" << std::endl;
        std::cout << "Cells: " << numMillionCells << "M" << std::endl;
        if (dedicatedMemGiB < 9.0 && numMillionCells > 30.0) {
            if (fragmentBufferSizeBytes / (1024.0 * 1024.0 * 1024.0) >= 2.6) {
                sgl::Logfile::get()->writeError(
                        std::string() + "Clamped memory to 2.6 GiB due to dedicated VRAM limitations.");
                fragmentBufferSizeBytes = size_t(2.6 * 1024.0 * 1024.0 * 1024.0);
                fragmentBufferSize = fragmentBufferSizeBytes / sizeof(LinkedListFragmentNode);
            }
        }
    }

    if (fragmentBufferMode == FragmentBufferMode::BUFFER) {
        if (fragmentBufferSizeBytes > maxSingleBufferAllocation) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Fragment buffer size was larger than maximum allocation size ("
                    + std::to_string(maxSingleBufferAllocation) + "). Clamping to maximum allocation size.",
                    false);
            fragmentBufferSize = maxSingleBufferAllocation / sizeof(LinkedListFragmentNode);
            fragmentBufferSizeBytes = fragmentBufferSize * sizeof(LinkedListFragmentNode);
        } else {
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Fragment buffer size GiB: "
                    + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
        }

        numFragmentBuffers = 1;
        cachedNumFragmentBuffers = 1;
        fragmentBuffer = sgl::Renderer->createGeometryBuffer(
                fragmentBufferSizeBytes, nullptr, sgl::SHADER_STORAGE_BUFFER);
    } else {
        if (fragmentBufferSizeBytes > maxDeviceMemoryBudget) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Fragment buffer size was larger than maximum allocation size ("
                    + std::to_string(maxDeviceMemoryBudget) + "). Clamping to maximum allocation size.",
                    false);
            fragmentBufferSize = maxDeviceMemoryBudget / 12ull;
            fragmentBufferSizeBytes = fragmentBufferSize * 12ull;
        } else {
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Fragment buffer size GiB: "
                    + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
        }

        numFragmentBuffers = sgl::sizeceil(fragmentBufferSizeBytes, maxStorageBufferSize);
        size_t fragmentBufferSizeBytesLeft = fragmentBufferSizeBytes;
        for (size_t i = 0; i < numFragmentBuffers; i++) {
            fragmentBuffers.emplace_back(sgl::Renderer->createGeometryBuffer(
                    std::min(fragmentBufferSizeBytesLeft, maxStorageBufferSize), nullptr, sgl::SHADER_STORAGE_BUFFER));
            fragmentBufferSizeBytesLeft -= maxStorageBufferSize;
        }

        if (numFragmentBuffers != cachedNumFragmentBuffers) {
            cachedNumFragmentBuffers = numFragmentBuffers;
            reloadShaders = true;
        }
    }

    if (fragmentBufferMode == FragmentBufferMode::BUFFER) {
        sgl::ShaderManager->removePreprocessorDefine("FRAGMENT_BUFFER_ARRAY");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGMENT_BUFFERS");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGS_PER_BUFFER");
    } else {
        sgl::ShaderManager->addPreprocessorDefine("FRAGMENT_BUFFER_ARRAY", "");
        sgl::ShaderManager->addPreprocessorDefine(
                "NUM_FRAGMENT_BUFFERS", std::to_string(cachedNumFragmentBuffers));
        sgl::ShaderManager->addPreprocessorDefine(
                "NUM_FRAGS_PER_BUFFER", std::to_string(maxStorageBufferSize / 12ull) + "u");
    }

    // We only need buffer arrays when the maximum allocation is larger than our budget.
    if (reloadShaders) {
        reloadGatherShader(true);
        reloadResolveShader();
    }

    if (sceneData.performanceMeasurer) {
        sceneData.performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }
}

void VolumeRenderer_FacesSlim::reloadGatherShader(bool copyShaderAttributes) {
    sgl::ShaderManager->invalidateShaderCache();
    if (!isMultiVarData || !useMultiVarData) {
        gatherShader = sgl::ShaderManager->getShaderProgram(
                {"MeshShader_NoShading.Vertex.Attribute", "MeshShader_NoShading.Fragment"});
    } else {
        gatherShader = sgl::ShaderManager->getShaderProgram(
                {"MeshShader_MultiVar.Vertex.Attribute", "MeshShader_MultiVar.Fragment"});
    }

    if (copyShaderAttributes && shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
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
    if (hexMesh->getNumCells() > 1e7) { // > 10m elements
        newMeshLargeMeshMode = MESH_SIZE_VERY_LARGE;
    } else if (hexMesh->getNumCells() > 1e6) { // > 1m elements
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
        if (getIsGpuVendorAmd()) {
            sgl::ShaderManager->addPreprocessorDefine("INITIALIZE_ARRAY_POW2", "");
        }
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
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();
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
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    int width = (*sceneData.sceneTexture)->getW();

    sgl::ShaderManager->bindShaderStorageBuffer(0, startOffsetBuffer);
    if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
        for (size_t i = 0; i < numFragmentBuffers; i++) {
            sgl::ShaderManager->bindShaderStorageBuffer(int(i + 1), fragmentBuffers.at(i));
        }
    } else {
        sgl::ShaderManager->bindShaderStorageBuffer(1, fragmentBuffer);
    }
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShader->setUniform("viewportW", width);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShader->setUniformOptional("cameraPosition", sceneData.camera->getPosition());
    if (!useMultiVarData || !isMultiVarData) {
        gatherShader->setUniform(
                "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
        gatherShader->setUniform(
                "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
        gatherShader->setUniform(
                "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    }

    resolveShader->setUniform("viewportW", width);
    clearShader->setUniform("viewportW", width);

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
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(5, 72, 586, 346);
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (isMultiVarData && ImGui::Checkbox("Use Multi-Var Data", &useMultiVarData)) {
            reloadGatherShader(false);
            uploadVisualizationMapping(hexMesh, false);
            reRender = true;
        }
        if (isMultiVarData && useMultiVarData && hexMesh) {
            std::vector<std::string> manualVertexAttributesNames = hexMesh->getManualVertexAttributesNames();
            if (ImGui::Combo(
                    "Attribute #2", &multiVarAttrIdx, manualVertexAttributesNames.data(),
                    manualVertexAttributesNames.size())) {
                uploadVisualizationMapping(hexMesh, false);
                reRender = true;
            }
        }
        if (ImGui::Combo(
                "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
            setSortingAlgorithmDefine();
            reloadResolveShader();
            reRender = true;
        }
        if (ImGui::Combo(
                "Fragment Buffer Mode", (int*)&fragmentBufferMode,
                FRAGMENT_BUFFER_MODE_NAMES, IM_ARRAYSIZE(FRAGMENT_BUFFER_MODE_NAMES))) {
            reallocateFragmentBuffer();
            reloadResolveShader();
            reloadGatherShader(true);
            reRender = true;
        }
        if (ImGui::Button("Reload Gather Shader")) {
            reloadGatherShader(true);
            reRender = true;
        }
        reRender = renderGuiEdgeDetection() || reRender;
    }
    ImGui::End();
}
