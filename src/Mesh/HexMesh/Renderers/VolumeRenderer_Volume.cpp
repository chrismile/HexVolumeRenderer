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
#include <ImGui/ImGuiWrapper.hpp>

#include "Helpers/SortingVendorFix.hpp"
#include "VolumeRenderer_Volume.hpp"

const char* const sortingModeStrings[] = {"Priority Queue", "Bubble Sort", "Insertion Sort", "Shell Sort", "Max Heap"};

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

/// Expected (average) depth complexity, i.e. width*height* this value = number of fragments that can be stored.
static int expectedDepthComplexityVolume = 60; // for front and back faces each
static int expectedDepthComplexitySurface = 40;
/// Maximum number of fragments to sort in second pass.
static int maxNumFragmentsSortingVolume = 128; // for front and back faces each
static int maxNumFragmentsSortingSurface = 32;

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

VolumeRenderer_Volume::VolumeRenderer_Volume(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListVolumeGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine(
            "MAX_NUM_FRAGS_VOLUME", sgl::toString(maxNumFragmentsSortingVolume));
    sgl::ShaderManager->addPreprocessorDefine(
            "MAX_NUM_FRAGS_SURFACE", sgl::toString(maxNumFragmentsSortingSurface));

    gatherShaderVolumeFrontFaces = sgl::ShaderManager->getShaderProgram(
            {"MeshLinkedListVolume.Vertex.Attribute", "MeshLinkedListVolume.Fragment.FrontFace"});
    gatherShaderVolumeBackFaces = sgl::ShaderManager->getShaderProgram(
            {"MeshLinkedListVolume.Vertex.Attribute", "MeshLinkedListVolume.Fragment.BackFace"});
    //gatherShaderSurface = sgl::ShaderManager->getShaderProgram(
    //        {"MeshShader.Vertex", "MeshShader.Fragment.Volume"});
    resolveShader = sgl::ShaderManager->getShaderProgram({
        "LinkedListVolumeResolve.Vertex", "LinkedListVolumeResolve.Fragment"});
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListVolumeClear.Vertex", "LinkedListVolumeClear.Fragment"});

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

void VolumeRenderer_Volume::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    this->hexMesh = meshIn;

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<float> vertexAttributes;
    if (useWeightedVertexAttributes) {
        meshIn->getVolumeData_VolumeShared(triangleIndices, vertexPositions, vertexAttributes);
    } else {
        meshIn->getVolumeData_Volume(triangleIndices, vertexPositions, vertexAttributes);
    }

    shaderAttributesVolumeFrontFaces = sgl::ShaderManager->createShaderAttributes(gatherShaderVolumeFrontFaces);
    shaderAttributesVolumeFrontFaces->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributesVolumeFrontFaces->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributesVolumeFrontFaces->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);
    shaderAttributesVolumeFrontFaces->addGeometryBuffer(
            attributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);

    // Use the same data for the back faces, but with a different shader.
    shaderAttributesVolumeBackFaces = shaderAttributesVolumeFrontFaces->copy(gatherShaderVolumeBackFaces);

    dirty = false;
    reRender = true;
}

void VolumeRenderer_Volume::setSortingAlgorithmDefine() {
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

void VolumeRenderer_Volume::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    size_t fragmentBufferSizeVolume = expectedDepthComplexityVolume * width * height;
    size_t fragmentBufferSizeBytesVolume = sizeof(LinkedListFragmentNode) * fragmentBufferSizeVolume;
    size_t fragmentBufferSizeSurface = expectedDepthComplexitySurface * width * height;
    size_t fragmentBufferSizeBytesSurface = sizeof(LinkedListFragmentNode) * fragmentBufferSizeSurface;
    std::cout << "Buffer size GiB: 2 * "
            << (fragmentBufferSizeBytesVolume / 1024.0 / 1024.0 / 1024.0) << " + "
            << (fragmentBufferSizeBytesSurface / 1024.0 / 1024.0 / 1024.0) << " = "
            << ((2 * fragmentBufferSizeBytesVolume + fragmentBufferSizeBytesSurface) / 1024.0 / 1024.0 / 1024.0)
            << std::endl;

    // Delete old data first (-> refcount 0)
    fragmentBufferVolumeFrontFaces = sgl::GeometryBufferPtr();
    fragmentBufferVolumeBackFaces = sgl::GeometryBufferPtr();
    fragmentBufferSurface = sgl::GeometryBufferPtr();
    fragmentBufferVolumeFrontFaces = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytesVolume, NULL, sgl::SHADER_STORAGE_BUFFER);
    fragmentBufferVolumeBackFaces = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytesVolume, NULL, sgl::SHADER_STORAGE_BUFFER);
    fragmentBufferSurface = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytesSurface, NULL, sgl::SHADER_STORAGE_BUFFER);

    // Delete old data first (-> refcount 0)
    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * width * height;
    startOffsetVolumeFrontFaces = sgl::GeometryBufferPtr();
    startOffsetVolumeBackFaces = sgl::GeometryBufferPtr();
    startOffsetSurface = sgl::GeometryBufferPtr();
    startOffsetVolumeFrontFaces = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);
    startOffsetVolumeBackFaces = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);
    startOffsetSurface = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    // Delete old data first (-> refcount 0)
    fragCounterVolumeFrontFaces = sgl::GeometryBufferPtr();
    fragCounterVolumeBackFaces = sgl::GeometryBufferPtr();
    fragCounterSurface = sgl::GeometryBufferPtr();
    fragCounterVolumeFrontFaces = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);
    fragCounterVolumeBackFaces = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);
    fragCounterSurface = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);
}

void VolumeRenderer_Volume::setUniformData() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    size_t fragmentBufferSizeVolume = expectedDepthComplexityVolume * width * height;
    size_t fragmentBufferSizeSurface = expectedDepthComplexitySurface * width * height;

    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBufferVolumeFrontFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(1, fragmentBufferVolumeBackFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(2, fragmentBufferSurface);
    sgl::ShaderManager->bindShaderStorageBuffer(3, startOffsetVolumeFrontFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(4, startOffsetVolumeBackFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(5, startOffsetSurface);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, fragCounterVolumeFrontFaces);
    sgl::ShaderManager->bindAtomicCounterBuffer(1, fragCounterVolumeBackFaces);
    sgl::ShaderManager->bindAtomicCounterBuffer(2, fragCounterSurface);

    sgl::ShaderProgramPtr gatherShaders[] = {
            gatherShaderVolumeFrontFaces, gatherShaderVolumeBackFaces/*, gatherShaderSurface*/
    };
    for (sgl::ShaderProgramPtr& gatherShader : gatherShaders) {
        gatherShader->setUniform("viewportW", width);
        gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    gatherShaderVolumeFrontFaces->setUniform(
            "linkedListVolumeFrontFacesCapacity", (unsigned int)fragmentBufferSizeVolume);
    gatherShaderVolumeBackFaces->setUniform(
            "linkedListVolumeBackFacesCapacity", (unsigned int)fragmentBufferSizeVolume);
    gatherShaderVolumeFrontFaces->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShaderVolumeFrontFaces->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    gatherShaderVolumeFrontFaces->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    gatherShaderVolumeBackFaces->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShaderVolumeBackFaces->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    gatherShaderVolumeBackFaces->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    /*gatherShaderSurface->setUniform(
            "linkedListSurfaceCapacity", (unsigned int)fragmentBufferSizeSurface);*/

    resolveShader->setUniform("viewportW", width);
    clearShader->setUniform("viewportW", width);
}

void VolumeRenderer_Volume::clear() {
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearRenderData);

    // Set atomic counters to zero.
    GLubyte val = 0;
    glClearNamedBufferData(
            static_cast<sgl::GeometryBufferGL*>(fragCounterVolumeFrontFaces.get())->getBuffer(),
            GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glClearNamedBufferData(
            static_cast<sgl::GeometryBufferGL*>(fragCounterVolumeBackFaces.get())->getBuffer(),
            GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glClearNamedBufferData(
            static_cast<sgl::GeometryBufferGL*>(fragCounterSurface.get())->getBuffer(),
            GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void VolumeRenderer_Volume::gather() {
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
    sgl::Renderer->render(shaderAttributesVolumeFrontFaces);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    glCullFace(GL_FRONT);
    sgl::Renderer->render(shaderAttributesVolumeBackFaces);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    glCullFace(GL_BACK);
}

void VolumeRenderer_Volume::resolve() {
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

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void VolumeRenderer_Volume::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void VolumeRenderer_Volume::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::Checkbox("Use Weighted Vertex Attributes", &useWeightedVertexAttributes)) {
            if (this->hexMesh) uploadVisualizationMapping(hexMesh, false);
        }
    }
    ImGui::End();
}
