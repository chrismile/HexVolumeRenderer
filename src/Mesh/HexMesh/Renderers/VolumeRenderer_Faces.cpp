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
#include "VolumeRenderer_Faces.hpp"

const char* const sortingModeStrings[] = {"Priority Queue", "Bubble Sort", "Insertion Sort", "Shell Sort", "Max Heap"};

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

/// Expected (average) depth complexity, i.e. width*height* this value = number of fragments that can be stored.
static int expectedDepthComplexity = 80;
/// Maximum number of fragments to sort in second pass.
static int maxNumFragmentsSorting = 256;

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

const glm::vec4 hullColor = glm::vec4(
        sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(0.5, 0.5, 0.5f)), 0.3f);

VolumeRenderer_Faces::VolumeRenderer_Faces(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(maxNumFragmentsSorting));

    sgl::ShaderManager->invalidateShaderCache();
    gatherShader = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Attribute", "MeshShader.Fragment"});
    gatherShaderHull = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain.PositiveDepthBias"});
    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"});
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

    onResolutionChanged();
}

void VolumeRenderer_Faces::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    this->hexMesh = meshIn;
    
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    if (useWeightedVertexAttributes) {
        meshIn->getVolumeData_FacesShared(triangleIndices, vertexPositions, vertexAttributes);
        // Just fill with dummy data for now
        vertexNormals.resize(vertexPositions.size(), glm::vec3(1.0f));
    } else {
        meshIn->getVolumeData_Faces(triangleIndices, vertexPositions, vertexNormals, vertexAttributes);
    }

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

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
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

    shaderAttributesHull = sgl::ShaderManager->createShaderAttributes(gatherShaderHull);
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

void VolumeRenderer_Faces::setSortingAlgorithmDefine() {
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

void VolumeRenderer_Faces::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    size_t fragmentBufferSize = expectedDepthComplexity * width * height;
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNode) * fragmentBufferSize;
    std::cout << "Fragment buffer size GiB: " << (fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0) << std::endl;

    fragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * width * height;
    startOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);
}

void VolumeRenderer_Faces::setUniformData() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    size_t fragmentBufferSize = expectedDepthComplexity * width * height;

    gatherShader->setUniform("useShading", int(useShading));
    gatherShader->setUniform("viewportW", width);
    gatherShader->setShaderStorageBuffer(0, "StartOffsetBuffer", startOffsetBuffer);
    gatherShader->setShaderStorageBuffer(1, "FragmentBuffer", fragmentBuffer);
    gatherShader->setAtomicCounterBuffer(0, atomicCounterBuffer);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherShader->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShader->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    gatherShader->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);

    resolveShader->setUniform("viewportW", width);
    resolveShader->setShaderStorageBuffer(0, "StartOffsetBuffer", startOffsetBuffer);
    resolveShader->setShaderStorageBuffer(1, "FragmentBuffer", fragmentBuffer);

    clearShader->setUniform("viewportW", width);
    clearShader->setShaderStorageBuffer(0, "StartOffsetBuffer", startOffsetBuffer);

    if (hullOpacity > 0.0f) {
        gatherShaderHull->setUniform("useShading", int(useShading));
        gatherShaderHull->setUniform("viewportW", width);
        gatherShaderHull->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
        gatherShaderHull->setUniform("cameraPosition", sceneData.camera->getPosition());
        gatherShaderHull->setUniform("color", glm::vec4(hullColor.r, hullColor.g, hullColor.b, hullOpacity));
    }
}

void VolumeRenderer_Faces::clear() {
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

void VolumeRenderer_Faces::gather() {
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
    sgl::Renderer->render(shaderAttributes);
    if (hullOpacity > 0.0f) {
        sgl::Renderer->render(shaderAttributesHull);
    }
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void VolumeRenderer_Faces::resolve() {
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

void VolumeRenderer_Faces::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void VolumeRenderer_Faces::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (!useWeightedVertexAttributes && ImGui::Checkbox("Use Shading", &useShading)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Hull Opacity", &hullOpacity, 0.0f, 0.5f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::Checkbox("Use Weighted Vertex Attributes", &useWeightedVertexAttributes)) {
            useShading = false;
            if (this->hexMesh) uploadVisualizationMapping(hexMesh, false);
        }
    }
    ImGui::End();
}
