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
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Mesh/HexMesh/Renderers/Tubes/Tubes.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/Sphere.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "ClearViewRenderer_Faces.hpp"

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

/// Expected (average) depth complexity, i.e. width*height* this value = number of fragments that can be stored.
static int expectedDepthComplexity = 100;
/// Maximum number of fragments to sort in second pass.
static int maxNumFragmentsSorting = 256;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNode {
    // RGBA color of the node.
    uint32_t color;
    // Depth value of the fragment (in view space).
    float depth;
    // The index of the next node in the "nodes" array.
    uint32_t next;
};

ClearViewRenderer_Faces::ClearViewRenderer_Faces(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : ClearViewRenderer(sceneData, transferFunctionWindow) {
    windowName = getWindowName();
    clearViewRendererType = CLEAR_VIEW_RENDERER_TYPE_FACES;

    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(maxNumFragmentsSorting));

    loadClearViewBaseData();

    gatherShaderContext = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Attribute", "MeshShader.Fragment.ClearView.Context"});
    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"});
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClear.Vertex", "LinkedListClear.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    blitRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    clearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
}

void ClearViewRenderer_Faces::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    if (isNewMesh) {
        Pickable::focusPoint = glm::vec3(0.0f);
    }

    hexMesh = meshIn;
    const float avgCellVolumeCbrt = std::cbrt(meshIn->getAverageCellVolume());
    lineWidth = glm::clamp(
            avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
    focusRadius = glm::clamp(
            avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
    reloadSphereRenderData();

    // Unload old data.
    shaderAttributesContext = sgl::ShaderAttributesPtr();

    // First, start with the rendering data for the context region.
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

    shaderAttributesContext = sgl::ShaderManager->createShaderAttributes(gatherShaderContext);
    shaderAttributesContext->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributesContext->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributesContext->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);
    shaderAttributesContext->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);
    shaderAttributesContext->addGeometryBuffer(
            attributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);


    // Now, continue with the rendering data for the focus region.
    loadFocusRepresentation();

    dirty = false;
    reRender = true;
    hasHitInformation = false;
}

void ClearViewRenderer_Faces::onResolutionChanged() {
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

void ClearViewRenderer_Faces::setUniformData() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    size_t fragmentBufferSize = expectedDepthComplexity * width * height;

    glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
    glm::vec3 lookingDirection(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

    sgl::ShaderManager->bindShaderStorageBuffer(0, startOffsetBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, fragmentBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShaderContext->setUniform("useShading", int(useShading));
    gatherShaderContext->setUniform("viewportW", width);
    gatherShaderContext->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShaderContext->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherShaderContext->setUniform("lookingDirection", lookingDirection);
    gatherShaderContext->setUniform("sphereCenter", focusPoint);
    gatherShaderContext->setUniform("sphereRadius", focusRadius);
    gatherShaderContext->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShaderContext->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    gatherShaderContext->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);

    sgl::ShaderProgram* gatherShaderFocus = shaderAttributesFocus->getShaderProgram();
    gatherShaderFocus->setUniform("viewportW", width);
    gatherShaderFocus->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShaderFocus->setUniform("cameraPosition", sceneData.camera->getPosition());
    if (gatherShaderFocus->hasUniform("lookingDirection")) {
        gatherShaderFocus->setUniform("lookingDirection", lookingDirection);
    }
    gatherShaderFocus->setUniform("sphereCenter", focusPoint);
    gatherShaderFocus->setUniform("sphereRadius", focusRadius);
    if (gatherShaderFocus->hasUniform("lineWidth")) {
        gatherShaderFocus->setUniform("lineWidth", lineWidth);
    }

    shaderProgramSurface->setUniform("viewportW", width);
    shaderProgramSurface->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgramSurface->setUniform("color", focusPointColor);

    if (shaderAttributesFocusPoints) {
        gatherShaderFocusSpheres->setUniform("viewportW", width);
        gatherShaderFocusSpheres->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
        gatherShaderFocusSpheres->setUniform("cameraPosition", sceneData.camera->getPosition());
        if (gatherShaderFocusSpheres->hasUniform("lookingDirection")) {
            gatherShaderFocusSpheres->setUniform("lookingDirection", lookingDirection);
        }
        gatherShaderFocusSpheres->setUniform("sphereCenter", focusPoint);
        gatherShaderFocusSpheres->setUniform("sphereRadius", focusRadius);
    }

    resolveShader->setUniform("viewportW", width);
    clearShader->setUniform("viewportW", width);
}

void ClearViewRenderer_Faces::clear() {
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

void ClearViewRenderer_Faces::gather() {
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
    sgl::Renderer->render(shaderAttributesContext);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Render the focus region lines.
    if (lineRenderingMode == LINE_RENDERING_MODE_WIREFRAME_FACES) {
        sgl::ShaderManager->bindShaderStorageBuffer(6, hexahedralCellFacesBuffer);
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(shaderAttributesFocus);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    if (lineRenderingMode == LINE_RENDERING_MODE_WIREFRAME_FACES) {
        glEnable(GL_CULL_FACE);
    }

    // Render the focus point.
    sgl::Renderer->setModelMatrix(sgl::matrixTranslation(focusPoint));
    sgl::Renderer->render(focusPointShaderAttributes);
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    if (shaderAttributesFocusPoints) {
        sgl::ShaderManager->bindShaderStorageBuffer(6, pointLocationsBuffer);
        //sgl::Renderer->render(shaderAttributesFocusPoints);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void ClearViewRenderer_Faces::resolve() {
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
