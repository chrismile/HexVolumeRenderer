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
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Tubes/Tubes.hpp"
#include "Helpers/Sphere.hpp"
#include "ClearViewRenderer_Volume.hpp"

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
    // RGBA color of the node.
    uint32_t color;
    // Depth value of the fragment (in view space).
    float depth;
    // The index of the next node in the "nodes" array.
    uint32_t next;
};

// When rendering spheres using instancing.
struct SphereInstancingData {
    glm::vec3 position;
    float padding;
    glm::vec4 color;
};

ClearViewRenderer_Volume::ClearViewRenderer_Volume(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListVolumeGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine(
            "MAX_NUM_FRAGS_VOLUME", sgl::toString(maxNumFragmentsSortingVolume));
    sgl::ShaderManager->addPreprocessorDefine(
            "MAX_NUM_FRAGS_SURFACE", sgl::toString(maxNumFragmentsSortingSurface));

    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain"});

    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec3> sphereVertexNormals;
    std::vector<uint32_t> sphereIndices;
    getSphereSurfaceRenderData(
            glm::vec3(0,0,0), 0.002f, 20, 20,
            sphereVertexPositions, sphereVertexNormals, sphereIndices);

    focusPointShaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgramSurface);
    focusPointShaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    sgl::GeometryBufferPtr focusPointVertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            sphereVertexPositions.size() * sizeof(glm::vec3), sphereVertexPositions.data(), sgl::VERTEX_BUFFER);
    focusPointShaderAttributes->addGeometryBuffer(
            focusPointVertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    sgl::GeometryBufferPtr focusPointVertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            sphereVertexNormals.size() * sizeof(glm::vec3), sphereVertexNormals.data(), sgl::VERTEX_BUFFER);
    focusPointShaderAttributes->addGeometryBuffer(
            focusPointVertexNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);
    sgl::GeometryBufferPtr focusPointIndexBuffer = sgl::Renderer->createGeometryBuffer(
            sphereIndices.size() * sizeof(uint32_t), sphereIndices.data(), sgl::INDEX_BUFFER);
    focusPointShaderAttributes->setIndexGeometryBuffer(focusPointIndexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    gatherShaderVolumeFrontFaces = sgl::ShaderManager->getShaderProgram(
            {"MeshLinkedListVolume.Vertex", "MeshLinkedListVolume.Fragment.ClearView.Context.FrontFace"});
    gatherShaderVolumeBackFaces = sgl::ShaderManager->getShaderProgram(
            {"MeshLinkedListVolume.Vertex", "MeshLinkedListVolume.Fragment.ClearView.Context.BackFace"});
    gatherShaderFocusLines = sgl::ShaderManager->getShaderProgram(
            {"WireframeFocus.Vertex", "WireframeFocus.Geometry", "WireframeFocus.Fragment"});
    gatherShaderFocusTubes = sgl::ShaderManager->getShaderProgram(
            {"TubeWireframe.Vertex", "TubeWireframe.Fragment.ClearView.Focus"});
    gatherShaderFocusSpheres = sgl::ShaderManager->getShaderProgram(
            {"InstancedSpheres.Vertex", "InstancedSpheres.Fragment"});
    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListVolumeResolve.Vertex", "LinkedListVolumeResolve.Fragment"});
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListVolumeClear.Vertex", "LinkedListVolumeClear.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    blitRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    clearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
}

void ClearViewRenderer_Volume::generateVisualizationMapping(HexMeshPtr meshIn) {
    mesh = meshIn;

    // Unload old data.
    shaderAttributesVolumeFrontFaces = sgl::ShaderAttributesPtr();
    shaderAttributesVolumeBackFaces = sgl::ShaderAttributesPtr();

    // First, start with the rendering data for the context region.
    std::vector<uint32_t> indices;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec4> colors;
    meshIn->getVolumeData_Volume(indices, vertices, normals, colors);

    shaderAttributesVolumeFrontFaces = sgl::ShaderManager->createShaderAttributes(gatherShaderVolumeFrontFaces);
    shaderAttributesVolumeFrontFaces->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
    shaderAttributesVolumeFrontFaces->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertices.size()*sizeof(glm::vec3), (void*)&vertices.front(), sgl::VERTEX_BUFFER);
    shaderAttributesVolumeFrontFaces->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    /*sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            normals.size()*sizeof(glm::vec3), (void*)&normals.front(), sgl::VERTEX_BUFFER);
    shaderAttributesVolumeFrontFaces->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);*/

    // Add the color buffer.
    sgl::GeometryBufferPtr colorBuffer = sgl::Renderer->createGeometryBuffer(
            colors.size()*sizeof(glm::vec4), (void*)&colors.front(), sgl::VERTEX_BUFFER);
    shaderAttributesVolumeFrontFaces->addGeometryBuffer(
            colorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);

    shaderAttributesVolumeBackFaces = shaderAttributesVolumeFrontFaces->copy(gatherShaderVolumeBackFaces);


    // Now, continue with the rendering data for the focus region.
    loadFocusRepresentation();

    dirty = false;
    reRender = true;
    hasHitInformation = false;
}

void ClearViewRenderer_Volume::loadFocusRepresentation() {
    if (!mesh) {
        return;
    }

    // Unload old data.
    shaderAttributesFocus = sgl::ShaderAttributesPtr();
    shaderAttributesFocusPoints = sgl::ShaderAttributesPtr();
    pointLocationsBuffer = sgl::GeometryBufferPtr();

    if (useTubes) {
        std::vector<glm::vec3> lineVertices;
        std::vector<glm::vec4> lineColors;
        mesh->getCompleteWireframeData(lineVertices, lineColors);

        const size_t numLines = lineVertices.size() / 2;
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<glm::vec4>> lineAttributesList;
        lineCentersList.resize(numLines);
        lineAttributesList.resize(numLines);
        for (size_t i = 0; i < numLines; i++) {
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(i);
            std::vector<glm::vec4>& lineAttributes = lineAttributesList.at(i);
            lineCenters.push_back(lineVertices.at(i * 2));
            lineCenters.push_back(lineVertices.at(i * 2 + 1));
            lineAttributes.push_back(lineColors.at(i * 2));
            lineAttributes.push_back(lineColors.at(i * 2 + 1));
        }

        std::vector<uint32_t> triangleIndices;
        std::vector<glm::vec3> vertexPositions;
        std::vector<glm::vec3> vertexNormals;
        std::vector<glm::vec3> vertexTangents;
        std::vector<glm::vec4> vertexColors;
        createTriangleTubesRenderDataGPU(
                lineCentersList, lineAttributesList, lineWidth / 2.0f, 8,
                triangleIndices, vertexPositions, vertexNormals, vertexTangents, vertexColors);

        shaderAttributesFocus = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusTubes);
        shaderAttributesFocus->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

        sgl::GeometryBufferPtr tubeIndexBuffer = sgl::Renderer->createGeometryBuffer(
                triangleIndices.size() * sizeof(uint32_t), triangleIndices.data(), sgl::INDEX_BUFFER);
        shaderAttributesFocus->setIndexGeometryBuffer(tubeIndexBuffer, sgl::ATTRIB_UNSIGNED_INT);

        // Add the position buffer.
        sgl::GeometryBufferPtr tubeVertexBuffer = sgl::Renderer->createGeometryBuffer(
                vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeVertexBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

        // Add the normal buffer.
        sgl::GeometryBufferPtr tubeNormalBuffer = sgl::Renderer->createGeometryBuffer(
                vertexNormals.size()*sizeof(glm::vec3), (void*)&vertexNormals.front(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

        // Add the tangent buffer.
        sgl::GeometryBufferPtr tubeTangentBuffer = sgl::Renderer->createGeometryBuffer(
                vertexTangents.size()*sizeof(glm::vec3), (void*)&vertexTangents.front(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeTangentBuffer, "vertexTangent", sgl::ATTRIB_FLOAT, 3);

        // Add the color buffer.
        sgl::GeometryBufferPtr tubeColorBuffer = sgl::Renderer->createGeometryBuffer(
                vertexColors.size()*sizeof(glm::vec4), (void*)&vertexColors.front(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);


        // Get points to fill holes and generate SSBOs with the point data to access when doing instancing.
        std::vector<glm::vec3> pointVertices;
        std::vector<glm::vec4> pointColors;
        mesh->getCompleteVertexData(pointVertices, pointColors);

        const size_t numInstancingPoints = pointVertices.size();
        std::vector<SphereInstancingData> sphereInstancingData;
        sphereInstancingData.resize(numInstancingPoints);
        for (size_t i = 0; i < numInstancingPoints; i++) {
            SphereInstancingData& sphereData = sphereInstancingData.at(i);
            sphereData.position = pointVertices.at(i);
            sphereData.color = pointColors.at(i);
        }
        pointLocationsBuffer = sgl::Renderer->createGeometryBuffer(
                sizeof(SphereInstancingData) * numInstancingPoints, sphereInstancingData.data(),
                sgl::SHADER_STORAGE_BUFFER);

        // Get the sphere render data.
        std::vector<glm::vec3> sphereVertexPositions;
        std::vector<glm::vec3> sphereVertexNormals;
        std::vector<uint32_t> sphereIndices;
        getSphereSurfaceRenderData(
                glm::vec3(0,0,0), lineWidth * 0.5f, 8, 8,
                sphereVertexPositions, sphereVertexNormals, sphereIndices);

        shaderAttributesFocusPoints = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusSpheres);
        shaderAttributesFocusPoints->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        sgl::GeometryBufferPtr focusPointVertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
                sphereVertexPositions.size() * sizeof(glm::vec3), sphereVertexPositions.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocusPoints->addGeometryBuffer(
                focusPointVertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
        /*sgl::GeometryBufferPtr focusPointVertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
                sphereVertexNormals.size() * sizeof(glm::vec3), sphereVertexNormals.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocusPoints->addGeometryBuffer(
                focusPointVertexNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);*/
        sgl::GeometryBufferPtr focusPointIndexBuffer = sgl::Renderer->createGeometryBuffer(
                sphereIndices.size() * sizeof(uint32_t), sphereIndices.data(), sgl::INDEX_BUFFER);
        shaderAttributesFocusPoints->setIndexGeometryBuffer(focusPointIndexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributesFocusPoints->setInstanceCount(numInstancingPoints);
    } else {
        std::vector<glm::vec3> lineVertices;
        std::vector<glm::vec4> lineColors;
        mesh->getCompleteWireframeData(lineVertices, lineColors);

        shaderAttributesFocus = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusLines);
        shaderAttributesFocus->setVertexMode(sgl::VERTEX_MODE_LINES);

        // Add the position buffer.
        sgl::GeometryBufferPtr lineVertexBuffer = sgl::Renderer->createGeometryBuffer(
                lineVertices.size()*sizeof(glm::vec3), (void*)&lineVertices.front(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                lineVertexBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

        // Add the color buffer.
        sgl::GeometryBufferPtr lineColorBuffer = sgl::Renderer->createGeometryBuffer(
                lineColors.size()*sizeof(glm::vec4), (void*)&lineColors.front(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                lineColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);
    }

    reRender = true;
}

void ClearViewRenderer_Volume::setSortingAlgorithmDefine() {
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

void ClearViewRenderer_Volume::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

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

void ClearViewRenderer_Volume::setUniformData() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    size_t fragmentBufferSizeVolume = expectedDepthComplexityVolume * width * height;
    size_t fragmentBufferSizeSurface = expectedDepthComplexitySurface * width * height;

    glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
    glm::vec3 lookingDirection(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBufferVolumeFrontFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(1, fragmentBufferVolumeBackFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(2, fragmentBufferSurface);
    sgl::ShaderManager->bindShaderStorageBuffer(3, startOffsetVolumeFrontFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(4, startOffsetVolumeBackFaces);
    sgl::ShaderManager->bindShaderStorageBuffer(5, startOffsetSurface);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, fragCounterVolumeFrontFaces);
    sgl::ShaderManager->bindAtomicCounterBuffer(1, fragCounterVolumeBackFaces);
    sgl::ShaderManager->bindAtomicCounterBuffer(2, fragCounterSurface);

    sgl::ShaderProgram* gatherShaderFocus = shaderAttributesFocus->getShaderProgram();
    sgl::ShaderProgram* gatherShaders[] = {
            gatherShaderVolumeFrontFaces.get(), gatherShaderVolumeBackFaces.get(), gatherShaderFocus
    };
    for (sgl::ShaderProgram* gatherShader : gatherShaders) {
        gatherShader->setUniform("viewportW", width);
        gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
        if (gatherShader->hasUniform("lightDirection")) {
            gatherShader->setUniform("lightDirection", sceneData.lightDirection);
        }
        if (gatherShader->hasUniform("lookingDirection")) {
            gatherShader->setUniform("lookingDirection", lookingDirection);
        }
        gatherShader->setUniform("sphereCenter", focusPoint);
        gatherShader->setUniform("sphereRadius", focusRadius);
    }
    gatherShaderVolumeFrontFaces->setUniform(
            "linkedListVolumeFrontFacesCapacity", (unsigned int)fragmentBufferSizeVolume);
    gatherShaderVolumeBackFaces->setUniform(
            "linkedListVolumeBackFacesCapacity", (unsigned int)fragmentBufferSizeVolume);
    gatherShaderFocus->setUniform(
            "linkedListSurfaceCapacity", (unsigned int)fragmentBufferSizeSurface);
    if (gatherShaderFocus->hasUniform("lineWidth")) {
        gatherShaderFocus->setUniform("lineWidth", lineWidth);
    }

    shaderProgramSurface->setUniform("viewportW", width);
    shaderProgramSurface->setUniform(
            "linkedListSurfaceCapacity", (unsigned int)fragmentBufferSizeSurface);
    shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgramSurface->setUniform("color", focusPointColor);

    if (shaderAttributesFocusPoints) {
        gatherShaderFocusSpheres->setUniform("viewportW", width);
        gatherShaderFocusSpheres->setUniform("linkedListSurfaceCapacity", (unsigned int)fragmentBufferSizeSurface);
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

void ClearViewRenderer_Volume::clear() {
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

void ClearViewRenderer_Volume::gather() {
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

    // Render the focus region lines.
    sgl::Renderer->render(shaderAttributesFocus);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Render the focus point.
    sgl::Renderer->setModelMatrix(sgl::matrixTranslation(focusPoint));
    sgl::Renderer->render(focusPointShaderAttributes);
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    if (shaderAttributesFocusPoints) {
        sgl::ShaderManager->bindShaderStorageBuffer(6, pointLocationsBuffer);
        sgl::Renderer->render(shaderAttributesFocusPoints);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void ClearViewRenderer_Volume::resolve() {
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

void ClearViewRenderer_Volume::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void ClearViewRenderer_Volume::renderGui() {
    if (ImGui::Begin("ClearView Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Focus Radius", &focusRadius, 0.001f, 0.4f)) {
            reRender = true;
        }
        if (ImGui::SliderFloat3("Focus Point", &focusPoint.x, -0.4f, 0.4f)) {
            hasHitInformation = false;
            reRender = true;
        }
        if (ImGui::ColorEdit4("Focus Point Color", &focusPointColor.x)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Line Width", &lineWidth, 0.0001f, 0.002f, "%.4f")) {
            if (useTubes) {
                loadFocusRepresentation();
            }
            reRender = true;
        }
        if (ImGui::Checkbox("Use Tubes", &useTubes)) {
            loadFocusRepresentation();
            reRender = true;
        }
    }
    ImGui::End();
}

void ClearViewRenderer_Volume::update(float dt) {
    if (sgl::Keyboard->getModifier() & KMOD_SHIFT) {
        if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
            float scrollAmount = sgl::Mouse->getScrollWheel() * dt * 1.0;
            focusRadius += scrollAmount;
            focusRadius = glm::clamp(focusRadius, 0.001f, 0.4f);
            reRender = true;
        }
    }

    if (sgl::Keyboard->getModifier() & KMOD_CTRL) {
        if (sgl::Mouse->buttonPressed(1) || (sgl::Mouse->isButtonDown(1) && sgl::Mouse->mouseMoved())) {
            int mouseX = sgl::Mouse->getX();
            int mouseY = sgl::Mouse->getY();
            bool rayHasHitMesh = this->sceneData.rayMeshIntersection.pickPointScreen(
                    mouseX, mouseY, firstHit, lastHit);
            if (rayHasHitMesh) {
                focusPoint = firstHit;
                hitLookingDirection = glm::normalize(firstHit - sceneData.camera->getPosition());
                hasHitInformation = true;
                reRender = true;
            }
        }

        if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
            if (!hasHitInformation) {
                glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
                glm::vec3 lookingDirection = glm::vec3(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

                float moveAmount = sgl::Mouse->getScrollWheel() * dt * 2.0;
                glm::vec3 moveDirection = focusPoint - sceneData.camera->getPosition();
                moveDirection *= float(sgl::sign(glm::dot(lookingDirection, moveDirection)));
                if (glm::length(moveDirection) < 1e-4) {
                    moveDirection = lookingDirection;
                }
                moveDirection = glm::normalize(moveDirection);
                focusPoint = focusPoint + moveAmount * moveDirection;
            } else {
                float moveAmount = sgl::Mouse->getScrollWheel() * dt;
                glm::vec3 newFocusPoint = focusPoint + moveAmount * hitLookingDirection;
                float t = glm::dot(newFocusPoint - firstHit, hitLookingDirection);
                t = glm::clamp(t, 0.0f, glm::length(lastHit - firstHit));
                focusPoint = firstHit + t * hitLookingDirection;
            }
            reRender = true;
        }
    }
}