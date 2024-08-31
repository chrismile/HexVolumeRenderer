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

#include "Mesh/HexMesh/Renderers/Tubes/Tubes.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/Sphere.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "Helpers/SortingVendorFix.hpp"
#include "ClearViewRenderer.hpp"

// For compute shaders.
constexpr size_t BLOCK_SIZE = 256;

// When rendering spheres using instancing.
struct SphereInstancingData {
    glm::vec3 position;
    float padding;
    glm::vec4 color;
};

ClearViewRenderer::ClearViewRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
}

void ClearViewRenderer::loadClearViewBaseData() {
    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain"});

    reloadSphereRenderData();
    reloadFocusShaders();
}

void ClearViewRenderer::reloadSphereRenderData() {
    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec3> sphereVertexNormals;
    std::vector<uint32_t> sphereIndices;
    float sphereRadius = glm::clamp(
            focusRadius * FOCUS_SPHERE_SIZE_FACTOR, MIN_FOCUS_SPHERE_RADIUS, MAX_FOCUS_SPHERE_RADIUS);
    getSphereSurfaceRenderData(
            glm::vec3(0,0,0), sphereRadius, 20, 20,
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
}

void ClearViewRenderer::reloadFocusShaders() {
    std::string lineRenderingStyleDefineName;
    if (lineRenderingStyle == LINE_RENDERING_STYLE_HALO) {
        lineRenderingStyleDefineName = "LINE_RENDERING_STYLE_HALO";
    } else if (lineRenderingStyle == LINE_RENDERING_STYLE_TRON) {
        lineRenderingStyleDefineName = "LINE_RENDERING_STYLE_TRON";
    }

    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine(lineRenderingStyleDefineName, "");

    gatherShaderFocusWireframeFaces = sgl::ShaderManager->getShaderProgram(
            {"WireframeSurface.Vertex", "WireframeSurface.Fragment.ClearView.Focus"});
    gatherShaderFocusLines = sgl::ShaderManager->getShaderProgram(
            {"WireframeFocus.Vertex", "WireframeFocus.Geometry", "WireframeFocus.Fragment"});
    gatherShaderFocusTubes = sgl::ShaderManager->getShaderProgram(
            {"TubeWireframe.Vertex", "TubeWireframe.Fragment.ClearView.Focus"});
    gatherShaderFocusSpheres = sgl::ShaderManager->getShaderProgram(
            {"InstancedSpheres.Vertex", "InstancedSpheres.Fragment"});

    sgl::ShaderManager->removePreprocessorDefine(lineRenderingStyleDefineName);
}

void ClearViewRenderer::loadFocusRepresentation() {
    if (!hexMesh) {
        return;
    }

    // Unload old data.
    shaderAttributesFocus = sgl::ShaderAttributesPtr();
    shaderAttributesFocusPoints = sgl::ShaderAttributesPtr();
    pointLocationsBuffer = sgl::GeometryBufferPtr();

    printCounter = 0.5f;

    if (lineRenderingMode == LINE_RENDERING_MODE_WIREFRAME_FACES) {
        std::vector<uint32_t> indices;
        std::vector<HexahedralCellFace> hexahedralCellFaces;
        hexMesh->getSurfaceDataWireframeFaces(
                indices, hexahedralCellFaces, false,
                lineRenderingStyle == LINE_RENDERING_STYLE_TRON);

        shaderAttributesFocus = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusWireframeFaces);
        shaderAttributesFocus->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

        // Add the index buffer.
        sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
                sizeof(uint32_t)*indices.size(), indices.data(), sgl::INDEX_BUFFER);
        shaderAttributesFocus->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

        // Create an SSBO for the hexahedral cell faces.
        hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
                hexahedralCellFaces.size()*sizeof(HexahedralCellFace), hexahedralCellFaces.data(),
                sgl::SHADER_STORAGE_BUFFER);
    } else if (lineRenderingMode == LINE_RENDERING_MODE_TUBES || lineRenderingMode == LINE_RENDERING_MODE_TUBES_CAPPED
            || lineRenderingMode == LINE_RENDERING_MODE_TUBES_UNION) {
        std::vector<glm::vec3> lineVertices;
        std::vector<glm::vec4> lineColors;
        hexMesh->getCompleteWireframeData(
                lineVertices, lineColors,
                lineRenderingStyle == LINE_RENDERING_STYLE_TRON);

        const size_t numLines = lineVertices.size() / 2;
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<glm::vec4>> lineColorsList;
        lineCentersList.resize(numLines);
        lineColorsList.resize(numLines);
        for (size_t i = 0; i < numLines; i++) {
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(i);
            std::vector<glm::vec4>& lineAttributes = lineColorsList.at(i);
            lineCenters.push_back(lineVertices.at(i * 2));
            lineCenters.push_back(lineVertices.at(i * 2 + 1));
            lineAttributes.push_back(lineColors.at(i * 2));
            lineAttributes.push_back(lineColors.at(i * 2 + 1));
        }

        /*std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<glm::vec4>> lineColorsList;
        hexMesh->getCompleteWireframeTubeData(lineCentersList, lineColorsList);*/

        std::vector<uint32_t> triangleIndices;
        std::vector<glm::vec3> vertexPositions;
        std::vector<glm::vec3> vertexNormals;
        std::vector<glm::vec3> vertexTangents;
        std::vector<glm::vec4> vertexColors;
        if (lineRenderingMode == LINE_RENDERING_MODE_TUBES) {
            createTriangleTubesRenderDataGPU(
                    lineCentersList, lineColorsList, lineWidth * 0.5f, 8,
                    triangleIndices, vertexPositions, vertexNormals, vertexTangents, vertexColors);
        } else if (lineRenderingMode == LINE_RENDERING_MODE_TUBES_CAPPED) {
            createCappedTriangleTubesRenderDataCPU(
                    lineCentersList, lineColorsList, lineWidth * 0.5f, false, 8,
                    triangleIndices, vertexPositions, vertexNormals, vertexTangents, vertexColors);
        } else if (lineRenderingMode == LINE_RENDERING_MODE_TUBES_UNION) {
            createCappedTriangleTubesUnionRenderDataCPU(
                    hexMesh, lineWidth * 0.5f, 8, triangleIndices, vertexPositions,
                    vertexNormals, vertexTangents, vertexColors,
                    lineRenderingStyle == LINE_RENDERING_STYLE_TRON);
        }

        shaderAttributesFocus = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusTubes);
        shaderAttributesFocus->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

        sgl::GeometryBufferPtr tubeIndexBuffer = sgl::Renderer->createGeometryBuffer(
                triangleIndices.size() * sizeof(uint32_t), triangleIndices.data(), sgl::INDEX_BUFFER);
        shaderAttributesFocus->setIndexGeometryBuffer(tubeIndexBuffer, sgl::ATTRIB_UNSIGNED_INT);

        // Add the position buffer.
        sgl::GeometryBufferPtr tubeVertexBuffer = sgl::Renderer->createGeometryBuffer(
                vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeVertexBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

        // Add the normal buffer.
        sgl::GeometryBufferPtr tubeNormalBuffer = sgl::Renderer->createGeometryBuffer(
                vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

        // Add the tangent buffer.
        sgl::GeometryBufferPtr tubeTangentBuffer = sgl::Renderer->createGeometryBuffer(
                vertexTangents.size()*sizeof(glm::vec3), vertexTangents.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeTangentBuffer, "vertexTangent", sgl::ATTRIB_FLOAT, 3);

        // Add the color buffer.
        sgl::GeometryBufferPtr tubeColorBuffer = sgl::Renderer->createGeometryBuffer(
                vertexColors.size()*sizeof(glm::vec4), vertexColors.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                tubeColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);


        // Get points to fill holes and generate SSBOs with the point data to access when doing instancing.
        std::vector<glm::vec3> pointVertices;
        std::vector<glm::vec4> pointColors;
        hexMesh->getVertexTubeData(
                pointVertices, pointColors, lineRenderingStyle == LINE_RENDERING_STYLE_TRON);

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
        std::vector<uint32_t> sphereTriangleIndices;
        getSphereSurfaceRenderData(
                glm::vec3(0,0,0), lineWidth * 0.5f, 8, 8,
                sphereVertexPositions, sphereVertexNormals, sphereTriangleIndices);

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
                sphereTriangleIndices.size() * sizeof(uint32_t), sphereTriangleIndices.data(), sgl::INDEX_BUFFER);
        shaderAttributesFocusPoints->setIndexGeometryBuffer(focusPointIndexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributesFocusPoints->setInstanceCount(numInstancingPoints);
    } else if (lineRenderingMode == LINE_RENDERING_MODE_BILLBOARD_LINES) {
        std::vector<glm::vec3> lineVertices;
        std::vector<glm::vec4> lineColors;
        hexMesh->getCompleteWireframeData(
                lineVertices, lineColors, lineRenderingStyle == LINE_RENDERING_STYLE_TRON);

        shaderAttributesFocus = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusLines);
        shaderAttributesFocus->setVertexMode(sgl::VERTEX_MODE_LINES);

        // Add the position buffer.
        sgl::GeometryBufferPtr lineVertexBuffer = sgl::Renderer->createGeometryBuffer(
                lineVertices.size()*sizeof(glm::vec3), lineVertices.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                lineVertexBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

        // Add the color buffer.
        sgl::GeometryBufferPtr lineColorBuffer = sgl::Renderer->createGeometryBuffer(
                lineColors.size()*sizeof(glm::vec4), lineColors.data(), sgl::VERTEX_BUFFER);
        shaderAttributesFocus->addGeometryBuffer(
                lineColorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);
    }

    reRender = true;
}

void ClearViewRenderer::createFocusOutlineRenderingData() {
    std::vector<uint16_t> indices;
    const size_t numCircleSubdivisions = 1024;
    const size_t numVertices = numCircleSubdivisions * 2;
    indices.reserve(numVertices);
    for (size_t i = 0; i < numCircleSubdivisions + 1; i++) {
        indices.push_back((2 * i + 0) % numVertices);
        indices.push_back((2 * i + 1) % numVertices);
    }

    focusOutlineShaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgramFocusOutline);
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            indices.size() * sizeof(uint16_t), indices.data(), sgl::INDEX_BUFFER);
    focusOutlineShaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_SHORT);
    focusOutlineShaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLE_STRIP);
}

void ClearViewRenderer::reloadFocusOutlineGatherShader() {
    shaderProgramFocusOutline = sgl::ShaderManager->getShaderProgram(
            {"FocusOutlineShader.Vertex", "FocusOutlineShader.Fragment"});
}

void ClearViewRenderer::renderFocusOutline(size_t fragmentBufferSize) {
    if ((useScreenSpaceLens && screenSpaceLensPixelRadius >= 0.0f) || (!useScreenSpaceLens && focusRadius >= 0.0f)) {
        if (useScreenSpaceLens) {
            shaderProgramFocusOutline->setUniform("circleScreenPosition", focusPointScreen);
            shaderProgramFocusOutline->setUniform("circleScreenRadius", screenSpaceLensPixelRadius);
            shaderProgramFocusOutline->setUniform("circleDepth", 0.0f);
            shaderProgramFocusOutline->setUniform("circlePixelThickness", focusOutlineWidth);
        } else {
            const glm::mat4& viewMatrix = sceneData.camera->getViewMatrix();
            const glm::mat4& vpMatrix = sceneData.camera->getViewProjMatrix();
            glm::vec3 viewPosition = sgl::transformPoint(viewMatrix, focusPoint);
            glm::vec3 ndcPosition = sgl::transformPoint(vpMatrix, focusPoint);
            glm::vec2 screenPosition =
                    (glm::vec2(ndcPosition.x, ndcPosition.y) * glm::vec2(0.5)
                     + glm::vec2(0.5)) * glm::vec2(windowWidth, windowHeight);
            float viewportDist = windowHeight * 0.5f / std::tan(sceneData.camera->getFOVy() * 0.5f);
            float screenRadius = -viewportDist * focusRadius / viewPosition.z;
            shaderProgramFocusOutline->setUniform("circleScreenPosition", screenPosition);
            shaderProgramFocusOutline->setUniform("circleScreenRadius", screenRadius);
            shaderProgramFocusOutline->setUniform("circleDepth", -viewPosition.z);
            float thickness = -viewportDist * focusOutlineWidth * 0.0005f / viewPosition.z;
            shaderProgramFocusOutline->setUniform("circlePixelThickness", thickness);
        }
        shaderProgramFocusOutline->setUniform("viewportSize", glm::ivec2(windowWidth, windowHeight));
        shaderProgramFocusOutline->setUniform("focusOutlineColor", focusOutlineColor);
        shaderProgramFocusOutline->setUniform("viewportW", windowWidth);
        shaderProgramFocusOutline->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);

        sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
        sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        sgl::Renderer->render(focusOutlineShaderAttributes);
    }
}

void ClearViewRenderer::setSortingAlgorithmDefine() {
    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_PRIORITY_QUEUE) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "frontToBackPQ");
        if (getIsGpuVendorAmd()) {
            sgl::ShaderManager->addPreprocessorDefine("INITIALIZE_ARRAY_POW2", "");
        }
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BUBBLE_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "bubbleSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_INSERTION_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "insertionSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_SHELL_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "shellSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_MAX_HEAP) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "heapSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BITONIC_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "bitonicSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "quicksort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "quicksortHybrid");
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
            || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        sgl::ShaderManager->addPreprocessorDefine("USE_QUICKSORT", "");
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_QUICKSORT");
    }
}

void ClearViewRenderer::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void ClearViewRenderer::updateDepthCueMode() {
    if (useDepthCues) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SCREEN_SPACE_POSITION", "");
        sgl::ShaderManager->addPreprocessorDefine("USE_DEPTH_CUES", "");

        if (hexMesh && filteredCellVertices.empty()) {
            updateDepthCueGeometryData();
        }
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
}

void ClearViewRenderer::updateDepthCueGeometryData() {
    filteredCellVertices = hexMesh->getFilteredVertices();
    std::vector<glm::vec4> filteredCellVerticesVec4;
    for (const glm::vec3& point : filteredCellVertices) {
        filteredCellVerticesVec4.push_back(glm::vec4(point.x, point.y, point.z, 1.0f));
    }
    filteredCellVerticesBuffer = sgl::Renderer->createGeometryBuffer(
            filteredCellVerticesVec4.size() * sizeof(glm::vec4), filteredCellVerticesVec4.data(),
            sgl::SHADER_STORAGE_BUFFER);

    depthMinMaxBuffers[0] = sgl::Renderer->createGeometryBuffer(
            sgl::iceil(filteredCellVertices.size(), BLOCK_SIZE) * sizeof(glm::vec2),
            sgl::SHADER_STORAGE_BUFFER);
    depthMinMaxBuffers[1] = sgl::Renderer->createGeometryBuffer(
            sgl::iceil(filteredCellVertices.size(), BLOCK_SIZE * BLOCK_SIZE * 2) * sizeof(glm::vec2),
            sgl::SHADER_STORAGE_BUFFER);
}

void ClearViewRenderer::setUniformDataDepthCues(sgl::ShaderProgramPtr shaderProgram) {
    if (useDepthCues && hexMesh && computeDepthCuesOnGpu) {
        sgl::ShaderManager->bindShaderStorageBuffer(12, filteredCellVerticesBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(11, depthMinMaxBuffers[0]);
        uint32_t numVertices = filteredCellVerticesBuffer->getSize() / sizeof(glm::vec4);
        uint32_t numBlocks = sgl::iceil(numVertices, BLOCK_SIZE);
        computeDepthValuesShaderProgram->setUniform("numVertices", numVertices);
        computeDepthValuesShaderProgram->setUniform("nearDist", sceneData.camera->getNearClipDistance());
        computeDepthValuesShaderProgram->setUniform("farDist", sceneData.camera->getFarClipDistance());
        computeDepthValuesShaderProgram->setUniform("cameraViewMatrix", sceneData.camera->getViewMatrix());
        computeDepthValuesShaderProgram->setUniform(
                "cameraProjectionMatrix", sceneData.camera->getProjectionMatrix());
        computeDepthValuesShaderProgram->dispatchCompute(numBlocks);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        minMaxReduceDepthShaderProgram->setUniform("nearDist", sceneData.camera->getNearClipDistance());
        minMaxReduceDepthShaderProgram->setUniform("farDist", sceneData.camera->getFarClipDistance());
        int iteration = 0;
        uint32_t inputSize;
        while (numBlocks > 1) {
            if (iteration != 0) {
                // Already bound for computeDepthValuesShaderProgram if i == 0.
                sgl::ShaderManager->bindShaderStorageBuffer(11, depthMinMaxBuffers[iteration % 2]);
            }
            sgl::ShaderManager->bindShaderStorageBuffer(12, depthMinMaxBuffers[(iteration + 1) % 2]);

            inputSize = numBlocks;
            numBlocks = sgl::iceil(numBlocks, BLOCK_SIZE*2);
            minMaxReduceDepthShaderProgram->setUniform("sizeOfInput", inputSize);
            minMaxReduceDepthShaderProgram->dispatchCompute(numBlocks);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            iteration++;
        }

        // Bind the output of ComputeDepthValues.glsl to position 12 for Lighting.glsl if no reduction was necessary.
        if (iteration == 0) {
            sgl::ShaderManager->bindShaderStorageBuffer(12, depthMinMaxBuffers[0]);
        }
    }

    if (useDepthCues && hexMesh && !computeDepthCuesOnGpu) {
        bool useBoundingBox = hexMesh->getNumVertices() > 1000;
        if (useBoundingBox) {
            const sgl::AABB3& boundingBox = hexMesh->getModelBoundingBox();
            sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed(sceneData.camera->getViewMatrix());
            minDepth = -screenSpaceBoundingBox.getMaximum().z;
            maxDepth = -screenSpaceBoundingBox.getMinimum().z;
        } else {
            glm::mat4 viewMatrix = sceneData.camera->getViewMatrix();
            minDepth = std::numeric_limits<float>::max();
            maxDepth = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
            #pragma omp parallel for default(none) shared(viewMatrix, filteredCellVertices) \
            reduction(min: minDepth) reduction(max: maxDepth)
#endif
            for (size_t pointIdx = 0; pointIdx < filteredCellVertices.size(); pointIdx++) {
                const glm::vec3& point = filteredCellVertices.at(pointIdx);
                float depth = -sgl::transformPoint(viewMatrix, point).z;
                minDepth = std::min(minDepth, depth);
                maxDepth = std::max(maxDepth, depth);
            }
        }

        minDepth = glm::clamp(
                minDepth, sceneData.camera->getFarClipDistance(), sceneData.camera->getNearClipDistance());
        maxDepth = glm::clamp(
                maxDepth, sceneData.camera->getFarClipDistance(), sceneData.camera->getNearClipDistance());

        shaderProgram->setUniformOptional("minDepth", minDepth);
        shaderProgram->setUniformOptional("maxDepth", maxDepth);
    }

    shaderProgram->setUniformOptional("depthCueStrength", depthCueStrength);
}

void ClearViewRenderer::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(0, 0, 746, 1426);
    if (ImGui::Begin(windowName.c_str(), &showRendererWindow)) {
        childClassRenderGuiBegin();
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            if (lineRenderingMode == LINE_RENDERING_MODE_TUBES || lineRenderingMode == LINE_RENDERING_MODE_TUBES_CAPPED
                || lineRenderingMode == LINE_RENDERING_MODE_TUBES_UNION) {
                loadFocusRepresentation();
            }
            reRender = true;
        }
        if (clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::Checkbox("Modulate Line Thickness by Depth", &modulateLineThicknessByDepth)) {
            reloadGatherShader(true);
            reRender = true;
        }
        if (!useScreenSpaceLens
                && ImGui::SliderFloat("Focus Radius", &focusRadius, MIN_FOCUS_RADIUS, MAX_FOCUS_RADIUS)) {
            reloadSphereRenderData();
            reRender = true;
        }
        if (!useScreenSpaceLens && ImGui::SliderFloat3("Focus Point", &focusPoint.x, -0.4f, 0.4f)) {
            hasHitInformation = false;
            reRender = true;
        }
        if (!useScreenSpaceLens && clearViewRendererType != CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::ColorEdit4("Focus Point Color", &focusPointColor.x)) {
            reRender = true;
        }
        if (clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::Checkbox("Use Focus Outline", &useFocusOutline)) {
            reloadGatherShader(true);
            reRender = true;
        }
        if (useFocusOutline && clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::ColorEdit4("Focus Outline Color", &focusOutlineColor.x)) {
            reRender = true;
        }
        if (useFocusOutline && clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::SliderFloat("Focus Outline Width", &focusOutlineWidth, 1.0f, 20.0f)) {
            reRender = true;
        }
        if (useFocusOutline && clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::Checkbox("Clip Focus Outline", &clipFocusOutline)) {
            reRender = true;
        }
        /*if (clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::Checkbox("Use Depth Cues", &useDepthCues)) {
            updateDepthCueMode();
            reloadGatherShader(true);
            reRender = true;
        }*/
        if (clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED && ImGui::SliderFloat(
                "Depth Cue Strength", &depthCueStrength, 0.0f, 1.0f)) {
            if (depthCueStrength <= 0.0f && useDepthCues) {
                useDepthCues = false;
                updateDepthCueMode();
                reloadGatherShader(true);
            }
            if (depthCueStrength > 0.0f && !useDepthCues) {
                useDepthCues = true;
                updateDepthCueMode();
                reloadGatherShader(true);
            }
            reRender = true;
        }
        if (clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES
                && !useWeightedVertexAttributes && ImGui::Checkbox("Use Shading", &useShading)) {
            reRender = true;
        }
        if (clearViewRendererType != CLEAR_VIEW_RENDERER_TYPE_VOLUME
                && clearViewRendererType != CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
                && ImGui::Checkbox("Use Weighted Vertex Attributes", &useWeightedVertexAttributes)) {
            useShading = false;
            if (this->hexMesh) uploadVisualizationMapping(hexMesh, false);
        }
        if (useWeightedVertexAttributes && ImGui::Checkbox("Use Volume Weighting", &useVolumeWeighting)) {
            if (this->hexMesh) uploadVisualizationMapping(hexMesh, false);
            reRender = true;
        }
        if (clearViewRendererType != CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED && ImGui::Combo(
                "Line Rendering", (int*)&lineRenderingMode, LINE_RENDERING_MODE_NAMES,
                NUM_LINE_RENDERING_MODES)) {
            loadFocusRepresentation();
            reRender = true;
        }
        if (clearViewRendererType != CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED && ImGui::Combo(
                "Line Style", (int*)&lineRenderingStyle, LINE_RENDERING_STYLE_NAMES,
                IM_ARRAYSIZE(LINE_RENDERING_STYLE_NAMES))) {
            reloadFocusShaders();
            loadFocusRepresentation();
            reRender = true;
        }
        if (LET_USER_SELECT_LOD_STYLE) {
            if (ImGui::SliderFloat(
                    "LOD Merge Factor", &lodSettings.lodMergeFactor, 0.999f, 4.0f, "%.3f")) {
                if (hexMesh) {
                    uploadVisualizationMapping(hexMesh, false);
                    reRender = true;
                }
            }
            if (ImGui::Checkbox("Use Volume and Area Measures", &lodSettings.useVolumeAndAreaMeasures)) {
                if (hexMesh) {
                    uploadVisualizationMapping(hexMesh, false);
                    reRender = true;
                }
            }
            if (ImGui::Checkbox("Use Weights for Merging", &lodSettings.useWeightsForMerging)) {
                if (hexMesh) {
                    uploadVisualizationMapping(hexMesh, false);
                    reRender = true;
                }
            }
            if (ImGui::Checkbox("Use #Cells/Volume", &lodSettings.useNumCellsOrVolume)) {
                if (hexMesh) {
                    uploadVisualizationMapping(hexMesh, false);
                    reRender = true;
                }
            }
        }
        childClassRenderGuiEnd();
    }
    ImGui::End();
}

void ClearViewRenderer::update(float dt) {
    if (!useScreenSpaceLens) {
#ifdef SGL_INPUT_API_V2
        if (sgl::Keyboard->getModifier(ImGuiKey_ModShift)) {
#else
        if (sgl::Keyboard->getModifier() & KMOD_SHIFT) {
#endif
            if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
                float scrollAmount = float(sgl::Mouse->getScrollWheel()) * dt * 0.5f;
                focusRadius += scrollAmount;
                focusRadius = glm::clamp(focusRadius, 0.001f, 0.4f);
                reRender = true;
            }
        }

        Pickable::updatePickable(dt, reRender, sceneData);
    } else {
#ifdef SGL_INPUT_API_V2
        if (sgl::Keyboard->getModifier(ImGuiKey_ModShift)) {
#else
        if (sgl::Keyboard->getModifier() & KMOD_SHIFT) {
#endif
            if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
                float scrollAmount = float(sgl::Mouse->getScrollWheel()) * dt * 800.0f;
                screenSpaceLensPixelRadius += scrollAmount;
                //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
                //int width = window->getWidth();
                //int height = window->getHeight();
                int width = (*sceneData.sceneTexture)->getW();
                int height = (*sceneData.sceneTexture)->getH();
                screenSpaceLensPixelRadius = glm::min(screenSpaceLensPixelRadius, float(std::max(width, height)));
                screenSpaceLensPixelRadius = std::max(screenSpaceLensPixelRadius, 0.0f);
                reRender = true;
            }
        }

#ifdef SGL_INPUT_API_V2
        if (sgl::Keyboard->getModifier(ImGuiKey_ModCtrl)) {
#else
        if (sgl::Keyboard->getModifier() & KMOD_CTRL) {
#endif
            if (sgl::Mouse->buttonPressed(1) || (sgl::Mouse->isButtonDown(1) && sgl::Mouse->mouseMoved())) {
                sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
                int mouseX = sgl::Mouse->getX();
                int mouseY = sgl::Mouse->getY();
                int viewportWidth = (*sceneData.sceneTexture)->getW();
                int viewportHeight = (*sceneData.sceneTexture)->getH();
                if (sgl::ImGuiWrapper::get()->getUseDockSpaceMode()) {
                    mouseX -= sceneData.pickingOffsetX;
                    mouseY -= sceneData.pickingOffsetY;
                }
                focusPointScreen = glm::vec2(mouseX, viewportHeight - mouseY - 1);
                glm::vec2 normalizedPosition = glm::vec2(
                        (focusPointScreen.x * 2.0f - (float(viewportWidth) - 1.0f)) / float(viewportWidth),
                        focusPointScreen.y / float(viewportHeight) * 2.0f - 1.0f
                );
                //if (sgl::Mouse->buttonPressed(1)) {
                //    std::cout << "(" << normalizedPosition.x << ", " << normalizedPosition.y << ")," << std::endl;
                //}
                reRender = true;
            }
        }

        /*printCounter += 1.0f / 30.0f;
        if (printCounter >= 0.5f) {
            sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
            int mouseX = sgl::Mouse->getX();
            int mouseY = sgl::Mouse->getY();
            focusPointScreen = glm::vec2(mouseX, window->getHeight() - mouseY - 1);
            glm::vec2 normalizedPosition = glm::vec2(
                    (focusPointScreen.x - (windowWidth - 1.0f) * 0.5f) / windowHeight,
                    focusPointScreen.y / windowHeight * 2.0f - 1.0f
            );
            std::cout << "(" << normalizedPosition.x << ", " << normalizedPosition.y << ")," << std::endl;
            printCounter -= 0.5f;
        }*/
    }
}
