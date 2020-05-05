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
#include "Helpers/LineRenderingDefines.hpp"
#include "ClearViewRenderer.hpp"

const char* const sortingModeStrings[] = {"Priority Queue", "Bubble Sort", "Insertion Sort", "Shell Sort", "Max Heap"};

// Choice of sorting algorithm
static int sortingAlgorithmMode = 0;

// When rendering spheres using instancing.
struct SphereInstancingData {
    glm::vec3 position;
    float padding;
    glm::vec4 color;
};

ClearViewRenderer::ClearViewRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
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
    if (!mesh) {
        return;
    }

    // Unload old data.
    shaderAttributesFocus = sgl::ShaderAttributesPtr();
    shaderAttributesFocusPoints = sgl::ShaderAttributesPtr();
    pointLocationsBuffer = sgl::GeometryBufferPtr();

    if (lineRenderingMode == LINE_RENDERING_MODE_WIREFRAME_FACES) {
        std::vector<uint32_t> indices;
        std::vector<HexahedralCellFace> hexahedralCellFaces;
        mesh->getSurfaceDataWireframeFaces(
                indices, hexahedralCellFaces, false,
                lineRenderingStyle == LINE_RENDERING_STYLE_TRON);

        shaderAttributesFocus = sgl::ShaderManager->createShaderAttributes(gatherShaderFocusWireframeFaces);
        shaderAttributesFocus->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

        // Add the index buffer.
        sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
                sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
        shaderAttributesFocus->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

        // Create an SSBO for the hexahedral cell faces.
        hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
                hexahedralCellFaces.size()*sizeof(HexahedralCellFace), (void*)&hexahedralCellFaces.front(),
                sgl::SHADER_STORAGE_BUFFER);
    } else if (lineRenderingMode == LINE_RENDERING_MODE_TUBES || lineRenderingMode == LINE_RENDERING_MODE_TUBES_CAPPED
            || lineRenderingMode == LINE_RENDERING_MODE_TUBES_UNION) {
        std::vector<glm::vec3> lineVertices;
        std::vector<glm::vec4> lineColors;
        mesh->getCompleteWireframeData(
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
        mesh->getCompleteWireframeTubeData(lineCentersList, lineColorsList);*/

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
                    mesh, lineWidth * 0.5f, 8, triangleIndices, vertexPositions,
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
        mesh->getVertexTubeData(
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
    } else if (lineRenderingMode == LINE_RENDERING_MODE_BILLBOARD_LINES) {
        std::vector<glm::vec3> lineVertices;
        std::vector<glm::vec4> lineColors;
        mesh->getCompleteWireframeData(
                lineVertices, lineColors, lineRenderingStyle == LINE_RENDERING_STYLE_TRON);

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

void ClearViewRenderer::setSortingAlgorithmDefine() {
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

void ClearViewRenderer::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void ClearViewRenderer::renderGui() {
    if (ImGui::Begin(windowName.c_str(), &showRendererWindow)) {
        childClassRenderGuiBegin();
        if (!useScreenSpaceLens
                && ImGui::SliderFloat("Focus Radius", &focusRadius, MIN_FOCUS_RADIUS, MAX_FOCUS_RADIUS)) {
            reloadSphereRenderData();
            reRender = true;
        }
        if (!useScreenSpaceLens && ImGui::SliderFloat3("Focus Point", &focusPoint.x, -0.4f, 0.4f)) {
            hasHitInformation = false;
            reRender = true;
        }
        if (!useScreenSpaceLens && ImGui::ColorEdit4("Focus Point Color", &focusPointColor.x)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            if (lineRenderingMode == LINE_RENDERING_MODE_TUBES || lineRenderingMode == LINE_RENDERING_MODE_TUBES_CAPPED
                    || lineRenderingMode == LINE_RENDERING_MODE_TUBES_UNION) {
                loadFocusRepresentation();
            }
            reRender = true;
        }
        if (clearViewRendererType == CLEAR_VIEW_RENDERER_TYPE_FACES
                && !useWeightedVertexAttributes && ImGui::Checkbox("Use Shading", &useShading)) {
            reRender = true;
        }
        if (clearViewRendererType != CLEAR_VIEW_RENDERER_TYPE_VOLUME
                && ImGui::Checkbox("Use Weighted Vertex Attributes", &useWeightedVertexAttributes)) {
            useShading = false;
            if (this->mesh) generateVisualizationMapping(mesh, false);
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
        childClassRenderGuiEnd();
    }
    ImGui::End();
}

void ClearViewRenderer::update(float dt) {
    if (!useScreenSpaceLens) {
        if (sgl::Keyboard->getModifier() & KMOD_SHIFT) {
            if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
                float scrollAmount = sgl::Mouse->getScrollWheel() * dt * 2.0;
                focusRadius += scrollAmount;
                focusRadius = glm::clamp(focusRadius, 0.001f, 0.4f);
                reRender = true;
            }
        }

        Pickable::updatePickable(dt, reRender, sceneData);
    } else {
        if (sgl::Keyboard->getModifier() & KMOD_SHIFT) {
            if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
                float scrollAmount = sgl::Mouse->getScrollWheel() * dt * 800.0;
                screenSpaceLensPixelRadius += scrollAmount;
                sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
                int width = window->getWidth();
                int height = window->getHeight();
                screenSpaceLensPixelRadius = glm::min(screenSpaceLensPixelRadius, float(std::max(width, height)));
                reRender = true;
            }
        }

        if (sgl::Keyboard->getModifier() & KMOD_CTRL) {
            if (sgl::Mouse->buttonPressed(1) || (sgl::Mouse->isButtonDown(1) && sgl::Mouse->mouseMoved())) {
                sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
                int mouseX = sgl::Mouse->getX();
                int mouseY = sgl::Mouse->getY();
                focusPointScreen = glm::vec2(mouseX, window->getHeight() - mouseY - 1);
                reRender = true;
            }
        }
    }
}
