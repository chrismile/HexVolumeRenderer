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

#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>

#include "Mesh/BaseComplex/global_types.h"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"

#include "HexSheetRenderer.hpp"

HexSheetRenderer::HexSheetRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    shaderProgramHull = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain.PositiveDepthBias"});
    shaderProgramSheet = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Color", "MeshShader.Fragment"});
    shaderProgramWireframe = sgl::ShaderManager->getShaderProgram(
            {"WireframeSurface.Vertex", "WireframeSurface.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");
}

bool computeHexahedralSheetComponentNeighborship_B(
        HexMesh* hexMesh, SheetComponent& component0, SheetComponent& component1,
        float& matchingWeight, ComponentConnectionType& componentConnectionType) {
    SheetComponent mergedComponent;
    std::set_intersection(
            component0.cellIds.begin(), component0.cellIds.end(),
            component1.cellIds.begin(), component1.cellIds.end(),
            std::back_inserter(mergedComponent.cellIds));

    // Is intersecting (or hybrid), i.e. not parallel?
    bool isIntersecting = !mergedComponent.cellIds.empty();

    // Is the merged component exactly the original components?
    if (mergedComponent.cellIds.size() == component0.cellIds.size()
        && mergedComponent.cellIds.size() == component1.cellIds.size()) {
        return false;
    }

    setHexahedralSheetBoundaryFaceIds(hexMesh, mergedComponent);
    std::sort(mergedComponent.boundaryFaceIds.begin(), mergedComponent.boundaryFaceIds.end());

    std::vector<uint32_t> boundaryFaceIdsIntersection;
    std::set_intersection( // can't use set_union here
            component0.boundaryFaceIds.begin(), component0.boundaryFaceIds.end(),
            component1.boundaryFaceIds.begin(), component1.boundaryFaceIds.end(),
            std::back_inserter(boundaryFaceIdsIntersection));

    std::vector<uint32_t> boundaryFaceIdsNoLongerBoundaryAfterMerging;
    std::set_difference(
            boundaryFaceIdsIntersection.begin(), boundaryFaceIdsIntersection.end(),
            mergedComponent.boundaryFaceIds.begin(), mergedComponent.boundaryFaceIds.end(),
            std::back_inserter(boundaryFaceIdsNoLongerBoundaryAfterMerging));

    bool isHybrid = isIntersecting && !boundaryFaceIdsNoLongerBoundaryAfterMerging.empty();

    float percentageOfAdjacency = float(boundaryFaceIdsNoLongerBoundaryAfterMerging.size())
                                  / float(component0.boundaryFaceIds.size() + component1.boundaryFaceIds.size());
    matchingWeight = percentageOfAdjacency / float(component0.cellIds.size() + component1.cellIds.size());

    if (isIntersecting) {
        // Add a delta so that intersecting components without shared boundary faces that would no longer be boundary faces
        // after merging may also be matched (even though with a much lower priority).
        matchingWeight = std::max(matchingWeight, 1e-6f);
    }

    if (!isIntersecting) {
        componentConnectionType = ComponentConnectionType::ADJACENT;
    } else if (isHybrid) {
        componentConnectionType = ComponentConnectionType::HYBRID;
    } else {
        componentConnectionType = ComponentConnectionType::INTERSECTING;
    }

    return !boundaryFaceIdsNoLongerBoundaryAfterMerging.empty() || !mergedComponent.cellIds.empty();
}

void computeHexahedralSheetComponentConnectionData_B(
        HexMesh* hexMesh, std::vector<SheetComponent*>& components,
        std::vector<ComponentConnectionData>& connectionDataList) {
    for (size_t i = 0; i < components.size(); i++) {
        for (size_t j = i + 1; j < components.size(); j++) {
            SheetComponent& component0 = *components.at(i);
            SheetComponent& component1 = *components.at(j);
            ComponentConnectionType componentConnectionType;
            float edgeWeight = 1.0f;
            bool componentsAreNeighbors = computeHexahedralSheetComponentNeighborship_B(
                    hexMesh, component0, component1, edgeWeight, componentConnectionType);
            if (!componentsAreNeighbors) {
                continue;
            }

            component0.neighborIndices.insert(j);
            component1.neighborIndices.insert(i);

            ComponentConnectionData connectionData;
            connectionData.firstIdx = i;
            connectionData.secondIdx = j;
            connectionData.componentConnectionType = componentConnectionType;
            connectionData.weight = edgeWeight;
            connectionDataList.push_back(connectionData);
        }
    }
}

void HexSheetRenderer::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    hexMesh = meshIn;
    hexahedralSheets.clear();
    connectionDataSet.clear();
    currentSheetConnectionDataSet.clear();
    selectedSheetIndex0 = -1;
    selectedSheetIndex1 = -1;
    selectedRow = -1;
    currentSheetConnectionDataSet.clear();
    shaderAttributesSheet = sgl::ShaderAttributesPtr();

    extractAllHexahedralSheets(meshIn.get(), hexahedralSheets);

    // Every sheet belongs to its own component.
    std::vector<SheetComponent*> components;
    components.reserve(hexahedralSheets.size());
    for (size_t i = 0; i < hexahedralSheets.size(); i++) {
        SheetComponent* component = new SheetComponent;
        components.push_back(component);
        HexahedralSheet& sheet = hexahedralSheets.at(i);
        std::sort(sheet.cellIds.begin(), sheet.cellIds.end());
        std::sort(sheet.boundaryFaceIds.begin(), sheet.boundaryFaceIds.end());
        component->cellIds = sheet.cellIds;
        component->boundaryFaceIds = sheet.boundaryFaceIds;
    }

    std::vector<ComponentConnectionData> connectionDataList;
    computeHexahedralSheetComponentConnectionData_B(
            meshIn.get(), components, connectionDataList);
    for (ComponentConnectionData& componentConnectionData : connectionDataList) {
        connectionDataSet.insert(componentConnectionData);
    }

    for (SheetComponent* component : components) {
        delete component;
    }
    components.clear();

    // Get hull data.
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    meshIn->getSurfaceData(triangleIndices, vertexPositions, vertexNormals, vertexAttributes);

    shaderAttributesHull = sgl::ShaderManager->createShaderAttributes(shaderProgramHull);
    shaderAttributesHull->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributesHull->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributesHull->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);
    shaderAttributesHull->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);


    // Set wireframe data.
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    dirty = false;
    reRender = true;
}

void HexSheetRenderer::recreateRenderingData() {
    if (selectedSheetIndex0 == -1) {
        shaderAttributesSheet = sgl::ShaderAttributesPtr();
        return;
    }

    Mesh& mesh = hexMesh->getBaseComplexMesh();
    std::vector<uint32_t> wireframeFaceIds;

    std::vector<uint32_t> selectedSheet0CellList;
    std::vector<uint32_t> selectedSheet1CellList;
    if (selectedSheetIndex0 != -1) {
        HexahedralSheet& selectedSheet0 = hexahedralSheets.at(selectedSheetIndex0);
        selectedSheet0CellList = selectedSheet0.cellIds;
    }
    if (selectedSheetIndex1 != -1) {
        HexahedralSheet& selectedSheet1 = hexahedralSheets.at(selectedSheetIndex1);
        selectedSheet1CellList = selectedSheet1.cellIds;
    }

    std::unordered_set<uint32_t> cellSheet0Set;
    for (uint32_t h_id : selectedSheet0CellList) {
        cellSheet0Set.insert(h_id);
    }

    std::vector<uint32_t> cellIntersectionList;
    std::unordered_set<uint32_t> cellIntersectionSet;
    std::set_intersection(
            selectedSheet0CellList.begin(), selectedSheet0CellList.end(),
            selectedSheet1CellList.begin(), selectedSheet1CellList.end(),
            std::back_inserter(cellIntersectionList));
    for (uint32_t h_id : cellIntersectionList) {
        cellIntersectionSet.insert(h_id);
    }

    std::vector<uint32_t> cellUnionList;
    std::unordered_set<uint32_t> cellUnionSet;
    std::set_union(
            selectedSheet0CellList.begin(), selectedSheet0CellList.end(),
            selectedSheet1CellList.begin(), selectedSheet1CellList.end(),
            std::back_inserter(cellUnionList));
    for (uint32_t h_id : cellUnionList) {
        cellUnionSet.insert(h_id);
    }

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec4> vertexColors;
    size_t indexOffset = 0;
    for (uint32_t h_id : cellUnionList) {
        Hybrid& h = mesh.Hs.at(h_id);

        glm::vec4 color;
        if (cellIntersectionSet.find(h_id) != cellIntersectionSet.end()) {
            color = sheetIntersectionColor;
        } else if (cellSheet0Set.find(h_id) != cellSheet0Set.end()) {
            color = sheet0Color;
        } else  {
            color = sheet1Color;
        }

        for (uint32_t f_id : h.fs) {
            Hybrid_F& f = mesh.Fs.at(f_id);
            if (std::all_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [this](uint32_t h_id) {
                return hexMesh->isCellMarked(h_id);
            })) {
                continue;
            }

            if (f.neighbor_hs.size() == 1 || std::any_of(f.neighbor_hs.begin(), f.neighbor_hs.end(), [&](uint32_t h_id) {
                return cellUnionSet.find(h_id) == cellUnionSet.end();
            })) {
                wireframeFaceIds.push_back(f_id);
            }

            assert(f.neighbor_hs.size() >= 1 && f.neighbor_hs.size() <= 2);
            bool invertWinding = f.neighbor_hs.at(0) != h.id;

            assert(f.vs.size() == 4);
            for (size_t j = 0; j < 4; j++) {
                uint32_t v_id = f.vs.at(j);
                glm::vec3 vertexPosition(mesh.V(0, v_id), mesh.V(1, v_id), mesh.V(2, v_id));
                vertexPositions.push_back(vertexPosition);
                vertexColors.push_back(color);
            }

            size_t oldTriangleIndicesSize = triangleIndices.size();
            if (!invertWinding) {
                /**
                 * vertex 1     edge 1    vertex 2
                 *          | - - - - - |
                 *          | \         |
                 *          |   \       |
                 *   edge 0 |     \     | edge 2
                 *          |       \   |
                 *          |         \ |
                 *          | - - - - - |
                 * vertex 0     edge 3    vertex 3
                 */
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
            }
            if (invertWinding) {
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 0);
                triangleIndices.push_back(indexOffset + 3);
                triangleIndices.push_back(indexOffset + 1);
                triangleIndices.push_back(indexOffset + 2);
            }

            for (size_t j = 0; j < 2; j++) {
                glm::vec3 v[3];
                for (int k = 0; k < 3; k++) {
                    v[k] = vertexPositions.at(triangleIndices.at(oldTriangleIndicesSize + j*3 + k));
                }
                glm::vec3 vertexNormal = glm::normalize(glm::cross(v[1] - v[0], v[2] - v[0]));
                vertexNormals.push_back(vertexNormal);
                vertexNormals.push_back(vertexNormal);
            }

            indexOffset += 4;
        }
    }

    shaderAttributesSheet = sgl::ShaderManager->createShaderAttributes(shaderProgramSheet);
    shaderAttributesSheet->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributesSheet->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSheet->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the normal buffer.
    sgl::GeometryBufferPtr normalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSheet->addGeometryBuffer(
            normalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr colorBuffer = sgl::Renderer->createGeometryBuffer(
            vertexColors.size()*sizeof(glm::vec4), vertexColors.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSheet->addGeometryBuffer(
            colorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);


    std::vector<uint32_t> indices;
    std::vector<HexahedralCellFace> hexahedralCellFaces;
    hexMesh->getSurfaceDataWireframeFaces(
            indices, hexahedralCellFaces, wireframeFaceIds, false);

    shaderAttributesWireframe = sgl::ShaderManager->createShaderAttributes(shaderProgramWireframe);
    shaderAttributesWireframe->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBufferWireframe = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), indices.data(), sgl::INDEX_BUFFER);
    shaderAttributesWireframe->setIndexGeometryBuffer(indexBufferWireframe, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    wireframeFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(HexahedralCellFace), hexahedralCellFaces.data(),
            sgl::SHADER_STORAGE_BUFFER);

}

void HexSheetRenderer::render() {
    if (shaderAttributesSheet) {
        shaderProgramSheet->setUniform("useShading", int(useShading));
        shaderProgramSheet->setUniform("cameraPosition", sceneData.camera->getPosition());
        sgl::Renderer->render(shaderAttributesSheet);

        shaderProgramWireframe->setUniform("cameraPosition", sceneData.camera->getPosition());
        shaderProgramWireframe->setUniform("lineWidth", lineWidth);
        sgl::ShaderManager->bindShaderStorageBuffer(6, wireframeFacesBuffer);
        glDisable(GL_CULL_FACE);
        sgl::Renderer->render(shaderAttributesWireframe);
        glEnable(GL_CULL_FACE);
    }

    shaderProgramHull->setUniform("useShading", int(useShading));
    shaderProgramHull->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgramHull->setUniform("color", glm::vec4(hullColor.r, hullColor.g, hullColor.b, hullOpacity));
    glDepthMask(GL_FALSE);
    sgl::Renderer->render(shaderAttributesHull);
    glDepthMask(GL_TRUE);
}

void HexSheetRenderer::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::Checkbox("Use Shading", &useShading)) {
            reRender = true;
        }

        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }

        if (ImGui::SliderFloat("Hull Opacity", &hullOpacity, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }

        ImGui::Text("Number of Sheets: %u", uint32_t(hexahedralSheets.size()));
        if (ImGui::InputInt("First Sheet Index", &selectedSheetIndex0)) {
            selectedSheetIndex0 = glm::clamp(selectedSheetIndex0, 0, int(hexahedralSheets.size()) - 1);
            selectedSheetIndex1 = -1;
            selectedRow = -1;
            currentSheetConnectionDataSet.clear();
            for (const ComponentConnectionData& connectionData : connectionDataSet) {
                if (connectionData.firstIdx == selectedSheetIndex0 || connectionData.secondIdx == selectedSheetIndex0) {
                    currentSheetConnectionDataSet.insert(connectionData);
                }
            }
            recreateRenderingData();
            reRender = true;
        }

        if (ImGui::Checkbox("Show Connected Sheets", &showConnectedSheets)) {
            reRender = true;
        }

        if (showConnectedSheets) {
            ImGui::Columns(4, "columns");
            ImGui::Separator();
            ImGui::Text("ID"); ImGui::NextColumn();
            ImGui::Text("Sheet Index"); ImGui::NextColumn();
            ImGui::Text("Type"); ImGui::NextColumn();
            ImGui::Text("Weight"); ImGui::NextColumn();
            ImGui::Separator();
            int counter = 0;
            for (const ComponentConnectionData& connectionData : currentSheetConnectionDataSet) {
                char label[32];
                sprintf(label, "%04d", counter);
                int sheetIndex =
                        connectionData.firstIdx == selectedSheetIndex0
                        ? connectionData.secondIdx : connectionData.firstIdx;
                if (ImGui::Selectable(label, selectedRow == counter, ImGuiSelectableFlags_SpanAllColumns)) {
                    selectedRow = counter;
                    selectedSheetIndex1 = sheetIndex;
                    recreateRenderingData();
                    reRender = true;
                }
                ImGui::NextColumn();
                ImGui::Text("%d", sheetIndex); ImGui::NextColumn();
                ImGui::Text(connectionData.componentConnectionType == ComponentConnectionType::ADJACENT ? "Adjacent"
                        : connectionData.componentConnectionType == ComponentConnectionType::HYBRID
                        ? "Hybrid" : "Intersecting"); ImGui::NextColumn();
                ImGui::Text("%f", connectionData.weight); ImGui::NextColumn();

                counter++;
            }
            ImGui::Columns(1);
        }

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Color Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::ColorEdit3("Sheet 0 Color", &sheet0Color.r)) {
                recreateRenderingData();
                reRender = true;
            }
            if (ImGui::ColorEdit3("Sheet 1 Color", &sheet1Color.r)) {
                recreateRenderingData();
                reRender = true;
            }
            if (ImGui::ColorEdit3("Sheet Intersection", &sheetIntersectionColor.r)) {
                recreateRenderingData();
                reRender = true;
            }
            if (ImGui::ColorEdit3("Hull Color", &hullColor.r)) {
                reRender = true;
            }
        }
    }
    ImGui::End();
}
