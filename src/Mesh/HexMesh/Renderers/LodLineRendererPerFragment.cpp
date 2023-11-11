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
#include <Graphics/Renderer.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "Mesh/HexMesh/Renderers/Helpers/Sphere.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "LodLineRendererPerFragment.hpp"

LodLineRendererPerFragment::LodLineRendererPerFragment(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    shaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeLod.Vertex", "WireframeLod.Geometry", "WireframeLod.Fragment"});

    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");

    reloadSphereRenderData();
}

void LodLineRendererPerFragment::reloadSphereRenderData() {
    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec3> sphereVertexNormals;
    std::vector<uint32_t> sphereIndices;
    float sphereRadius = glm::clamp(
            maxDistance * FOCUS_SPHERE_SIZE_FACTOR / 2.0f, MIN_FOCUS_SPHERE_RADIUS, MAX_FOCUS_SPHERE_RADIUS);
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

void LodLineRendererPerFragment::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    if (isNewMesh) {
        Pickable::focusPoint = glm::vec3(0.0f);
    }

    const float avgCellVolumeCbrt = std::cbrt(meshIn->getAverageCellVolume());
    lineWidth = glm::clamp(
            avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
    maxDistance = 2.0f * glm::clamp(
            avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
    reloadSphereRenderData();

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec4> colors;
    std::vector<float> lodValues;
    meshIn->getLodLineRepresentation(vertices, colors, lodValues, false);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertices.size()*sizeof(glm::vec3), vertices.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr colorBuffer = sgl::Renderer->createGeometryBuffer(
            colors.size()*sizeof(glm::vec4), colors.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            colorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);

    // Add the LOD value buffer.
    sgl::GeometryBufferPtr lodValueBuffer = sgl::Renderer->createGeometryBuffer(
            lodValues.size()*sizeof(float), lodValues.data(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            lodValueBuffer, "vertexLodValue", sgl::ATTRIB_FLOAT, 1);

    dirty = false;
    reRender = true;
}

void LodLineRendererPerFragment::render() {
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform("focusPoint", focusPoint);
    shaderProgram->setUniform("maxDistance", maxDistance);
    shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());

    // Render the LOD lines.
    shaderProgram->setUniform("lineWidth", lineWidth);
    sgl::Renderer->render(shaderAttributes);

    // Render the focus point.
    shaderProgramSurface->setUniform("color", focusPointColor);
    sgl::Renderer->setModelMatrix(sgl::matrixTranslation(focusPoint));
    sgl::Renderer->render(focusPointShaderAttributes);
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
}

void LodLineRendererPerFragment::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::SliderFloat("Maximum Distance", &maxDistance, 0.0f, 1.5f)) {
            reloadSphereRenderData();
            reRender = true;
        }
        if (ImGui::SliderFloat3("Focus Point", &focusPoint.x, -0.4f, 0.4f)) {
            reRender = true;
        }
        if (ImGui::ColorEdit4("Focus Point Color", &focusPointColor.x)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();
}

void LodLineRendererPerFragment::update(float dt) {
    Pickable::updatePickable(dt, reRender, sceneData);
}
