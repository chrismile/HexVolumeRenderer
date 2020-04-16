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

#include "Helpers/Sphere.hpp"
#include "Helpers/LineRenderingDefines.hpp"
#include "LOD/LodSheetGeneration.hpp"
#include "LodLinePreviewRenderer_SheetsFaces.hpp"

LodLinePreviewRenderer_SheetsFaces::LodLinePreviewRenderer_SheetsFaces(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : PerPixelLinkedListRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->addPreprocessorDefine("LINE_RENDERING_STYLE_HALO", "");
    initShaders({"WireframeSurfaceSingular.Vertex", "WireframeSurfaceSingular.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("LINE_RENDERING_STYLE_HALO");
}

void LodLinePreviewRenderer_SheetsFaces::generateVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    std::vector<uint32_t> indices;
    std::vector<LodHexahedralCellFace> hexahedralCellFaces;
    generateSheetLevelOfDetailLineStructureAndVertexData(
            meshIn.get(), indices, hexahedralCellFaces,
            useVolumeAndAreaMeasures, useWeightsForMerging);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(LodHexahedralCellFace), (void*)&hexahedralCellFaces.front(),
            sgl::SHADER_STORAGE_BUFFER);

    /*std::vector<glm::vec3> vertices;
    std::vector<glm::vec4> colors;
    std::vector<float> lodValues;
    generateSheetLevelOfDetailLineStructureAndVertexData(meshIn.get(), vertices, colors, lodValues);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertices.size()*sizeof(glm::vec3), (void*)&vertices.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Add the color buffer.
    sgl::GeometryBufferPtr colorBuffer = sgl::Renderer->createGeometryBuffer(
            colors.size()*sizeof(glm::vec4), (void*)&colors.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            colorBuffer, "vertexColor", sgl::ATTRIB_FLOAT, 4);

    // Add the LOD value buffer.
    sgl::GeometryBufferPtr lodValueBuffer = sgl::Renderer->createGeometryBuffer(
            lodValues.size()*sizeof(float), (void*)&lodValues.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            lodValueBuffer, "vertexLodValue", sgl::ATTRIB_FLOAT, 1);*/

    dirty = false;
    reRender = true;
}

void LodLinePreviewRenderer_SheetsFaces::setUniformData(){
    PerPixelLinkedListRenderer::setUniformData();
    gatherShader->setUniform("maxLod", maxLod);
    gatherShader->setUniform("lineWidth", lineWidth);
    sgl::ShaderManager->bindShaderStorageBuffer(6, hexahedralCellFacesBuffer);
}

void LodLinePreviewRenderer_SheetsFaces::gather() {
    // Render the LOD lines.
    glDisable(GL_CULL_FACE);
    PerPixelLinkedListRenderer::gather();
    glEnable(GL_CULL_FACE);
}

void LodLinePreviewRenderer_SheetsFaces::renderGui() {
    if (ImGui::Begin("Line LOD Preview Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Maximum LOD", &maxLod, 0.0f, 1.0f)) {
            reRender = true;
        }
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::Checkbox("Use Volume and Area Measures", &useVolumeAndAreaMeasures)) {
            if (mesh) {
                generateVisualizationMapping(mesh, false);
                reRender = true;
            }
        }
        if (ImGui::Checkbox("Use Weights for Merging", &useWeightsForMerging)) {
            if (mesh) {
                generateVisualizationMapping(mesh, false);
                reRender = true;
            }
        }
    }
    ImGui::End();
}
