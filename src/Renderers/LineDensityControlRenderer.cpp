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
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "Helpers/LineRenderingDefines.hpp"
#include "LOD/LodSheetGeneration.hpp"
#include "LineDensityControlRenderer.hpp"

LineDensityControlRenderer::LineDensityControlRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    lineShaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeSurfaceSingular.Vertex", "WireframeSurfaceSingular.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");

}

void LineDensityControlRenderer::generateVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    std::vector<uint32_t> indices;
    std::vector<LodHexahedralCellFace> hexahedralCellFaces;
    generateSheetLevelOfDetailLineStructureAndVertexData(
            meshIn.get(), indices, hexahedralCellFaces);

    lineShaderAttributes = sgl::ShaderManager->createShaderAttributes(lineShaderProgram);
    lineShaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
    lineShaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(LodHexahedralCellFace), (void*)&hexahedralCellFaces.front(),
            sgl::SHADER_STORAGE_BUFFER);

    dirty = false;
    reRender = true;
}

void LineDensityControlRenderer::render() {
    /*if (lineShaderProgram->hasUniform("cameraPosition")) {
        lineShaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    lineShaderProgram->setUniform("lineWidth", lineWidth);
    sgl::Renderer->render(lineShaderAttributes);*/
}

void LineDensityControlRenderer::renderGui() {
    if (ImGui::Begin("Line Density Control Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"\u03BB", &lambda, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"m", &factor_m, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"c", &factor_c, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"v", &factor_v, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(u8"d", &factor_d, 0.0f, 1.0f, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();
}
