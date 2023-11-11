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

#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "WireframeRenderer_Faces.hpp"

WireframeRenderer_Faces::WireframeRenderer_Faces(
        SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow, bool useOutline, bool onlyBoundary)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow), onlyBoundary(onlyBoundary) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    if (useOutline) {
        sgl::ShaderManager->addPreprocessorDefine("LINE_RENDERING_STYLE_HALO", "");
    }
    shaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeSurface.Vertex", "WireframeSurface.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");
    if (useOutline) {
        sgl::ShaderManager->removePreprocessorDefine("LINE_RENDERING_STYLE_HALO");
    }
}

void WireframeRenderer_Faces::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    std::vector<uint32_t> indices;
    std::vector<HexahedralCellFace> hexahedralCellFaces;
    meshIn->getSurfaceDataWireframeFaces(indices, hexahedralCellFaces, onlyBoundary, false);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), indices.data(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(HexahedralCellFace), hexahedralCellFaces.data(),
            sgl::SHADER_STORAGE_BUFFER);

    dirty = false;
    reRender = true;
}

void WireframeRenderer_Faces::render() {
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform("lineWidth", lineWidth);
    sgl::ShaderManager->bindShaderStorageBuffer(6, hexahedralCellFacesBuffer);

    glDisable(GL_CULL_FACE);
    sgl::Renderer->render(shaderAttributes);
    glEnable(GL_CULL_FACE);
}

void WireframeRenderer_Faces::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(0, 200, 540, 110);
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();
}
