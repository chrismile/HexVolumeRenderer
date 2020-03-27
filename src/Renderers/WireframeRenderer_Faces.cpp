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

#include "WireframeRenderer_Faces.hpp"

WireframeRenderer_Faces::WireframeRenderer_Faces(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    shaderProgram = sgl::ShaderManager->getShaderProgram(
            {"WireframeSurface.Vertex", "WireframeSurface.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");
}

void WireframeRenderer_Faces::generateVisualizationMapping(HexMeshPtr meshIn) {
    std::vector<uint32_t> indices;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec4> colors;
    std::vector<glm::vec3> barycentricCoordinates;
    meshIn->getSurfaceDataBarycentric(indices, vertices, colors, barycentricCoordinates);

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgram);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

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

    // Add the normal buffer.
    sgl::GeometryBufferPtr barycentricCoordinatesBuffer = sgl::Renderer->createGeometryBuffer(
            barycentricCoordinates.size()*sizeof(glm::vec3),
            (void*)&barycentricCoordinates.front(), sgl::VERTEX_BUFFER);
    shaderAttributes->addGeometryBuffer(
            barycentricCoordinatesBuffer, "vertexBarycentricCoordinates",
            sgl::ATTRIB_FLOAT, 3);

    dirty = false;
    reRender = true;
}

void WireframeRenderer_Faces::render() {
    if (shaderProgram->hasUniform("cameraPosition")) {
        shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    glDisable(GL_CULL_FACE);
    sgl::Renderer->render(shaderAttributes);
    glEnable(GL_CULL_FACE);
}

void WireframeRenderer_Faces::renderGui() {
    ;
}
