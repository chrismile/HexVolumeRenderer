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
#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "Mesh/HexMesh/Renderers/LOD/LodSheetGeneration.hpp"
#include "LineDensityControlRenderer.hpp"

/// Expected (average) depth complexity, i.e. width*height* this value = number of fragments that can be stored.
static int EXPECTED_DEPTH_COMPLEXITY = 90;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNodeAttributeTextures {
    float importanceAttribute; ///< Between 0 and 1.
    float depth; ///< The linear depth of the fragment (i.e., distance to the camera).
    uint32_t directionQuantized; ///< First 16 bits encode x, second 16 bits encode y.
    uint32_t next; ///< Next entry index in the per-pixel linked list (or -1 == 0xFFFFFFFFu).
};

LineDensityControlRenderer::LineDensityControlRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : HexahedralMeshRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    sgl::ShaderManager->addPreprocessorDefine("LINE_RENDERING_STYLE_HALO", "");
    lineDensityControlShader = sgl::ShaderManager->getShaderProgram(
            {"WireframeLineDensityControl.Vertex", "WireframeLineDensityControl.Fragment"});
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");
    sgl::ShaderManager->removePreprocessorDefine("LINE_RENDERING_STYLE_HALO");

    createAttributeTextureClearShader = sgl::ShaderManager->getShaderProgram(
            {"CreateAttributeTextureClear.Vertex", "CreateAttributeTextureClear.Fragment"});
    createAttributeTextureGatherShader = sgl::ShaderManager->getShaderProgram(
            {"CreateAttributeTextureGather.Vertex", "CreateAttributeTextureGather.Fragment"});
    createAttributeTextureResolveShader = sgl::ShaderManager->getShaderProgram(
            {"CreateAttributeTextureResolve.Vertex", "CreateAttributeTextureResolve.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates)
    createAttributeTextureResolveRenderData =
            sgl::ShaderManager->createShaderAttributes(createAttributeTextureResolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    createAttributeTextureResolveRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    createAttributeTextureClearRenderData =
            sgl::ShaderManager->createShaderAttributes(createAttributeTextureClearShader);
    geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    createAttributeTextureClearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    // Create render data for blurring.
    blurShader = sgl::ShaderManager->getShaderProgram(
            {"GaussianBlurFloatTexture.Vertex", "GaussianBlurFloatTexture.Fragment"});
    blurRenderData = sgl::ShaderManager->createShaderAttributes(blurShader);

    // Set-up the vertex data of the rectangle
    std::vector<sgl::VertexTextured> fullscreenTexturedQuad(sgl::Renderer->createTexturedQuad(
            sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f))));

    // Create the blur render data.
    int stride = sizeof(sgl::VertexTextured);
    sgl::GeometryBufferPtr geomBufferTextured = sgl::Renderer->createGeometryBuffer(
            sizeof(sgl::VertexTextured) * fullscreenTexturedQuad.size(), fullscreenTexturedQuad.data());
    blurRenderData = sgl::ShaderManager->createShaderAttributes(blurShader);
    blurRenderData->addGeometryBuffer(
            geomBufferTextured, "vertexPosition",
            sgl::ATTRIB_FLOAT, 3, 0, stride);
    blurRenderData->addGeometryBuffer(
            geomBufferTextured, "vertexTextureCoordinates",
            sgl::ATTRIB_FLOAT, 2, sizeof(glm::vec3), stride);

    onResolutionChanged();
}

void LineDensityControlRenderer::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();
    attributeTextureResolution.x = std::round(width * attributeTextureSubsamplingFactor);
    attributeTextureResolution.y = std::round(height * attributeTextureSubsamplingFactor);

    fragmentBufferSize = size_t(EXPECTED_DEPTH_COMPLEXITY)
            * size_t(attributeTextureResolution.x) * size_t(attributeTextureResolution.y);
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNodeAttributeTextures) * fragmentBufferSize;
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeBytes = (1ull << 32ull) - sizeof(LinkedListFragmentNodeAttributeTextures);
        fragmentBufferSize = fragmentBufferSizeBytes / sizeof(LinkedListFragmentNodeAttributeTextures);
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));
    }

    createAttributeTextureFragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    createAttributeTextureFragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * attributeTextureResolution.x * attributeTextureResolution.y;
    createAttributeTextureStartOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    createAttributeTextureStartOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    createAttributeTextureAtomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    createAttributeTextureAtomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);


    // Create the attribute texture and the framebuffer for using it as a render target.
    attributeTextureFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    textureSettings.internalFormat = GL_RGBA32F;
    textureSettings.textureMinFilter = GL_LINEAR;
    textureSettings.textureMagFilter = GL_LINEAR;
    attributeTexture = sgl::TextureManager->createEmptyTexture(
            attributeTextureResolution.x, attributeTextureResolution.y, textureSettings);
    attributeTextureFramebuffer->bindTexture(attributeTexture);


    // Create a framebuffer and temporary texture for blurring. The texture is bound during rendering.
    blurFramebuffer = sgl::Renderer->createFBO();
    tempBlurTexture = sgl::TextureManager->createEmptyTexture(
            attributeTextureResolution.x, attributeTextureResolution.y, textureSettings);
}

void LineDensityControlRenderer::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    lineWidth = glm::clamp(
            std::cbrt(meshIn->getAverageCellVolume()) * LINE_WIDTH_VOLUME_CBRT_FACTOR,
            MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);

    std::vector<uint32_t> triangleIndices;
    std::vector<HexahedralCellFaceLineDensityControl> hexahedralCellFaces;
    meshIn->getSurfaceDataWireframeFacesLineDensityControl(
            triangleIndices, hexahedralCellFaces, maxLodValue);

    lineDensityControlRenderData = sgl::ShaderManager->createShaderAttributes(lineDensityControlShader);
    lineDensityControlRenderData->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    createAttributeTextureGatherRenderData = sgl::ShaderManager->createShaderAttributes(
            createAttributeTextureGatherShader);
    createAttributeTextureGatherRenderData->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);
    lineDensityControlRenderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    createAttributeTextureGatherRenderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(HexahedralCellFaceLineDensityControl),
            hexahedralCellFaces.data(), sgl::SHADER_STORAGE_BUFFER);

    singularEdgeColorMapWidget.generateSingularityStructureInformation(meshIn);

    dirty = false;
    reRender = true;
}

void LineDensityControlRenderer::setUniformData() {
    sgl::ShaderManager->bindShaderStorageBuffer(0, createAttributeTextureFragmentBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, createAttributeTextureStartOffsetBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, createAttributeTextureAtomicCounterBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(6, hexahedralCellFacesBuffer);

    createAttributeTextureGatherShader->setUniform("viewportW", attributeTextureResolution.x);
    createAttributeTextureGatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    if (createAttributeTextureGatherShader->hasUniform("cameraPosition")) {
        createAttributeTextureGatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    createAttributeTextureGatherShader->setUniform("lineWidth", lineWidth);

    createAttributeTextureResolveShader->setUniform("viewportW", attributeTextureResolution.x);
    if (createAttributeTextureResolveShader->hasUniform("zNear")) {
        createAttributeTextureResolveShader->setUniform("zNear", sceneData.camera->getNearClipDistance());
    }
    if (createAttributeTextureResolveShader->hasUniform("zFar")) {
        createAttributeTextureResolveShader->setUniform("zFar", sceneData.camera->getFarClipDistance());
    }

    createAttributeTextureClearShader->setUniform("viewportW", attributeTextureResolution.x);

    if (lineDensityControlShader->hasUniform("cameraPosition")) {
        lineDensityControlShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    lineDensityControlShader->setUniform("lineWidth", lineWidth);
    lineDensityControlShader->setUniform("lambda", lambda);
    lineDensityControlShader->setUniform("factor_m", factor_m);
    lineDensityControlShader->setUniform("factor_c", factor_c);
    lineDensityControlShader->setUniform("factor_v", factor_v);
    lineDensityControlShader->setUniform("factor_d", factor_d);
    lineDensityControlShader->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    lineDensityControlShader->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    lineDensityControlShader->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    lineDensityControlShader->setUniform(
            "singularEdgeColorLookupTexture",
            singularEdgeColorMapWidget.getSingularEdgeColorLookupTexture(), 1);
    lineDensityControlShader->setUniform(
            "attributeTexture", attributeTexture, 2);
}


void LineDensityControlRenderer::attributeTextureClear() {
    glViewport(0, 0, attributeTextureResolution.x, attributeTextureResolution.y);

    // In the clear and gather pass, we just want to write data to an SSBO.
    sgl::Renderer->bindFBO(attributeTextureFramebuffer);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDisable(GL_BLEND);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(createAttributeTextureClearRenderData);

    // Set atomic counter to zero.
    GLuint bufferID = static_cast<sgl::GeometryBufferGL*>(createAttributeTextureAtomicCounterBuffer.get())->getBuffer();
    GLubyte val = 0;
    glClearNamedBufferData(bufferID, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void LineDensityControlRenderer::attributeTextureGather() {
    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    glDisable(GL_CULL_FACE);
    sgl::Renderer->render(createAttributeTextureGatherRenderData);
    glEnable(GL_CULL_FACE);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void LineDensityControlRenderer::attributeTextureResolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    sgl::Renderer->render(createAttributeTextureResolveRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void LineDensityControlRenderer::attributeTextureBlur() {
    blurShader->setUniform("textureSize", glm::vec2(this->attributeTextureResolution));

    // Perform a horizontal and a vertical blur.
    blurFramebuffer->bindTexture(tempBlurTexture);
    sgl::Renderer->bindFBO(blurFramebuffer);
    blurShader->setUniform("textureImage", attributeTexture, 3);
    blurShader->setUniform("horizontalBlur", true);
    sgl::Renderer->render(blurRenderData);

    blurFramebuffer->bindTexture(attributeTexture);
    sgl::Renderer->bindFBO(blurFramebuffer, true);
    blurShader->setUniform("textureImage", tempBlurTexture, 3);
    blurShader->setUniform("horizontalBlur", false);
    sgl::Renderer->render(blurRenderData);
}

void LineDensityControlRenderer::lineDensityControlRendering() {
    glViewport(0, 0, (*sceneData.sceneTexture)->getW(), (*sceneData.sceneTexture)->getH());

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Enable depth test and depth write.
    sgl::Renderer->bindFBO(*sceneData.framebuffer);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glEnable(GL_BLEND);

    glDisable(GL_CULL_FACE);
    sgl::Renderer->render(lineDensityControlRenderData);
    glEnable(GL_CULL_FACE);

    glDisable(GL_DEPTH_TEST);
}

void LineDensityControlRenderer::render() {
    setUniformData();
    attributeTextureClear();
    attributeTextureGather();
    attributeTextureResolve();
    attributeTextureBlur();
    lineDensityControlRendering();
}

void LineDensityControlRenderer::renderGui() {
    if (ImGui::Begin(getWindowName(), &showRendererWindow)) {
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(U8("\u03BB"), &lambda, 0.0f, 10.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(U8("m"), &factor_m, 0.0f, 50.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(U8("c"), &factor_c, 0.0f, 50.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(U8("v"), &factor_v, 0.0f, 50.0f, "%.4f")) {
            reRender = true;
        }
        if (ImGui::SliderFloat(U8("d"), &factor_d, 0.0f, 50.0f, "%.4f")) {
            reRender = true;
        }
    }
    ImGui::End();

    if (singularEdgeColorMapWidget.renderGui()) {
        reRender = true;
    }
}
