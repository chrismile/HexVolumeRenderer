/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2023, Christoph Neuhauser
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

#include <utility>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/SciVis/Navigation/CameraNavigator2D.hpp>
#include <Input/Mouse.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Scene/RenderTarget.hpp>
#include <Graphics/Scene/Camera.hpp>
#include <Graphics/OpenGL/Texture.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>

#include "DataView.hpp"

DataView::DataView(
        sgl::CameraPtr camera, bool& screenshotTransparentBackground, bool& useLinearRGB,
        sgl::ShaderProgramPtr gammaCorrectionShader)
        : camera(std::move(camera)), screenshotTransparentBackground(screenshotTransparentBackground),
          useLinearRGB(useLinearRGB), gammaCorrectionShader(std::move(gammaCorrectionShader)) {
}

DataView::~DataView() {
}

void DataView::resize(int newWidth, int newHeight) {
    viewportWidth = uint32_t(std::max(newWidth, 0));
    viewportHeight = uint32_t(std::max(newHeight, 0));

    if (viewportWidth == 0 || viewportHeight == 0) {
        sceneFramebuffer = {};
        sceneTexture = {};
        resolvedSceneFramebuffer = {};
        resolvedSceneTexture = {};
        sceneDepthRBO = {};
        return;
    }

    // Buffers for off-screen rendering.
    sceneFramebuffer = sgl::Renderer->createFBO();
    resolvedSceneFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA16;
    } else {
        textureSettings.internalFormat = GL_RGBA8;
    }
    sceneTexture = sgl::TextureManager->createEmptyTexture(int(viewportWidth), int(viewportHeight), textureSettings);
    sceneFramebuffer->bindTexture(sceneTexture);
    sceneDepthRBO = sgl::Renderer->createRBO(int(viewportWidth), int(viewportHeight), sceneDepthRBOType);
    sceneFramebuffer->bindRenderbuffer(sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    auto renderTarget = std::make_shared<sgl::RenderTarget>(sceneFramebuffer);
    camera->setRenderTarget(renderTarget, false);
    camera->onResolutionChanged({});

    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA8;
        resolvedSceneTexture = sgl::TextureManager->createEmptyTexture(
                int(viewportWidth), int(viewportHeight), textureSettings);
        resolvedSceneFramebuffer->bindTexture(resolvedSceneTexture);
    }
}

void DataView::beginRender() {
    //sgl::Renderer->bindFBO(sceneFramebuffer);
    sgl::Renderer->setCamera(camera, true);
    sgl::Renderer->clearFramebuffer(
            GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, clearColor);

    sgl::Renderer->setProjectionMatrix(camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glEnable(GL_DEPTH_TEST);
    if (!screenshotTransparentBackground) {
        glBlendEquation(GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
    }
}

void DataView::endRender() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    sgl::Renderer->bindFBO(resolvedSceneFramebuffer);
    if (useLinearRGB) {
        sgl::Renderer->blitTexture(
                sceneTexture, sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)),
                gammaCorrectionShader);
    }
    sgl::Renderer->unbindFBO();
}
