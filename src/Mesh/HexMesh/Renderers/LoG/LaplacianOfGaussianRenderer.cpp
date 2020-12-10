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

#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>

#include "LaplacianOfGaussianRenderer.hpp"

void LaplacianOfGaussianRenderer::initializeLoG() {
    shaderFullScreenBlitLoG = sgl::ShaderManager->getShaderProgram(
            {"Mesh.Vertex.Plain", "Mesh.Fragment.Plain"});
    colorTextureShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"LaplacianOfGaussian.Vertex", "LaplacianOfGaussian.Fragment.ColorTexture"});
    depthTextureShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"LaplacianOfGaussian.Vertex", "LaplacianOfGaussian.Fragment.DepthTexture"});
    normalTextureShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"LaplacianOfGaussian.Vertex", "LaplacianOfGaussian.Fragment.NormalTexture"});
    depthNormalTextureShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"LaplacianOfGaussian.Vertex", "LaplacianOfGaussian.Fragment.DepthNormalTexture"});
    meshShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"MeshLoG.Vertex.Plain", "MeshLoG.Fragment.Plain"});
    meshShaderNormalLoG = sgl::ShaderManager->getShaderProgram(
            {"MeshLoG.Vertex.PlainNormal", "MeshLoG.Fragment.PlainNormal"});

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());

    shaderAttributesFullScreenBlitLoG = sgl::ShaderManager->createShaderAttributes(shaderFullScreenBlitLoG);
    shaderAttributesFullScreenBlitLoG->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    if (outlineMode == OUTLINE_MODE_DEPTH) {
        shaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(depthTextureShaderLoG);
    } else if (outlineMode == OUTLINE_MODE_DEPTH) {
        shaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(normalTextureShaderLoG);
    } else if (outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(depthNormalTextureShaderLoG);
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        shaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(colorTextureShaderLoG);
    }
    shaderAttributesLoG->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    createWeightTextureLoG();
}

void LaplacianOfGaussianRenderer::reloadTexturesLoG() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    imageTextureLoG = sgl::TexturePtr();
    normalTextureLoG = sgl::TexturePtr();
    depthStencilTextureLoG = sgl::TexturePtr();

    framebufferLoG = sgl::Renderer->createFBO();
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_NORMAL
            || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        depthStencilTextureLoG = sgl::TextureManager->createDepthStencilTexture(width, height, sgl::DEPTH24_STENCIL8);
        depthStencilTextureLoG->setDepthStencilComponentMode(sgl::DEPTH_STENCIL_TEXTURE_MODE_DEPTH_COMPONENT);
        framebufferLoG->bindTexture(depthStencilTextureLoG, sgl::DEPTH_STENCIL_ATTACHMENT);
    }

    if (outlineMode == OUTLINE_MODE_NORMAL || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        sgl::TextureSettings textureSettingsNormal;
        textureSettingsNormal.internalFormat = GL_RGB32F; // TODO: Compress.
        textureSettingsNormal.pixelType = GL_FLOAT;
        textureSettingsNormal.pixelFormat = GL_RGB;
        normalTextureLoG = sgl::TextureManager->createEmptyTexture(width, height, textureSettingsNormal);
        framebufferLoG->bindTexture(normalTextureLoG, sgl::COLOR_ATTACHMENT0);
    }

    if (outlineMode == OUTLINE_MODE_STENCIL) {
        sgl::TextureSettings textureSettings;
        textureSettings.internalFormat = GL_RGBA8;
        textureSettings.pixelType = GL_UNSIGNED_BYTE;
        textureSettings.pixelFormat = GL_RGB;
        imageTextureLoG = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
        framebufferLoG->bindTexture(imageTextureLoG);
        framebufferLoG->bindRenderbuffer(sceneDataLoG.sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    }
}

void LaplacianOfGaussianRenderer::reloadModelLoG(HexMeshPtr& mesh) {
    meshLoG = mesh;
    if (!mesh) {
        return;
    }

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    if (!useSlimMeshData) {
        mesh->getSurfaceData(triangleIndices, vertexPositions);
    } else {
        mesh->getSurfaceData_Slim(triangleIndices, vertexPositions);
    }

    // Create the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), (void*)&triangleIndices.front(), sgl::INDEX_BUFFER);

    // Create the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);

    meshShaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(meshShaderLoG);
    meshShaderAttributesLoG->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    meshShaderAttributesLoG->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    meshShaderAttributesLoG->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    meshShaderNormalAttributesLoG = sgl::ShaderManager->createShaderAttributes(meshShaderNormalLoG);
    meshShaderNormalAttributesLoG->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    meshShaderNormalAttributesLoG->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    meshShaderNormalAttributesLoG->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
}

void LaplacianOfGaussianRenderer::createWeightTextureLoG() {
    float* textureData = new float[weightTextureSize.x * weightTextureSize.y];
    const float FACTOR_1 = -1.0f / (sgl::PI * std::pow(rhoLoG, 4.0f));
    const float FACTOR_2 = -1.0f / (2.0f * rhoLoG * rhoLoG);
    // Compute the LoG kernel weights.
    for (int iy = 0; iy < weightTextureSize.y; iy++) {
        for (int ix = 0; ix < weightTextureSize.x; ix++) {
            int x = ix - weightTextureSize.x / 2;
            int y = iy - weightTextureSize.y / 2;
            // https://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm
            float term3 = (x*x + y*y) * FACTOR_2;
            textureData[ix + iy*weightTextureSize.x] = FACTOR_1 * (1.0f + term3) * std::exp(term3);
        }
    }
    // Normalize the kernel weights.
    const int numEntries = weightTextureSize.x * weightTextureSize.y;
    float positiveEntriesAbsoluteSum = 0.0f, negativeEntriesAbsoluteSum = 0.0f;
    for (int i = 0; i < numEntries; i++) {
        float entry = textureData[i];
        if (entry >= 0.0f) {
            positiveEntriesAbsoluteSum += entry;
        } else {
            negativeEntriesAbsoluteSum += -entry;
        }
    }
    for (int i = 0; i < numEntries; i++) {
        float entry = textureData[i];
        if (entry >= 0.0f) {
            textureData[i] /= positiveEntriesAbsoluteSum;
        } else {
            textureData[i] /= negativeEntriesAbsoluteSum;
        }
    }
    /*for (int iy = 0; iy < weightTextureSize.y; iy++) {
        for (int ix = 0; ix < weightTextureSize.x; ix++) {
            std::cout << textureData[ix + iy*weightTextureSize.x] << "\t";
        }
        std::cout << std::endl;
    }*/
    sgl::TextureSettings textureSettings;
    textureSettings.pixelType = GL_FLOAT;
    textureSettings.pixelFormat = GL_RED;
    textureSettings.internalFormat = GL_R32F;
    weightTextureLoG = sgl::TextureManager->createTexture(
            textureData, weightTextureSize.x, weightTextureSize.y, textureSettings);
    delete[] textureData;
}

void LaplacianOfGaussianRenderer::setUniformDataLoG() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    sgl::ShaderProgram* shaderProgramLoG = shaderAttributesLoG->getShaderProgram();

    shaderFullScreenBlitLoG->setUniform("color", sgl::Color(255, 255, 255));
    shaderProgramLoG->setUniform("clearColor", sceneDataLoG.clearColor);
    if (shaderProgramLoG->hasUniform("weightTexture")) {
        shaderProgramLoG->setUniform("weightTexture", weightTextureLoG, 2);
    }
    shaderProgramLoG->setUniform("weightTextureSize", glm::ivec2(weightTextureSize.x, weightTextureSize.y));
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_NORMAL
            || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderProgramLoG->setUniform("viewportSize", glm::ivec2(width, height));
    }
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderProgramLoG->setUniform("depthTexture", depthStencilTextureLoG, 3);
        shaderProgramLoG->setUniform("zNear", sceneDataLoG.camera->getNearClipDistance());
        shaderProgramLoG->setUniform("zFar", sceneDataLoG.camera->getFarClipDistance());
    }
    if (outlineMode == OUTLINE_MODE_NORMAL || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderProgramLoG->setUniform("normalTexture", normalTextureLoG, 4);
    }
    if (outlineMode == OUTLINE_MODE_STENCIL) {
        shaderProgramLoG->setUniform("imageTexture", imageTextureLoG, 3);
        shaderProgramLoG->setUniform("imageTextureSize", glm::ivec2(width, height));
    }
}

void LaplacianOfGaussianRenderer::renderLoGContours() {
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_NORMAL
            || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        sgl::Renderer->setProjectionMatrix(sceneDataLoG.camera->getProjectionMatrix());
        sgl::Renderer->setViewMatrix(sceneDataLoG.camera->getViewMatrix());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        sgl::Renderer->bindFBO(framebufferLoG);
        if (outlineMode != OUTLINE_MODE_NORMAL && outlineMode != OUTLINE_MODE_DEPTH_NORMAL) {
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        }
        glDisable(GL_STENCIL_TEST);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        //glDisable(GL_CULL_FACE);
        unsigned int bits = GL_DEPTH_BUFFER_BIT;
        if (outlineMode == OUTLINE_MODE_NORMAL || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
            // TODO
            bits = bits | GL_COLOR_BUFFER_BIT;
        }
        sgl::Renderer->clearFramebuffer(bits, sgl::Color(255, 255, 255));
        glDisable(GL_BLEND);
        if (outlineMode == OUTLINE_MODE_NORMAL || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
            sgl::Renderer->render(meshShaderNormalAttributesLoG);
        } else {
            sgl::Renderer->render(meshShaderAttributesLoG);
        }
        glEnable(GL_BLEND);

        sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
        sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        if (outlineMode != OUTLINE_MODE_NORMAL && outlineMode != OUTLINE_MODE_DEPTH_NORMAL) {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        }
        //glEnable(GL_CULL_FACE);
        sgl::Renderer->bindFBO(sceneDataLoG.framebuffer);
        sgl::Renderer->render(shaderAttributesLoG);
        glDisable(GL_DEPTH_TEST);
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        sgl::Renderer->bindFBO(framebufferLoG);
        sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT, sgl::Color(0, 0, 0));
        sgl::Renderer->render(shaderAttributesFullScreenBlitLoG);

        glDisable(GL_STENCIL_TEST);
        sgl::Renderer->bindFBO(sceneDataLoG.framebuffer);
        sgl::Renderer->render(shaderAttributesLoG);
        glEnable(GL_STENCIL_TEST);
    }
}

bool LaplacianOfGaussianRenderer::renderGuiLoG() {
    bool reRender = false;
    if (ImGui::Combo(
            "Outline Mode", (int*)&outlineMode, OUTLINE_MODE_NAMES, NUM_OUTLINE_MODES)) {
        if (outlineMode != OUTLINE_MODE_NONE) {
            if (outlineMode == OUTLINE_MODE_DEPTH) {
                shaderAttributesLoG = shaderAttributesLoG->copy(depthTextureShaderLoG);
            } else if (outlineMode == OUTLINE_MODE_NORMAL) {
                shaderAttributesLoG = shaderAttributesLoG->copy(normalTextureShaderLoG);
            } else if (outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
                shaderAttributesLoG = shaderAttributesLoG->copy(depthNormalTextureShaderLoG);
            } else if (outlineMode == OUTLINE_MODE_STENCIL) {
                shaderAttributesLoG = shaderAttributesLoG->copy(colorTextureShaderLoG);
            }
            reloadTexturesLoG();
            reloadModelLoG(meshLoG);
        }
        reRender = true;
    }
    return reRender;
}
