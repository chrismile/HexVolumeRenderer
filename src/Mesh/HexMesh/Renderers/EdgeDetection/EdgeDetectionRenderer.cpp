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

#include "EdgeDetectionRenderer.hpp"

void EdgeDetectionRenderer::initializeEdgeDetection() {
    weightTextureSize = glm::ivec2(5, 5);
    oldWeightTextureSize = weightTextureSize;

    noiseReduction.initialize(true);

    shaderFullScreenBlitEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"Mesh.Vertex.Plain", "Mesh.Fragment.Plain"});
    meshShaderEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"MeshEdgeDetection.Vertex.Plain", "MeshEdgeDetection.Fragment.Plain"});
    meshShaderNormalEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"MeshEdgeDetection.Vertex.PlainNormal", "MeshEdgeDetection.Fragment.PlainNormal"});
    directBlitShader = sgl::ShaderManager->getShaderProgram(
            {"BlitEdgeDetection.Vertex", "BlitEdgeDetection.Fragment"});
    reloadModelEdgeDetectionShader();

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());

    shaderAttributesFullScreenBlitEdgeDetection = sgl::ShaderManager->createShaderAttributes(
            shaderFullScreenBlitEdgeDetection);
    shaderAttributesFullScreenBlitEdgeDetection->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    directBlitShaderAttributes = sgl::ShaderManager->createShaderAttributes(directBlitShader);
    directBlitShaderAttributes->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    shaderAttributesEdgeDetection->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    createWeightTextureEdgeDetection();
}

void EdgeDetectionRenderer::reloadModelEdgeDetectionShader() {
    if (useNoiseReduction) {
        sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_OUTPUT", "");
    }

    sgl::ShaderManager->invalidateShaderCache();
    colorTextureShaderEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"EdgeDetection.Vertex", "EdgeDetection.Fragment.ColorTexture"});
    depthTextureShaderEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"EdgeDetection.Vertex", "EdgeDetection.Fragment.DepthTexture"});
    normalTextureShaderEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"EdgeDetection.Vertex", "EdgeDetection.Fragment.NormalTexture"});
    depthNormalTextureShaderEdgeDetection = sgl::ShaderManager->getShaderProgram(
            {"EdgeDetection.Vertex", "EdgeDetection.Fragment.DepthNormalTexture"});

    if (useNoiseReduction) {
        sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_OUTPUT");
    }

    if (shaderAttributesEdgeDetection) {
        if (outlineMode == OUTLINE_MODE_DEPTH) {
            shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                    depthTextureShaderEdgeDetection);
        } else if (outlineMode == OUTLINE_MODE_NORMAL) {
            shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                    normalTextureShaderEdgeDetection);
        } else if (outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
            shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                    depthNormalTextureShaderEdgeDetection);
        } else if (outlineMode == OUTLINE_MODE_STENCIL) {
            shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                    colorTextureShaderEdgeDetection);
        }
    } else {
        if (outlineMode == OUTLINE_MODE_DEPTH) {
            shaderAttributesEdgeDetection = sgl::ShaderManager->createShaderAttributes(
                    depthTextureShaderEdgeDetection);
        } else if (outlineMode == OUTLINE_MODE_NORMAL) {
            shaderAttributesEdgeDetection = sgl::ShaderManager->createShaderAttributes(
                    normalTextureShaderEdgeDetection);
        } else if (outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
            shaderAttributesEdgeDetection = sgl::ShaderManager->createShaderAttributes(
                    depthNormalTextureShaderEdgeDetection);
        } else if (outlineMode == OUTLINE_MODE_STENCIL) {
            shaderAttributesEdgeDetection = sgl::ShaderManager->createShaderAttributes(
                    colorTextureShaderEdgeDetection);
        }
    }
}

void EdgeDetectionRenderer::reloadTexturesEdgeDetection() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneDataEdgeDetection.sceneTexture)->getW();
    int height = (*sceneDataEdgeDetection.sceneTexture)->getH();

    imageTextureEdgeDetection = sgl::TexturePtr();
    normalTextureEdgeDetection = sgl::TexturePtr();
    depthStencilTextureEdgeDetection = sgl::TexturePtr();

    framebufferEdgeDetection = sgl::Renderer->createFBO();
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_NORMAL
            || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        depthStencilTextureEdgeDetection = sgl::TextureManager->createDepthStencilTexture(
                width, height, sgl::DEPTH24_STENCIL8);
        depthStencilTextureEdgeDetection->setDepthStencilComponentMode(
                sgl::DEPTH_STENCIL_TEXTURE_MODE_DEPTH_COMPONENT);
        framebufferEdgeDetection->bindTexture(
                depthStencilTextureEdgeDetection, sgl::DEPTH_STENCIL_ATTACHMENT);
    }

    if (outlineMode == OUTLINE_MODE_NORMAL || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        sgl::TextureSettings textureSettingsNormal;
        textureSettingsNormal.internalFormat = GL_RGB32F; // TODO: Compress.
        normalTextureEdgeDetection = sgl::TextureManager->createEmptyTexture(width, height, textureSettingsNormal);
        framebufferEdgeDetection->bindTexture(normalTextureEdgeDetection, sgl::COLOR_ATTACHMENT0);
    }

    if (outlineMode == OUTLINE_MODE_STENCIL) {
        sgl::TextureSettings textureSettings;
        textureSettings.internalFormat = GL_RGBA8;
        imageTextureEdgeDetection = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
        framebufferEdgeDetection->bindTexture(imageTextureEdgeDetection);
        framebufferEdgeDetection->bindRenderbuffer(
                *sceneDataEdgeDetection.sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    }

    noiseReduction.onResolutionChanged();
}

void EdgeDetectionRenderer::reloadModelEdgeDetection(HexMeshPtr& mesh) {
    meshEdgeDetection = mesh;
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
            sizeof(uint32_t)*triangleIndices.size(), triangleIndices.data(), sgl::INDEX_BUFFER);

    // Create the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);

    meshShaderAttributesEdgeDetection = sgl::ShaderManager->createShaderAttributes(meshShaderEdgeDetection);
    meshShaderAttributesEdgeDetection->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    meshShaderAttributesEdgeDetection->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    meshShaderAttributesEdgeDetection->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    meshShaderNormalAttributesEdgeDetection = sgl::ShaderManager->createShaderAttributes(
            meshShaderNormalEdgeDetection);
    meshShaderNormalAttributesEdgeDetection->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    meshShaderNormalAttributesEdgeDetection->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    meshShaderNormalAttributesEdgeDetection->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
}

void EdgeDetectionRenderer::createWeightTextureEdgeDetection() {
    float* textureData = new float[weightTextureSize.x * weightTextureSize.y];
    if (useLoG) {
        const float FACTOR_1 = -1.0f / (sgl::PI * std::pow(rhoEdgeDetection, 4.0f));
        const float FACTOR_2 = -1.0f / (2.0f * rhoEdgeDetection * rhoEdgeDetection);
        // Compute the EdgeDetection kernel weights.
        for (int iy = 0; iy < weightTextureSize.y; iy++) {
            for (int ix = 0; ix < weightTextureSize.x; ix++) {
                int x = ix - weightTextureSize.x / 2;
                int y = iy - weightTextureSize.y / 2;
                // https://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm
                float term3 = float(x*x + y*y) * FACTOR_2;
                textureData[ix + iy*weightTextureSize.x] = FACTOR_1 * (1.0f + term3) * std::exp(term3);
            }
        }
    } else {
        // Pre-defined isotropic Laplacian filter.
        // Cf. http://sepwww.stanford.edu/public/docs/sep95/sergey2/paper_html/node12.html
        float textureDataLaplacian[] = {
                0.25, 0.5, 0.25,
                0.5, -1., 0.5,
                0.25, 0.5, 0.25,
        };
        memcpy(textureData, textureDataLaplacian, sizeof(float) * 9);
        weightTextureSize = glm::ivec2(3, 3);
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
    positiveEntriesAbsoluteSum = std::max(positiveEntriesAbsoluteSum, 1e-7f);
    negativeEntriesAbsoluteSum = std::max(negativeEntriesAbsoluteSum, 1e-7f);
    for (int i = 0; i < numEntries; i++) {
        float entry = textureData[i];
        if (entry >= 0.0f) {
            textureData[i] /= positiveEntriesAbsoluteSum;
        } else {
            textureData[i] /= negativeEntriesAbsoluteSum;
        }
    }
    for (int iy = 0; iy < weightTextureSize.y; iy++) {
        for (int ix = 0; ix < weightTextureSize.x; ix++) {
            std::cout << textureData[ix + iy*weightTextureSize.x] << "\t";
        }
        std::cout << std::endl;
    }
    sgl::TextureSettings textureSettings;
    textureSettings.internalFormat = GL_R32F;
    weightTextureEdgeDetection = sgl::TextureManager->createTexture(
            textureData, weightTextureSize.x, weightTextureSize.y,
            sgl::PixelFormat(GL_RED, GL_FLOAT), textureSettings);
    delete[] textureData;
}

void EdgeDetectionRenderer::setUniformDataEdgeDetection() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneDataEdgeDetection.sceneTexture)->getW();
    int height = (*sceneDataEdgeDetection.sceneTexture)->getH();

    sgl::ShaderProgram* shaderProgramEdgeDetection = shaderAttributesEdgeDetection->getShaderProgram();

    shaderFullScreenBlitEdgeDetection->setUniform("color", sgl::Color(255, 255, 255));
    shaderProgramEdgeDetection->setUniformOptional("clearColor", sceneDataEdgeDetection.clearColor);
    if (shaderProgramEdgeDetection->hasUniform("weightTexture")) {
        shaderProgramEdgeDetection->setUniform("weightTexture", weightTextureEdgeDetection, 2);
    }
    shaderProgramEdgeDetection->setUniform(
            "weightTextureSize", glm::ivec2(weightTextureSize.x, weightTextureSize.y));
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_NORMAL
            || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderProgramEdgeDetection->setUniform(
                "viewportSize", glm::ivec2(width, height));
    }
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderProgramEdgeDetection->setUniform(
                "depthTexture", depthStencilTextureEdgeDetection, 3);
        shaderProgramEdgeDetection->setUniform(
                "zNear", sceneDataEdgeDetection.camera->getNearClipDistance());
        shaderProgramEdgeDetection->setUniform(
                "zFar", sceneDataEdgeDetection.camera->getFarClipDistance());
    }
    if (outlineMode == OUTLINE_MODE_NORMAL || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        shaderProgramEdgeDetection->setUniform(
                "normalTexture", normalTextureEdgeDetection, 4);
    }
    if (outlineMode == OUTLINE_MODE_STENCIL) {
        shaderProgramEdgeDetection->setUniform(
                "imageTexture", imageTextureEdgeDetection, 3);
        shaderProgramEdgeDetection->setUniform(
                "imageTextureSize", glm::ivec2(width, height));
    }
}

void EdgeDetectionRenderer::renderEdgeDetectionContours() {
    if (outlineMode == OUTLINE_MODE_DEPTH || outlineMode == OUTLINE_MODE_NORMAL
            || outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
        sgl::Renderer->setProjectionMatrix(sceneDataEdgeDetection.camera->getProjectionMatrix());
        sgl::Renderer->setViewMatrix(sceneDataEdgeDetection.camera->getViewMatrix());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        sgl::Renderer->bindFBO(framebufferEdgeDetection);
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
            sgl::Renderer->render(meshShaderNormalAttributesEdgeDetection);
        } else {
            sgl::Renderer->render(meshShaderAttributesEdgeDetection);
        }

        sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
        sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        if (outlineMode != OUTLINE_MODE_NORMAL && outlineMode != OUTLINE_MODE_DEPTH_NORMAL) {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        }
        //glEnable(GL_CULL_FACE);

        if (useNoiseReduction) {
            //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
            //int width = window->getWidth();
            //int height = window->getHeight();
            int width = (*sceneDataEdgeDetection.sceneTexture)->getW();
            int height = (*sceneDataEdgeDetection.sceneTexture)->getH();

            glDisable(GL_DEPTH_TEST);
            noiseReduction.bindFramebufferForRendering();
            sgl::Renderer->render(shaderAttributesEdgeDetection);
            noiseReduction.denoiseTexture();

            glEnable(GL_BLEND);
            sgl::Renderer->bindFBO(*sceneDataEdgeDetection.framebuffer);
            noiseReduction.getDenoisedTexture();
            directBlitShader->setUniform("clearColor", sceneDataEdgeDetection.clearColor);
            directBlitShader->setUniform("viewportSize", glm::ivec2(width, height));
            directBlitShader->setUniform("textureIn", noiseReduction.getDenoisedTexture(), 0);
            sgl::Renderer->render(directBlitShaderAttributes);
        } else {
            glEnable(GL_BLEND);
            sgl::Renderer->bindFBO(*sceneDataEdgeDetection.framebuffer);
            sgl::Renderer->render(shaderAttributesEdgeDetection);
            glDisable(GL_DEPTH_TEST);
        }
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        sgl::Renderer->bindFBO(framebufferEdgeDetection);
        sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT, sgl::Color(0, 0, 0));
        sgl::Renderer->render(shaderAttributesFullScreenBlitEdgeDetection);

        glDisable(GL_STENCIL_TEST);
        sgl::Renderer->bindFBO(*sceneDataEdgeDetection.framebuffer);
        sgl::Renderer->render(shaderAttributesEdgeDetection);
        glEnable(GL_STENCIL_TEST);
    }
}

bool EdgeDetectionRenderer::renderGuiEdgeDetection() {
    bool reRender = false;
    if (ImGui::Combo(
            "Outline Mode", (int*)&outlineMode, OUTLINE_MODE_NAMES, NUM_OUTLINE_MODES)) {
        if (outlineMode != OUTLINE_MODE_NONE) {
            if (outlineMode == OUTLINE_MODE_DEPTH) {
                shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                        depthTextureShaderEdgeDetection);
            } else if (outlineMode == OUTLINE_MODE_NORMAL) {
                shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                        normalTextureShaderEdgeDetection);
            } else if (outlineMode == OUTLINE_MODE_DEPTH_NORMAL) {
                shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                        depthNormalTextureShaderEdgeDetection);
            } else if (outlineMode == OUTLINE_MODE_STENCIL) {
                shaderAttributesEdgeDetection = shaderAttributesEdgeDetection->copy(
                        colorTextureShaderEdgeDetection);
            }
            reloadTexturesEdgeDetection();
            reloadModelEdgeDetection(meshEdgeDetection);
        }
        reRender = true;
    }
    if (ImGui::Checkbox("Use Laplacian of Gaussian (LoG)", &useLoG)) {
        if (useLoG) {
            weightTextureSize = oldWeightTextureSize;
        } else {
            oldWeightTextureSize = weightTextureSize;
        }
        createWeightTextureEdgeDetection();
        reRender = true;
    }
    if (useLoG && ImGui::SliderInt("Filter Size", &weightTextureSize.x, 4, 7)) {
        weightTextureSize.y = weightTextureSize.x;
        createWeightTextureEdgeDetection();
        reRender = true;
    }
    if (outlineMode != OUTLINE_MODE_STENCIL && ImGui::Checkbox("Use Noise Reduction", &useNoiseReduction)) {
        reloadModelEdgeDetectionShader();
        reRender = true;
    }
    if (useNoiseReduction && noiseReduction.renderGui()) {
        reRender = true;
    }
    return reRender;
}
