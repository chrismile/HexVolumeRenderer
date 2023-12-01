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

#include <glm/vec3.hpp>

#include <Utils/AppSettings.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Mesh/HexMesh/Renderers/SceneData.hpp"
#include "NoiseReduction.hpp"

void NoiseReduction::initialize(bool useSingleChannelTexture) {
    this->useSingleChannelTexture = useSingleChannelTexture;
    if (useSingleChannelTexture) {
        noiseReductionShader = sgl::ShaderManager->getShaderProgram(
                {"NoiseReduction.Vertex", "NoiseReduction.Fragment.SingleChannel"});
    } else {
        noiseReductionShader = sgl::ShaderManager->getShaderProgram(
                {"NoiseReduction.Vertex", "NoiseReduction.Fragment"});
    }
    blitShader = sgl::ShaderManager->getShaderProgram(
            {"Blit.Vertex.NoTexCoord", "Blit.Fragment.NoTexCoord"});

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());

    noiseReductionShaderAttributes = sgl::ShaderManager->createShaderAttributes(noiseReductionShader);
    noiseReductionShaderAttributes->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    blitShaderAttributes = sgl::ShaderManager->createShaderAttributes(blitShader);
    blitShaderAttributes->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
}

void NoiseReduction::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    sgl::TextureSettings textureSettings;
    if (useSingleChannelTexture) {
        textureSettings.internalFormat = GL_R8;
    }
    for (int i = 0; i < 2; i++) {
        textures[i] = sgl::TextureManager->createEmptyTexture(width, height, sgl::TextureSettings());
        framebuffers[i] = sgl::Renderer->createFBO();
        framebuffers[i]->bindTexture(textures[i]);
    }
}

void NoiseReduction::bindFramebufferForRendering() {
    sgl::Renderer->bindFBO(framebuffers[0]);
    sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT);
}

void NoiseReduction::denoiseTexture() {
    sgl::TexturePtr* textureIn;
    sgl::FramebufferObjectPtr* framebufferOut;

    for (iteration = 0; iteration < numIterations; iteration++) {
        textureIn = &textures[iteration % 2];
        framebufferOut = &framebuffers[(iteration + 1) % 2];
        sgl::Renderer->bindFBO(*framebufferOut);
        noiseReductionShader->setUniform("inputTexture", *textureIn, 0);
        sgl::Renderer->render(noiseReductionShaderAttributes);
    }
}

void NoiseReduction::blitDenoisedTexture() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    blitShader->setUniform("inputTexture", textures[iteration % 2], 0);
    blitShader->setUniform("viewportSize", glm::ivec2(width, height));
    sgl::Renderer->render(blitShaderAttributes);
}

sgl::TexturePtr& NoiseReduction::getDenoisedTexture() {
    return textures[iteration % 2];
}

bool NoiseReduction::renderGui() {
    bool reRender = false;
    if (ImGui::SliderInt("#Iterations", &numIterations, 1, 20)) {
        reRender = true;
    }
    return reRender;
}
