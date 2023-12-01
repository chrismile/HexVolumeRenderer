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

#ifndef HEXVOLUMERENDERER_NOISEREDUCTION_HPP
#define HEXVOLUMERENDERER_NOISEREDUCTION_HPP

#include <Graphics/Texture/Texture.hpp>
#include <Graphics/Shader/Shader.hpp>

struct SceneData;

class NoiseReduction {
public:
    NoiseReduction(SceneData& sceneData) : sceneData(sceneData) {}
    void initialize(bool useSingleChannelTexture = false);
    void onResolutionChanged();
    void bindFramebufferForRendering();
    void denoiseTexture();
    void blitDenoisedTexture();
    sgl::TexturePtr& getDenoisedTexture();
    bool renderGui();

private:
    SceneData& sceneData;

    bool useSingleChannelTexture = false;
    sgl::ShaderProgramPtr noiseReductionShader;
    sgl::ShaderProgramPtr blitShader;
    sgl::ShaderAttributesPtr noiseReductionShaderAttributes;
    sgl::ShaderAttributesPtr blitShaderAttributes;

    sgl::FramebufferObjectPtr framebuffers[2];
    sgl::TexturePtr textures[2];
    int iteration = 0;

    // GUI settings.
    int numIterations = 2;
};

#endif //HEXVOLUMERENDERER_NOISEREDUCTION_HPP
