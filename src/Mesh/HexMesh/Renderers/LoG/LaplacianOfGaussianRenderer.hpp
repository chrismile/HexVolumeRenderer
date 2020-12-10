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

#ifndef HEXVOLUMERENDERER_LAPLACIANOFGAUSSIANRENDERER_HPP
#define HEXVOLUMERENDERER_LAPLACIANOFGAUSSIANRENDERER_HPP

#include <Graphics/Texture/Texture.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>

#include "Mesh/HexMesh/HexMesh.hpp"
#include "../SceneData.hpp"

class LaplacianOfGaussianRenderer {
protected:
    LaplacianOfGaussianRenderer(SceneData &sceneData, bool useSlimMeshData = false)
            : sceneDataLoG(sceneData), useSlimMeshData(useSlimMeshData) {}

    // Initialization & loading.
    void initializeLoG();
    void createWeightTextureLoG();
    void reloadTexturesLoG();
    void reloadModelLoG(HexMeshPtr& mesh);

    // Rendering.
    void setUniformDataLoG();
    void renderLoGContours();
    bool renderGuiLoG();

    SceneData& sceneDataLoG;
    HexMeshPtr meshLoG;
    bool useSlimMeshData;

    // Rendering data for the LoG (Laplacian of Gaussian).
    sgl::ShaderProgramPtr shaderFullScreenBlitLoG;
    sgl::ShaderProgramPtr colorTextureShaderLoG;
    sgl::ShaderProgramPtr depthTextureShaderLoG;
    sgl::ShaderProgramPtr normalTextureShaderLoG;
    sgl::ShaderProgramPtr depthNormalTextureShaderLoG;
    sgl::ShaderProgramPtr meshShaderLoG;
    sgl::ShaderProgramPtr meshShaderNormalLoG;
    sgl::ShaderAttributesPtr meshShaderAttributesLoG;
    sgl::ShaderAttributesPtr meshShaderNormalAttributesLoG;
    sgl::ShaderAttributesPtr shaderAttributesFullScreenBlitLoG;
    sgl::ShaderAttributesPtr shaderAttributesLoG;
    sgl::FramebufferObjectPtr framebufferLoG;
    sgl::TexturePtr imageTextureLoG;
    sgl::TexturePtr normalTextureLoG;
    sgl::TexturePtr depthStencilTextureLoG;
    sgl::TexturePtr weightTextureLoG;
    glm::ivec2 weightTextureSize = glm::ivec2(5, 5);
    const float rhoLoG = 1.0f;

    // GUI data.
    enum OutlineMode {
        OUTLINE_MODE_NONE, OUTLINE_MODE_DEPTH, OUTLINE_MODE_NORMAL, OUTLINE_MODE_DEPTH_NORMAL, OUTLINE_MODE_STENCIL
    };
    const char* const OUTLINE_MODE_NAMES[5] = {
            "None", "Depth", "Normal", "Depth + Normal", "Stencil"
    };
    const int NUM_OUTLINE_MODES = ((int)(sizeof(OUTLINE_MODE_NAMES) / sizeof(*OUTLINE_MODE_NAMES)));
    OutlineMode outlineMode = OUTLINE_MODE_DEPTH;
};


#endif //HEXVOLUMERENDERER_LAPLACIANOFGAUSSIANRENDERER_HPP
