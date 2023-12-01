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

#ifndef HEXVOLUMERENDERER_EDGEDETECTIONRENDERER_HPP
#define HEXVOLUMERENDERER_EDGEDETECTIONRENDERER_HPP

#include <Graphics/Texture/Texture.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>

#include "Mesh/HexMesh/HexMesh.hpp"
#include "ImageFilters/NoiseReduction.hpp"
#include "../SceneData.hpp"

class EdgeDetectionRenderer {
protected:
    EdgeDetectionRenderer(SceneData &sceneData, bool useSlimMeshData = false)
            : sceneDataEdgeDetection(sceneData), useSlimMeshData(useSlimMeshData), noiseReduction(sceneData) {}

    // Initialization & loading.
    void initializeEdgeDetection();
    void createWeightTextureEdgeDetection();
    void reloadTexturesEdgeDetection();
    void reloadModelEdgeDetection(HexMeshPtr& mesh);

    /// Removes the old mesh.
    inline void removeOldMeshEdgeDetection() { meshEdgeDetection = HexMeshPtr(); }

    // Rendering.
    void setUniformDataEdgeDetection();
    void renderEdgeDetectionContours();
    bool renderGuiEdgeDetection();

private:
    void reloadModelEdgeDetectionShader();

    SceneData& sceneDataEdgeDetection;
    HexMeshPtr meshEdgeDetection;
    bool useSlimMeshData;

    // Noise reduction filter.
    NoiseReduction noiseReduction;

    // Rendering data for the EdgeDetection (Laplacian of Gaussian).
    sgl::ShaderProgramPtr shaderFullScreenBlitEdgeDetection;
    sgl::ShaderProgramPtr colorTextureShaderEdgeDetection;
    sgl::ShaderProgramPtr depthTextureShaderEdgeDetection;
    sgl::ShaderProgramPtr normalTextureShaderEdgeDetection;
    sgl::ShaderProgramPtr depthNormalTextureShaderEdgeDetection;
    sgl::ShaderProgramPtr meshShaderEdgeDetection;
    sgl::ShaderProgramPtr meshShaderNormalEdgeDetection;
    sgl::ShaderProgramPtr directBlitShader;
    sgl::ShaderAttributesPtr meshShaderAttributesEdgeDetection;
    sgl::ShaderAttributesPtr meshShaderNormalAttributesEdgeDetection;
    sgl::ShaderAttributesPtr shaderAttributesFullScreenBlitEdgeDetection;
    sgl::ShaderAttributesPtr shaderAttributesEdgeDetection;
    sgl::ShaderAttributesPtr directBlitShaderAttributes;
    sgl::FramebufferObjectPtr framebufferEdgeDetection;
    sgl::TexturePtr imageTextureEdgeDetection;
    sgl::TexturePtr normalTextureEdgeDetection;
    sgl::TexturePtr depthStencilTextureEdgeDetection;
    sgl::TexturePtr weightTextureEdgeDetection;
    glm::ivec2 weightTextureSize;
    glm::ivec2 oldWeightTextureSize;
    const float rhoEdgeDetection = 1.0f;
    bool useLoG = false; ///< Use Laplacian of Gaussian (LoG) or isotropic Laplacian filter?
    bool useNoiseReduction = true;

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


#endif //HEXVOLUMERENDERER_EDGEDETECTIONRENDERER_HPP
