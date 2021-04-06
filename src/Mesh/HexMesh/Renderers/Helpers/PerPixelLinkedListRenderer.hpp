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

#ifndef HEXVOLUMERENDERER_PERPIXELLINKEDLISTRENDERER_HPP
#define HEXVOLUMERENDERER_PERPIXELLINKEDLISTRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "Mesh/HexMesh/Renderers/HexahedralMeshRenderer.hpp"

class PerPixelLinkedListRenderer : public HexahedralMeshRenderer {
public:
    PerPixelLinkedListRenderer(
            SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~PerPixelLinkedListRenderer() {}

    // Renders the object to the scene framebuffer.
    virtual void render();

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

protected:
    void initShaders(const std::vector<std::string>& gatherShaderNames);
    virtual void setSortingAlgorithmDefine();
    virtual void setUniformData();
    virtual void clear();
    virtual void gather();
    virtual void resolve();

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributes;

    // Per-pixel linked list data.
    sgl::GeometryBufferPtr fragmentBuffer;
    sgl::GeometryBufferPtr startOffsetBuffer;
    sgl::GeometryBufferPtr atomicCounterBuffer;

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderProgramPtr resolveShader;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;
};


#endif //HEXVOLUMERENDERER_PERPIXELLINKEDLISTRENDERER_HPP
