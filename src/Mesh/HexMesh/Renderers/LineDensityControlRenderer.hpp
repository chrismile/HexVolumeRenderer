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

#ifndef HEXVOLUMERENDERER_LINEDENSITYCONTROLRENDERER_HPP
#define HEXVOLUMERENDERER_LINEDENSITYCONTROLRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "Mesh/HexMesh/Renderers/Widgets/SingularEdgeColorMapWidget.hpp"
#include "HexahedralMeshRenderer.hpp"

/**
 * For more details on line density control see: "Line density control in screen-space via balanced line hierarchies",
 * Mathias Kanzler, Florian Ferstl and Rüdiger Westermann (2016)
 * Computer Graphics and Visualization Group, Technical University Munich, Germany
 * https://www.in.tum.de/cg/research/publications/2016/line-density-control-in-screen-space-via-balanced-line-hierarchies/
 */
class LineDensityControlRenderer : public HexahedralMeshRenderer {
public:
    LineDensityControlRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~LineDensityControlRenderer() {}

    static const char* getWindowName() { return "Line Density Control Renderer"; }

    /**
     * Re-generates the visualization mapping.
     * @param meshIn The mesh to generate a visualization mapping for.
     * @param isNewMesh Whether a new mesh is loaded or just a new renderer is used.
     */
    virtual void uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh);

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

protected:
    void setUniformData();
    void attributeTextureClear();
    void attributeTextureGather();
    void attributeTextureResolve();
    void attributeTextureBlur();
    void lineDensityControlRendering();

    // Rendering data for both the attribute texture creation pass and the line rendering pass.
    sgl::GeometryBufferPtr hexahedralCellFacesBuffer;
    int maxLodValue = 0;

    // Rendering data for rendering the lines to the screen.
    sgl::ShaderProgramPtr lineDensityControlShader;
    sgl::ShaderAttributesPtr lineDensityControlRenderData;

    // For creating the attribute texture using per-pixel linked lists.
    sgl::ShaderProgramPtr createAttributeTextureClearShader;
    sgl::ShaderProgramPtr createAttributeTextureGatherShader;
    sgl::ShaderProgramPtr createAttributeTextureResolveShader;
    size_t fragmentBufferSize = 0;
    sgl::GeometryBufferPtr createAttributeTextureFragmentBuffer;
    sgl::GeometryBufferPtr createAttributeTextureStartOffsetBuffer;
    sgl::GeometryBufferPtr createAttributeTextureAtomicCounterBuffer;
    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr createAttributeTextureGatherRenderData;
    sgl::ShaderAttributesPtr createAttributeTextureResolveRenderData;
    sgl::ShaderAttributesPtr createAttributeTextureClearRenderData;

    // The attribute texture.
    sgl::FramebufferObjectPtr attributeTextureFramebuffer;
    sgl::TexturePtr attributeTexture;
    float attributeTextureSubsamplingFactor = 0.25f;
    glm::ivec2 attributeTextureResolution;

    // For blurring the attribute texture.
    sgl::ShaderProgramPtr blurShader;
    sgl::ShaderAttributesPtr blurRenderData;
    sgl::FramebufferObjectPtr blurFramebuffer;
    sgl::TexturePtr tempBlurTexture;

    // GUI data.
    bool showRendererWindow = true;
    float lineWidth = 0.0015f;
    SingularEdgeColorMapWidget singularEdgeColorMapWidget;

    // Scalar weights for computing visibility values rho and visibility parameter lambda.
    float lambda = 1.0f;
    float factor_m = 10.0f;
    float factor_c = 10.0f;
    float factor_v = 10.0f;
    float factor_d = 10.0f;
};


#endif //HEXVOLUMERENDERER_LINEDENSITYCONTROLRENDERER_HPP
