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

#ifndef HEXVOLUMERENDERER_DEPTHCOMPLEXITYRENDERER_HPP
#define HEXVOLUMERENDERER_DEPTHCOMPLEXITYRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

/**
 * Renders the hexahedral mesh with all faces and determines the depth complexity in each pixel.
 */
class DepthComplexityRenderer : public HexahedralMeshRenderer {
public:
    DepthComplexityRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~DepthComplexityRenderer() {}

    static const char* getWindowName() { return "Depth Complexity Renderer"; }

    /**
     * Re-generates the visualization mapping.
     * @param meshIn The mesh to generate a visualization mapping for.
     * @param isNewMesh Whether a new mesh is loaded or just a new renderer is used.
     */
    virtual void uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender();

    // Called when the transfer function was changed.
    virtual void onTransferFunctionMapRebuilt() {}

protected:
    void computeStatistics(bool isReRender);
    void setUniformData();
    void clear();
    void gather();
    void resolve();
    
    ImVec4 colorSelection = ImColor(0, 255, 255, 127);
    sgl::Color renderColor = sgl::Color(0, 255, 255);
    uint32_t numFragmentsMaxColor; // = max(16, max. depth complexity of scene)
    bool firstFrame = true;

    // User interface
    bool showWindow = true;
    uint64_t totalNumFragments = 0;
    uint64_t usedLocations = 1;
    uint64_t maxComplexity = 0;
    uint64_t bufferSize = 1;
    float intensity = 1.5f;

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributes;
    sgl::GeometryBufferPtr fragmentCounterBuffer;

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderProgramPtr resolveShader;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;
};

#endif //HEXVOLUMERENDERER_DEPTHCOMPLEXITYRENDERER_HPP
