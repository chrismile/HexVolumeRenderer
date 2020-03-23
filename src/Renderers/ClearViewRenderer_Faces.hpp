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

#ifndef HEXVOLUMERENDERER_CLEARVIEWRENDERER_FACES_HPP
#define HEXVOLUMERENDERER_CLEARVIEWRENDERER_FACES_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

/**
 * Renders the hexahedral mesh using an approach similar to ClearView (see below).
 * The focus region of the mesh is rendered using colored lines with white outlines that are faded out at the boundary.
 * The context region is rendered using face-based volume rendering (@see VolumeRenderer_Faces).
 *
 * For more details on ClearView see: "ClearView: An Interactive Context Preserving Hotspot Visualization Technique",
 * Jens Krüger, Jens Schneider, Rüdiger Westermann (2006)
 * Computer Graphics and Visualization Group, Technical University Munich, Germany
 * https://www.in.tum.de/cg/research/publications/2006/clearview-an-interactive-context-preserving-hotspot-visualization-technique/
 */
class ClearViewRenderer_Faces : public HexahedralMeshRenderer {
public:
    ClearViewRenderer_Faces(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow);
    virtual ~ClearViewRenderer_Faces() {}

    // Re-generates the visualization mapping.
    virtual void generateVisualizationMapping(HexMeshPtr meshIn);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();
    // Updates the internal logic (called once per frame).
    virtual void update(float dt);

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    // Called when the transfer function was changed.
    virtual void onTransferFunctionMapRebuilt() {}

protected:
    void setSortingAlgorithmDefine();
    void setUniformData();
    void clear();
    void gather();
    void resolve();

    // Load either tube or line representation depending on "useTubes".
    void loadFocusRepresentation();
    HexMeshPtr mesh;

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributesContext;
    sgl::ShaderAttributesPtr shaderAttributesFocus;

    // Per-pixel linked list data.
    sgl::GeometryBufferPtr fragmentBuffer;
    sgl::GeometryBufferPtr startOffsetBuffer;
    sgl::GeometryBufferPtr atomicCounterBuffer;

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShaderFocusLines;
    sgl::ShaderProgramPtr gatherShaderFocusTubes;
    sgl::ShaderProgramPtr gatherShaderContext;
    sgl::ShaderProgramPtr resolveShader;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;

    // For rendering the focus point sphere.
    sgl::ShaderProgramPtr shaderProgramSurface;
    sgl::ShaderAttributesPtr focusPointShaderAttributes;

    // GUI data
    bool showRendererWindow = true;
    glm::vec3 focusPoint = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec4 focusPointColor = glm::vec4(0.2f, 0.0f, 0.0f, 1.0f);
    float focusRadius = 0.05f;
    float lineWidth = 0.001f;
    bool useShading = false;
    bool useTubes = true;

    // Focus point move information.
    bool hasHitInformation = false;
    glm::vec3 firstHit, lastHit;
    glm::vec3 hitLookingDirection;
};

#endif //HEXVOLUMERENDERER_CLEARVIEWRENDERER_FACES_HPP
