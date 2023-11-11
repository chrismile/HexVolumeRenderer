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

#ifndef HEXVOLUMERENDERER_CLEARVIEWRENDERER_VOLUME2_HPP
#define HEXVOLUMERENDERER_CLEARVIEWRENDERER_VOLUME2_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "Mesh/HexMesh/Renderers/Widgets/SingularEdgeColorMapWidget.hpp"
#include "EdgeDetection/EdgeDetectionRenderer.hpp"
#include "ClearViewRenderer.hpp"

/**
 * Renders the hexahedral mesh using an approach similar to ClearView (see below).
 * The focus region of the mesh is rendered using colored lines with white outlines that are faded out at the boundary.
 * The context region is rendered using face-based volume rendering (@see VolumeRenderer_Faces), but instead of
 * rendering the faces with a flat color, the wireframe data (@see WireframeRenderer_Faces) is used to accentuate edges.
 * Thus, the same rendering data can be used for the context and focus region (i.e., a unified face-based approach).
 *
 * For more details on ClearView see: "ClearView: An Interactive Context Preserving Hotspot Visualization Technique",
 * Jens Krüger, Jens Schneider, Rüdiger Westermann (2006)
 * Computer Graphics and Visualization Group, Technical University Munich, Germany
 * https://www.in.tum.de/cg/research/publications/2006/clearview-an-interactive-context-preserving-hotspot-visualization-technique/
 */
class ClearViewRenderer_Volume2 : public ClearViewRenderer, protected EdgeDetectionRenderer {
public:
    ClearViewRenderer_Volume2(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~ClearViewRenderer_Volume2() {}

    static const char* getWindowName() { return "ClearView Renderer (Volume 2)"; }

    /**
     * Re-generates the visualization mapping.
     * @param meshIn The mesh to generate a visualization mapping for.
     * @param isNewMesh Whether a new mesh is loaded or just a new renderer is used.
     */
    virtual void uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh);

    /// Removes the old mesh.
    virtual void removeOldMesh() override {
        HexahedralMeshRenderer::removeOldMesh();
        removeOldMeshEdgeDetection();
    }

    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    /// Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    /// For changing performance measurement modes.
    virtual void setNewSettings(const SettingsMap& settings);

protected:
    void setUniformData();
    void clear();
    void gather();
    void resolve();
    void childClassRenderGuiBegin();
    void childClassRenderGuiEnd();

    // Don't highlight singular edges when we have far too many of them.
    void reloadGatherShader();
    void reloadResolveShader();
    bool tooMuchSingularEdgeMode = false;
    int maxLodValue;

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributes;

    // Per-pixel linked list data.
    size_t fragmentBufferSize = 0;
    sgl::GeometryBufferPtr fragmentBuffer;
    sgl::GeometryBufferPtr startOffsetBuffer;
    sgl::GeometryBufferPtr atomicCounterBuffer;

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderProgramPtr resolveShader;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates).
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;

    // Window data.
    const float screenSpaceLensPixelRadiusWindowFactor = 0.25f;
    int windowWidth = 0;
    int windowHeight = 0;

    // Settings for changing internal state.
    float lineWidthBoostFactor = 1.0f;
    float focusRadiusBoostFactor = 1.0f;

    // GUI data.
    SingularEdgeColorMapWidget singularEdgeColorMapWidget;
    bool useExperimentalApproach = true;
    float volumeOpacityFactor = 1.0f;
    float selectedLodValueFocus = 0.3f;
    float selectedLodValueContext = 0.16f;
    float importantLineBoostFactor = 0.5f;
    bool accentuateAllEdges = true;
    bool useSingularEdgeColorMap = false;
    bool usePerLineAttributes = true;
    bool highlightEdges = true;
    bool highlightLowLodEdges = true;
    bool highlightSingularEdges = true;
};

#endif //HEXVOLUMERENDERER_CLEARVIEWRENDERER_VOLUME2_HPP
