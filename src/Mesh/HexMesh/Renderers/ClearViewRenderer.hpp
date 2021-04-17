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

#ifndef HEXVOLUMERENDERER_CLEARVIEWRENDERER_HPP
#define HEXVOLUMERENDERER_CLEARVIEWRENDERER_HPP

#include <chrono>

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "PPLL/PerPixelLinkedList.hpp"
#include "HexahedralMeshRenderer.hpp"
#include "Mesh/HexMesh/Renderers/Intersection/Pickable.hpp"

/**
 * Renders the hexahedral mesh using an approach similar to ClearView (see below).
 * The focus region of the mesh is rendered using colored lines with white outlines that are faded out at the boundary.
 *
 * For more details on ClearView see: "ClearView: An Interactive Context Preserving Hotspot Visualization Technique",
 * Jens Krüger, Jens Schneider, Rüdiger Westermann (2006)
 * Computer Graphics and Visualization Group, Technical University Munich, Germany
 * https://www.in.tum.de/cg/research/publications/2006/clearview-an-interactive-context-preserving-hotspot-visualization-technique/
 */
class ClearViewRenderer : public HexahedralMeshRenderer, protected Pickable {
public:
    ClearViewRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();
    // Updates the internal logic (called once per frame).
    virtual void update(float dt);

protected:
    void loadClearViewBaseData();
    void reloadSphereRenderData();
    void reloadFocusShaders();
    virtual void reloadGatherShader(bool copyShaderAttributes) {}
    void setSortingAlgorithmDefine();
    virtual void setUniformData()=0;
    virtual void clear()=0;
    virtual void gather()=0;
    virtual void resolve()=0;
    virtual void childClassRenderGuiBegin() {}
    virtual void childClassRenderGuiEnd() {}

    // Focus outline rendering.
    void createFocusOutlineRenderingData();
    void reloadFocusOutlineGatherShader();
    void renderFocusOutline(size_t fragmentBufferSize);

    // Sorting algorithm for PPLL.
    SortingAlgorithmMode sortingAlgorithmMode = SORTING_ALGORITHM_MODE_PRIORITY_QUEUE;

    enum LineRenderingMode {
        LINE_RENDERING_MODE_WIREFRAME_FACES,
        LINE_RENDERING_MODE_BILLBOARD_LINES,
        LINE_RENDERING_MODE_TUBES,
        LINE_RENDERING_MODE_TUBES_CAPPED,
        LINE_RENDERING_MODE_TUBES_UNION,
    };
    const char *const LINE_RENDERING_MODE_NAMES[5] = {
            "Wireframe (Faces)", "Billboard Lines", "Tubes", "Tubes (Capped)", "Tubes (Union)"
    };
    const int NUM_LINE_RENDERING_MODES =
#ifdef USE_CSG
        5;
#else
        4;
#endif

    enum LineRenderingStyle {
        LINE_RENDERING_STYLE_HALO,
        LINE_RENDERING_STYLE_TRON
    };
    const char *const LINE_RENDERING_STYLE_NAMES[2] = {
            "Halos", "Tron-like Gloom"
    };
    const int NUM_LINE_RENDERING_STYLES =
            ((int)(sizeof(LINE_RENDERING_STYLE_NAMES) / sizeof(*LINE_RENDERING_STYLE_NAMES)));

    enum ClearViewRendererType {
        CLEAR_VIEW_RENDERER_TYPE_FACES, CLEAR_VIEW_RENDERER_TYPE_VOLUME, CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED
    };
    std::string windowName;
    ClearViewRendererType clearViewRendererType;

    // Load either tube or line representation depending on "useTubes".
    void loadFocusRepresentation();

    // The rendering data for the focus region.
    sgl::ShaderAttributesPtr shaderAttributesFocus;
    sgl::ShaderAttributesPtr shaderAttributesFocusPoints;
    sgl::ShaderAttributesPtr focusPointShaderAttributes;
    sgl::ShaderAttributesPtr focusOutlineShaderAttributes;

    // SSBOs - for gatherShaderFocusSpheres/shaderAttributesFocus.
    sgl::GeometryBufferPtr pointLocationsBuffer;
    sgl::GeometryBufferPtr hexahedralCellFacesBuffer;
    sgl::GeometryBufferPtr hexahedralCellVerticesBuffer;
    sgl::GeometryBufferPtr hexahedralCellEdgesBuffer;
    sgl::GeometryBufferPtr hexahedralCellFacesCellLinksBuffer;
    sgl::GeometryBufferPtr hexahedralCellsBuffer;

    // The shaders for rendering.
    sgl::ShaderProgramPtr gatherShaderFocusWireframeFaces; //< Focus (surface/faces)
    sgl::ShaderProgramPtr gatherShaderFocusLines; //< Focus (surface/lines)
    sgl::ShaderProgramPtr gatherShaderFocusTubes; //< Focus (surface/tubes)
    sgl::ShaderProgramPtr gatherShaderFocusSpheres; //< Focus (surface/spheres)
    sgl::ShaderProgramPtr shaderProgramSurface; //< Focus sphere (surface)
    sgl::ShaderProgramPtr shaderProgramFocusOutline; //< Focus outline

    // LOD data.
    const bool LET_USER_SELECT_LOD_STYLE = true;
    LodSettings lodSettings;

    // Window data.
    const float screenSpaceLensPixelRadiusWindowFactor = 0.25f;
    int windowWidth = 0;
    int windowHeight = 0;

    // Test data.
    float printCounter = 0;

    // GUI data
    bool showRendererWindow = true;
    bool useScreenSpaceLens = false;
    float screenSpaceLensPixelRadius = 0.0f;
    glm::vec2 focusPointScreen = glm::vec2(0.0, 0.0f);
    float focusRadius = 0.05f;
    float lineWidth = 0.0015f;
    bool modulateLineThicknessByDepth = false;
    bool useFocusOutline = true;
    glm::vec4 focusOutlineColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    float focusOutlineWidth = 3.0f;
    bool clipFocusOutline = true;
    bool useShading = false;
    bool useWeightedVertexAttributes = true;
    bool useVolumeWeighting = false;
    LineRenderingMode lineRenderingMode = LINE_RENDERING_MODE_WIREFRAME_FACES;
    LineRenderingStyle lineRenderingStyle = LINE_RENDERING_STYLE_HALO;

    // Depth cue computation.
    void updateDepthCueMode();
    void updateDepthCueGeometryData();
    void setUniformDataDepthCues(sgl::ShaderProgramPtr shaderProgram);
    bool useDepthCues = true;
    float depthCueStrength = 0.5f;
    bool computeDepthCuesOnGpu = true;
    float minDepth = 0.0f;
    float maxDepth = 1.0f;
    std::vector<glm::vec3> filteredCellVertices;
    sgl::GeometryBufferPtr filteredCellVerticesBuffer;
    sgl::GeometryBufferPtr depthMinMaxBuffers[2];
    sgl::ShaderProgramPtr computeDepthValuesShaderProgram;
    sgl::ShaderProgramPtr minMaxReduceDepthShaderProgram;
};

#endif //HEXVOLUMERENDERER_CLEARVIEWRENDERER_HPP
