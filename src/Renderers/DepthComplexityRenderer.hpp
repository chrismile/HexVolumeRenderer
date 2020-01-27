//
// Created by christoph on 26.01.20.
//

#ifndef HEXVOLUMERENDERER_DEPTHCOMPLEXITYRENDERER_HPP
#define HEXVOLUMERENDERER_DEPTHCOMPLEXITYRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

class DepthComplexityRenderer : public HexahedralMeshRenderer {
public:
    DepthComplexityRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow);
    virtual ~DepthComplexityRenderer() {}

    // Re-generates the visualization mapping.
    virtual void generateVisualizationMapping(HexMeshPtr meshIn);

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
    void computeStatistics();
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
