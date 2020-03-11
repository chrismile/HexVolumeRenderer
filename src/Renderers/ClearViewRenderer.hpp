//
// Created by christoph on 11.03.20.
//

#ifndef HEXVOLUMERENDERER_CLEARVIEWRENDERER_HPP
#define HEXVOLUMERENDERER_CLEARVIEWRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

class ClearViewRenderer : public HexahedralMeshRenderer {
public:
    ClearViewRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow);
    virtual ~ClearViewRenderer() {}

    // Re-generates the visualization mapping.
    virtual void generateVisualizationMapping(HexMeshPtr meshIn);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

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

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributesFocus;
    sgl::ShaderAttributesPtr shaderAttributesContext;

    // Per-pixel linked list data.
    sgl::GeometryBufferPtr fragmentBuffer;
    sgl::GeometryBufferPtr startOffsetBuffer;
    sgl::GeometryBufferPtr atomicCounterBuffer;

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShaderFocus;
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
    glm::vec4 focusPointColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    float focusRadius = 0.1f;

};

#endif //HEXVOLUMERENDERER_CLEARVIEWRENDERER_HPP
