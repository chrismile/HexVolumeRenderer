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

#ifndef HEXVOLUMERENDERER_HEXSHEETRENDERER_HPP
#define HEXVOLUMERENDERER_HEXSHEETRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"
#include "Mesh/HexMesh/Renderers/LOD/HexahedralSheet.hpp"

class HexSheetRenderer : public HexahedralMeshRenderer {
public:
    HexSheetRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~HexSheetRenderer() {}

    static const char* getWindowName() { return "Hexahedral Sheet Renderer"; }

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

protected:
    void recreateRenderingData();
    sgl::ShaderProgramPtr shaderProgramHull;
    sgl::ShaderProgramPtr shaderProgramSheet;
    sgl::ShaderAttributesPtr shaderAttributesHull;
    sgl::ShaderAttributesPtr shaderAttributesSheet;
    sgl::ShaderProgramPtr shaderProgramWireframe;
    sgl::ShaderAttributesPtr shaderAttributesWireframe;
    sgl::GeometryBufferPtr wireframeFacesBuffer;

    std::vector<HexahedralSheet> hexahedralSheets;
    std::set<ComponentConnectionData> connectionDataSet;
    std::set<ComponentConnectionData> currentSheetConnectionDataSet;
    int selectedSheetIndex0 = -1, selectedSheetIndex1 = -1;
    int selectedRow = -1;

    //const glm::vec4 sheet0Color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    //const glm::vec4 sheet1Color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
    //const glm::vec4 sheetIntersectionColor = glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);
    //const glm::vec4 hullColor = glm::vec4(0.537254902f, 0.803921569f, 1.0f, 0.1f);
    glm::vec4 sheet0Color = glm::vec4(
            sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(1.0f, 0.0f, 0.0f)), 1.0f);
    glm::vec4 sheet1Color = glm::vec4(
            sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(0.537254902f, 0.803921569f, 1.0f)), 1.0f);
    glm::vec4 sheetIntersectionColor = glm::vec4(
            sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(1.0f, 1.0f, 0.0f)), 1.0f);
    glm::vec4 hullColor = glm::vec4(
            sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(0.5, 0.5, 0.5f)), 0.3f);

    // GUI data
    bool showRendererWindow = true;
    float lineWidth = 0.0015f;
    float hullOpacity = 0.3f;
    bool useShading = true;
    bool showConnectedSheets = true;
};

#endif //HEXVOLUMERENDERER_HEXSHEETRENDERER_HPP
