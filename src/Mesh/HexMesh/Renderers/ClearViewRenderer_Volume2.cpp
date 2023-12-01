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

#include <iostream>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "Mesh/HexMesh/Renderers/Tubes/Tubes.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/Sphere.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "ClearViewRenderer_Volume2.hpp"

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

/// Expected (average) depth complexity, i.e. width*height* this value = number of fragments that can be stored.
static int EXPECTED_DEPTH_COMPLEXITY = 80;
/// Maximum number of fragments to sort in second pass.
static int maxNumFragmentsSorting = 256;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNode {
    // RGBA color of the node.
    uint32_t color;
    // Depth value of the fragment.
    uint32_t depth;
    // The index of the next node in the "nodes" array.
    uint32_t next;
};

ClearViewRenderer_Volume2::ClearViewRenderer_Volume2(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : ClearViewRenderer(sceneData, transferFunctionWindow), EdgeDetectionRenderer(sceneData) {
    windowName = getWindowName();
    clearViewRendererType = CLEAR_VIEW_RENDERER_TYPE_FACES_UNIFIED;
    useScreenSpaceLens = true;

    // Recording mode.
    if (sceneData.recordingMode && sceneData.useCameraFlight) {
        lineWidthBoostFactor = 1.4f;
        selectedLodValueFocus = 0.3f;
        selectedLodValueContext = 0.16f;
        importantLineBoostFactor = 0.8f;
    }

    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListVolume2Gather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(maxNumFragmentsSorting));

    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain"});
    reloadSphereRenderData();

    reloadGatherShader();
    reloadResolveShader();
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListVolume2Clear.Vertex", "LinkedListVolume2Clear.Fragment"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    blitRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    clearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    initializeEdgeDetection();
    onResolutionChanged();
}

void ClearViewRenderer_Volume2::reloadGatherShader() {
    sgl::ShaderManager->invalidateShaderCache();
    std::string lineRenderingStyleDefineName = "LINE_RENDERING_STYLE_HALO";
    sgl::ShaderManager->addPreprocessorDefine(lineRenderingStyleDefineName, "");
    if (tooMuchSingularEdgeMode) {
        sgl::ShaderManager->addPreprocessorDefine("TOO_MUCH_SINGULAR_EDGE_MODE", "");
    }
    if (highlightEdges) {
        sgl::ShaderManager->addPreprocessorDefine("HIGHLIGHT_EDGES", "");
    }
    if (highlightSingularEdges) {
        sgl::ShaderManager->addPreprocessorDefine("HIGHLIGHT_SINGULAR_EDGES", "");
    }
    if (highlightLowLodEdges) {
        sgl::ShaderManager->addPreprocessorDefine("HIGHLIGHT_LOW_LOD_EDGES", "");
    }
    if (usePerLineAttributes) {
        sgl::ShaderManager->addPreprocessorDefine("USE_PER_LINE_ATTRIBUTES", "");
    }
    if (useSingularEdgeColorMap) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SINGULAR_EDGE_COLOR_MAP", "");
    }
    if (accentuateAllEdges) {
        sgl::ShaderManager->addPreprocessorDefine("ACCENTUATE_ALL_EDGES", "");
    }

    if (useScreenSpaceLens) {
        gatherShader = sgl::ShaderManager->getShaderProgram(
                {"MeshLinkedListVolume2.Vertex", "MeshLinkedListVolume2.Fragment.ClearView_ScreenSpace"});
    } else {
        if (useExperimentalApproach) {
            gatherShader = sgl::ShaderManager->getShaderProgram(
                    {"MeshLinkedListVolume2.Vertex", "MeshLinkedListVolume2.Fragment.ClearView_1"});
        } else {
            gatherShader = sgl::ShaderManager->getShaderProgram(
                    {"MeshLinkedListVolume2.Vertex", "MeshLinkedListVolume2.Fragment.ClearView_0"});
        }
    }

    sgl::ShaderManager->removePreprocessorDefine(lineRenderingStyleDefineName);
    if (tooMuchSingularEdgeMode) {
        sgl::ShaderManager->removePreprocessorDefine("TOO_MUCH_SINGULAR_EDGE_MODE");
    }
    if (highlightEdges) {
        sgl::ShaderManager->removePreprocessorDefine("HIGHLIGHT_EDGES");
    }
    if (highlightSingularEdges) {
        sgl::ShaderManager->removePreprocessorDefine("HIGHLIGHT_SINGULAR_EDGES");
    }
    if (highlightLowLodEdges) {
        sgl::ShaderManager->removePreprocessorDefine("HIGHLIGHT_LOW_LOD_EDGES");
    }
    if (usePerLineAttributes) {
        sgl::ShaderManager->removePreprocessorDefine("USE_PER_LINE_ATTRIBUTES");
    }
    if (useSingularEdgeColorMap) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SINGULAR_EDGE_COLOR_MAP");
    }
    if (accentuateAllEdges) {
        sgl::ShaderManager->removePreprocessorDefine("ACCENTUATE_ALL_EDGES");
    }
}

void ClearViewRenderer_Volume2::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("VOLUME_FACTOR",
            std::to_string(volumeOpacityFactor * 400.0f));
    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListVolume2Resolve.Vertex", "LinkedListVolume2Resolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
    sgl::ShaderManager->removePreprocessorDefine("VOLUME_FACTOR");
}

void ClearViewRenderer_Volume2::setNewSettings(const SettingsMap& settings) {
    lineWidthBoostFactor = 1.0f;
    focusRadiusBoostFactor = 1.0f;
    settings.getValueOpt("lineWidthBoostFactor", lineWidthBoostFactor);
    settings.getValueOpt("focusRadiusBoostFactor", focusRadiusBoostFactor);

    if (hexMesh) {
        const float avgCellVolumeCbrt = std::cbrt(hexMesh->getAverageCellVolume());
        lineWidth = lineWidthBoostFactor * glm::clamp(
                avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
        focusRadius = focusRadiusBoostFactor * glm::clamp(
                avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
    }
}

void ClearViewRenderer_Volume2::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    if (isNewMesh) {
        Pickable::focusPoint = glm::vec3(0.0f);
    }

    hexMesh = meshIn;
    const float avgCellVolumeCbrt = std::cbrt(meshIn->getAverageCellVolume());
    // Higher radius for recording...
    lineWidth = lineWidthBoostFactor * glm::clamp(
            avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
    focusRadius = focusRadiusBoostFactor * glm::clamp(
            avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
    reloadSphereRenderData();

    // Don't highlight singular edges when we have far too many of them.
    bool tooMuchSingularEdgeModeNewMesh = meshIn->getNumberOfSingularEdges(true, 1) > 10000u;
    if (tooMuchSingularEdgeModeNewMesh != tooMuchSingularEdgeMode) {
        tooMuchSingularEdgeMode = tooMuchSingularEdgeModeNewMesh;
        reloadGatherShader();
    }

    // Get the information about the singularity structure.
    singularEdgeColorMapWidget.generateSingularityStructureInformation(hexMesh);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    hexahedralCellFacesBuffer = sgl::GeometryBufferPtr();

    // Load the unified data for the focus and context region.
    std::vector<uint32_t> indices;
    std::vector<HexahedralCellFaceUnified_Volume2> hexahedralCellFaces;
    if (useWeightedVertexAttributes) {
        hexMesh->getSurfaceDataWireframeFacesUnified_AttributePerVertex_Volume2(
                indices, hexahedralCellFaces, maxLodValue);
    } else {
        hexMesh->getSurfaceDataWireframeFacesUnified_AttributePerCell_Volume2(
                indices, hexahedralCellFaces, maxLodValue);
    }

    size_t modelBufferSizeBytes = indices.size() * sizeof(uint32_t)
                                  + hexahedralCellFaces.size() * sizeof(HexahedralCellFaceUnified);
    sgl::Logfile::get()->writeInfo(
            std::string() + "GPU model buffer size MiB: "
            + std::to_string(modelBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), indices.data(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(HexahedralCellFaceUnified_Volume2),
            hexahedralCellFaces.data(), sgl::SHADER_STORAGE_BUFFER);

    reloadModelEdgeDetection(hexMesh);

    dirty = false;
    reRender = true;
    hasHitInformation = false;
}

void ClearViewRenderer_Volume2::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();
    screenSpaceLensPixelRadius = std::min(width, height) * screenSpaceLensPixelRadiusWindowFactor;
    focusPointScreen = glm::vec2(width / 2.0f, height / 2.0f);
    windowWidth = width;
    windowHeight = height;

    fragmentBufferSize = size_t(EXPECTED_DEPTH_COMPLEXITY) * size_t(width) * size_t(height);
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNode) * fragmentBufferSize;
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeBytes = (1ull << 32ull) - sizeof(LinkedListFragmentNode);
        fragmentBufferSize = fragmentBufferSizeBytes / sizeof(LinkedListFragmentNode);
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));
    }

    if (sceneData.performanceMeasurer) {
        sceneData.performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }

    fragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * width * height;
    startOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);

    reloadTexturesEdgeDetection();
}

void ClearViewRenderer_Volume2::setUniformData() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
    glm::vec3 lookingDirection(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, startOffsetBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShader->setUniform("viewportW", width);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    if (gatherShader->hasUniform("cameraPosition")) {
        gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    if (gatherShader->hasUniform("lookingDirection")) {
        gatherShader->setUniform("lookingDirection", lookingDirection);
    }
    gatherShader->setUniform(
            "minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShader->setUniform(
            "maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
    gatherShader->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    if (gatherShader->hasUniform("singularEdgeColorLookupTexture")) {
        gatherShader->setUniform(
                "singularEdgeColorLookupTexture",
                singularEdgeColorMapWidget.getSingularEdgeColorLookupTexture(), 1);
    }

    if (gatherShader->hasUniform("lineWidth")) {
        gatherShader->setUniform("lineWidth", lineWidth);
    }
    if (gatherShader->hasUniform("maxLodValue")) {
        gatherShader->setUniform("maxLodValue", float(maxLodValue));
    }
    if (gatherShader->hasUniform("selectedLodValueFocus")) {
        gatherShader->setUniform("selectedLodValueFocus", float(selectedLodValueFocus));
    }
    if (gatherShader->hasUniform("selectedLodValueContext")) {
        gatherShader->setUniform("selectedLodValueContext", float(selectedLodValueContext));
    }
    if (gatherShader->hasUniform("importantLineBoostFactor")) {
        gatherShader->setUniform("importantLineBoostFactor", float(importantLineBoostFactor));
    }

    if (useScreenSpaceLens) {
        gatherShader->setUniform("viewportSize", glm::ivec2(windowWidth, windowHeight));
        gatherShader->setUniform("sphereCenterScreen", focusPointScreen);
        gatherShader->setUniform("sphereRadiusPixels", screenSpaceLensPixelRadius);
    } else {
        gatherShader->setUniform("sphereCenter", focusPoint);
        gatherShader->setUniform("sphereRadius", focusRadius);
    }

    shaderProgramSurface->setUniform("viewportW", width);
    shaderProgramSurface->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    if (shaderProgramSurface->hasUniform("cameraPosition")) {
        shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());
    }
    shaderProgramSurface->setUniform("color", focusPointColor);

    resolveShader->setUniform("viewportW", width);
    if (resolveShader->hasUniform("zNear")) {
        resolveShader->setUniform("zNear", sceneData.camera->getNearClipDistance());
        resolveShader->setUniform("zFar", sceneData.camera->getFarClipDistance());
    }

    clearShader->setUniform("viewportW", width);

    setUniformDataEdgeDetection();
}

void ClearViewRenderer_Volume2::clear() {
    glDepthMask(GL_FALSE);

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearRenderData);

    // Set atomic counter to zero.
    GLuint bufferID = static_cast<sgl::GeometryBufferGL*>(atomicCounterBuffer.get())->getBuffer();
    GLubyte val = 0;
    glClearNamedBufferData(bufferID, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void ClearViewRenderer_Volume2::gather() {
    // Recording mode.
    if (sceneData.recordingMode && sceneData.useCameraFlight) {
        // TODO: Adapt focus radius depending on distance of camera to origin?
    }

    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // We can use the stencil buffer to mask used pixels for the resolve pass.
    if (useStencilBuffer) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glClear(GL_STENCIL_BUFFER_BIT);
    }

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    sgl::ShaderManager->bindShaderStorageBuffer(6, hexahedralCellFacesBuffer);
    //if (useWeightedVertexAttributes) {
    //    glDisable(GL_CULL_FACE);
    //}
    sgl::Renderer->render(shaderAttributes);
    //if (useWeightedVertexAttributes) {
    //    glEnable(GL_CULL_FACE);
    //}
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Render the focus point.
    if (!useScreenSpaceLens) {
        sgl::Renderer->setModelMatrix(sgl::matrixTranslation(focusPoint));
        sgl::Renderer->render(focusPointShaderAttributes);
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void ClearViewRenderer_Volume2::resolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    if (useStencilBuffer) {
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
    }

    sgl::Renderer->render(blitRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    renderEdgeDetectionContours();

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void ClearViewRenderer_Volume2::renderGui() {
    ClearViewRenderer::renderGui();

    if (highlightEdges && useSingularEdgeColorMap && singularEdgeColorMapWidget.renderGui()) {
        reRender = true;
    }
}

void ClearViewRenderer_Volume2::childClassRenderGuiBegin() {
    if (ImGui::Checkbox("Use Screen Space Lens", &useScreenSpaceLens)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (useScreenSpaceLens && ImGui::SliderFloat(
            "Lens Pixel Radius", &screenSpaceLensPixelRadius, 0.0f, std::max(windowWidth, windowHeight))) {
        reRender = true;
    }
}

void ClearViewRenderer_Volume2::childClassRenderGuiEnd() {
    if (ImGui::SliderFloat("volumeOpacityFactor", &volumeOpacityFactor, 0.0f, 2.0f)) {
        reloadResolveShader();
        reRender = true;
    }
    if (!useScreenSpaceLens && ImGui::Checkbox("Use Experimental Approach", &useExperimentalApproach)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if ((useScreenSpaceLens || useExperimentalApproach)
        && ImGui::SliderFloat("LOD Value Focus", &selectedLodValueFocus, 0.0f, 1.0f)) {
        if (selectedLodValueFocus < selectedLodValueContext) {
            selectedLodValueContext = selectedLodValueFocus;
        }
        reRender = true;
    }
    if ((useScreenSpaceLens || useExperimentalApproach)
        && ImGui::SliderFloat("LOD Value Context", &selectedLodValueContext, 0.0f, 1.0f)) {
        if (selectedLodValueFocus < selectedLodValueContext) {
            selectedLodValueFocus = selectedLodValueContext;
        }
        reRender = true;
    }
    if ((useScreenSpaceLens || useExperimentalApproach)
        && ImGui::SliderFloat("Important Lines", &importantLineBoostFactor, 0.0f, 1.0f)) {
        reRender = true;
    }
    if ((useScreenSpaceLens || useExperimentalApproach)
        && ImGui::Checkbox("Accentuate Edges", &accentuateAllEdges)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if ((useScreenSpaceLens || useExperimentalApproach)
        && ImGui::Checkbox("Per Line Attributes", &usePerLineAttributes)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (ImGui::Checkbox("Use Singular Edge Color Map", &useSingularEdgeColorMap)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (ImGui::Checkbox("Highlight Edges", &highlightEdges)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (!useScreenSpaceLens && !useExperimentalApproach && highlightEdges
        && ImGui::Checkbox("Highlight Low LOD Edges", &highlightLowLodEdges)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (!useScreenSpaceLens && !useExperimentalApproach && highlightEdges
        && ImGui::Checkbox("Highlight Singular Edges", &highlightSingularEdges)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    reRender = renderGuiEdgeDetection() || reRender;
    if (ImGui::Button("Reload Shader")) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reloadResolveShader();
        reRender = true;
    }
}
