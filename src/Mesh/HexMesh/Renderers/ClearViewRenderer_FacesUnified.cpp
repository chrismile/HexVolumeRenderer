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
#include <Utils/File/FileUtils.hpp>
#include <Utils/AppSettings.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "Mesh/HexMesh/Renderers/Tubes/Tubes.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/Sphere.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"
#include "ClearViewRenderer_FacesUnified.hpp"

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

// A fragment node stores rendering information about one specific fragment.
struct LinkedListFragmentNode {
    // RGBA color of the node.
    uint32_t color;
    // Depth value of the fragment (in view space).
    float depth;
    // The index of the next node in the "nodes" array.
    uint32_t next;
};

ClearViewRenderer_FacesUnified::ClearViewRenderer_FacesUnified(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : ClearViewRenderer(sceneData, transferFunctionWindow) {
    windowName = "ClearView Renderer (Unified)";
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
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");

    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain"});
    reloadSphereRenderData();

    reloadGatherShader();
    reloadResolveShader();
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClear.Vertex", "LinkedListClear.Fragment"});
    shaderFullScreenBlitLoG = sgl::ShaderManager->getShaderProgram(
            {"Mesh.Vertex.Plain", "Mesh.Fragment.Plain"});
    colorTextureShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"LaplacianOfGaussian.Vertex", "LaplacianOfGaussian.Fragment.ColorTexture"});
    depthTextureShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"LaplacianOfGaussian.Vertex", "LaplacianOfGaussian.Fragment.DepthTexture"});
    meshShaderLoG = sgl::ShaderManager->getShaderProgram(
            {"Mesh.Vertex.Plain", "Mesh.Fragment.Plain"});

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    blitRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    clearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    shaderAttributesFullScreenBlitLoG = sgl::ShaderManager->createShaderAttributes(shaderFullScreenBlitLoG);
    shaderAttributesFullScreenBlitLoG->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    if (outlineMode == OUTLINE_MODE_DEPTH) {
        shaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(depthTextureShaderLoG);
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        shaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(colorTextureShaderLoG);
    }
    shaderAttributesLoG->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    createWeightTextureLoG();
    onResolutionChanged();
}

ClearViewRenderer_FacesUnified::~ClearViewRenderer_FacesUnified() {
    if (sceneData.performanceMeasurer && !timerDataIsWritten && timer) {
        delete timer;
        sceneData.performanceMeasurer->setClearViewTimer(nullptr);
    }
}

void ClearViewRenderer_FacesUnified::render() {
    setUniformData();
    if (sceneData.performanceMeasurer) {
        timer->startGPU("PPLLClear", frameCounter);
        clear();
        timer->end();
        timer->startGPU("FCGather", frameCounter);
        gather();
        timer->end();
        timer->startGPU("PPLLResolve", frameCounter);
        resolve();
        timer->end();
    } else {
        clear();
        gather();
        resolve();
    }
    frameCounter++;
}

void ClearViewRenderer_FacesUnified::reloadTexturesLoG() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    // Create the data for the LoG convolution.
    if (outlineMode == OUTLINE_MODE_DEPTH) {
        framebufferLoG = sgl::Renderer->createFBO();
        sgl::TextureSettings textureSettings;
        /*if (useLinearRGB) {
            textureSettings.internalFormat = GL_RGBA16;
        } else {*/
        textureSettings.internalFormat = GL_RGBA8;
        //}
        textureSettings.pixelType = GL_UNSIGNED_BYTE;
        textureSettings.pixelFormat = GL_RGB;
        imageTextureLoG = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
        framebufferLoG->bindTexture(sceneData.sceneTexture);

        depthStencilTextureLoG = sgl::TextureManager->createDepthStencilTexture(width, height, sgl::DEPTH24_STENCIL8);
        depthStencilTextureLoG->setDepthStencilComponentMode(sgl::DEPTH_STENCIL_TEXTURE_MODE_DEPTH_COMPONENT);
        framebufferLoG->bindTexture(depthStencilTextureLoG, sgl::DEPTH_STENCIL_ATTACHMENT);
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        framebufferLoG = sgl::Renderer->createFBO();
        sgl::TextureSettings textureSettings;
        /*if (useLinearRGB) {
            textureSettings.internalFormat = GL_RGBA16;
        } else {*/
        textureSettings.internalFormat = GL_RGBA8;
        //}
        textureSettings.pixelType = GL_UNSIGNED_BYTE;
        textureSettings.pixelFormat = GL_RGB;
        imageTextureLoG = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
        framebufferLoG->bindTexture(imageTextureLoG);
        framebufferLoG->bindRenderbuffer(sceneData.sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    }
}

void ClearViewRenderer_FacesUnified::reloadModelLoG() {
    if (!mesh) {
        return;
    }

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    mesh->getSurfaceData(triangleIndices, vertexPositions);

    meshShaderAttributesLoG = sgl::ShaderManager->createShaderAttributes(meshShaderLoG);
    meshShaderAttributesLoG->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*triangleIndices.size(), (void*)&triangleIndices.front(), sgl::INDEX_BUFFER);
    meshShaderAttributesLoG->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Add the position buffer.
    sgl::GeometryBufferPtr positionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);
    meshShaderAttributesLoG->addGeometryBuffer(
            positionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
}

void ClearViewRenderer_FacesUnified::createWeightTextureLoG() {
    float* textureData = new float[weightTextureSize.x * weightTextureSize.y];
    const float FACTOR_1 = -1.0f / (sgl::PI * std::pow(rhoLoG, 4.0f));
    const float FACTOR_2 = -1.0f / (2.0f * rhoLoG * rhoLoG);
    // Compute the LoG kernel weights.
    for (int iy = 0; iy < weightTextureSize.y; iy++) {
        for (int ix = 0; ix < weightTextureSize.x; ix++) {
            int x = ix - weightTextureSize.x / 2;
            int y = iy - weightTextureSize.y / 2;
            // https://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm
            float term3 = (x*x + y*y) * FACTOR_2;
            textureData[ix + iy*weightTextureSize.x] = FACTOR_1 * (1.0f + term3) * std::exp(term3);
        }
    }
    // Normalize the kernel weights.
    const int numEntries = weightTextureSize.x * weightTextureSize.y;
    float positiveEntriesAbsoluteSum = 0.0f, negativeEntriesAbsoluteSum = 0.0f;
    for (int i = 0; i < numEntries; i++) {
        float entry = textureData[i];
        if (entry >= 0.0f) {
            positiveEntriesAbsoluteSum += entry;
        } else {
            negativeEntriesAbsoluteSum += -entry;
        }
    }
    for (int i = 0; i < numEntries; i++) {
        float entry = textureData[i];
        if (entry >= 0.0f) {
            textureData[i] /= positiveEntriesAbsoluteSum;
        } else {
            textureData[i] /= negativeEntriesAbsoluteSum;
        }
    }
    for (int iy = 0; iy < weightTextureSize.y; iy++) {
        for (int ix = 0; ix < weightTextureSize.x; ix++) {
            std::cout << textureData[ix + iy*weightTextureSize.x] << "\t";
        }
        std::cout << std::endl;
    }
    sgl::TextureSettings textureSettings;
    textureSettings.pixelType = GL_FLOAT;
    textureSettings.pixelFormat = GL_RED;
    textureSettings.internalFormat = GL_R32F;
    weightTextureLoG = sgl::TextureManager->createTexture(
            textureData, weightTextureSize.x, weightTextureSize.y, textureSettings);
    delete[] textureData;
}

void ClearViewRenderer_FacesUnified::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity));

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
            || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4;
        //std::cout << "List size: " << expectedMaxDepthComplexity << std::endl;
        //std::cout << "Stack size: " << stackSize << std::endl;
        sgl::ShaderManager->addPreprocessorDefine("STACK_SIZE", sgl::toString(stackSize));
    }

    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void ClearViewRenderer_FacesUnified::reloadGatherShader() {
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
                {"MeshWireframe.Vertex", "MeshWireframe.Fragment.ClearView_ScreenSpace"});
    } else {
        gatherShader = sgl::ShaderManager->getShaderProgram(
                {"MeshWireframe.Vertex", "MeshWireframe.Fragment.ClearView_ObjectSpace"});
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

void ClearViewRenderer_FacesUnified::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if (sceneData.performanceMeasurer && !timerDataIsWritten) {
        if (timer) {
            delete timer;
        }
        timer = new sgl::TimerGL;
        sceneData.performanceMeasurer->setClearViewTimer(timer);
    }
}

void ClearViewRenderer_FacesUnified::setNewSettings(const SettingsMap& settings) {
    lineWidthBoostFactor = 1.0f;
    focusRadiusBoostFactor = 1.0f;
    settings.getValueOpt("lineWidthBoostFactor", lineWidthBoostFactor);
    settings.getValueOpt("focusRadiusBoostFactor", focusRadiusBoostFactor);

    if (settings.hasValue("sortingAlgorithmMode")) {
        sortingAlgorithmMode = (SortingAlgorithmMode)settings.getIntValue("sortingAlgorithmMode");
        setSortingAlgorithmDefine();
        reloadResolveShader();
    }

    if (mesh) {
        const float avgCellVolumeCbrt = std::cbrt(mesh->getAverageCellVolume());
        lineWidth = lineWidthBoostFactor * glm::clamp(
                avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
        focusRadius = focusRadiusBoostFactor * glm::clamp(
                avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
    }
}

void ClearViewRenderer_FacesUnified::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (mesh->getNumCells() > 1e6) { // > 1m elements
        newMeshLargeMeshMode = MESH_SIZE_LARGE;
    }
    if (newMeshLargeMeshMode != largeMeshMode) {
        largeMeshMode = newMeshLargeMeshMode;
        expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][0];
        expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][1];
        reallocateFragmentBuffer();
        reloadResolveShader();
    }
}

void ClearViewRenderer_FacesUnified::uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh) {
    if (isNewMesh) {
        Pickable::focusPoint = glm::vec3(0.0f);
    }
    frameCounter = 0;

    mesh = meshIn;
    updateLargeMeshMode();
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
    singularEdgeColorMapWidget.generateSingularityStructureInformation(mesh);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    hexahedralCellFacesBuffer = sgl::GeometryBufferPtr();

    // Load the unified data for the focus and context region.
    std::vector<uint32_t> indices;
    std::vector<HexahedralCellFaceUnified> hexahedralCellFaces;
    if (useWeightedVertexAttributes) {
        mesh->getSurfaceDataWireframeFacesUnified_AttributePerVertex(
                indices, hexahedralCellFaces, maxLodValue, useVolumeWeighting, lodSettings);
    } else {
        mesh->getSurfaceDataWireframeFacesUnified_AttributePerCell(
                indices, hexahedralCellFaces, maxLodValue, lodSettings);
    }

    size_t modelBufferSizeBytes = indices.size() * sizeof(uint32_t)
            + hexahedralCellFaces.size() * sizeof(HexahedralCellFaceUnified);
    sgl::Logfile::get()->writeInfo(
            std::string() + "GPU model buffer size MiB: "
            + std::to_string(modelBufferSizeBytes / 1024.0 / 1024.0));

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), (void*)&indices.front(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell faces.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(HexahedralCellFaceUnified), (void*)&hexahedralCellFaces.front(),
            sgl::SHADER_STORAGE_BUFFER);

    reloadModelLoG();

    dirty = false;
    reRender = true;
    hasHitInformation = false;
}

void ClearViewRenderer_FacesUnified::reallocateFragmentBuffer() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    fragmentBufferSize = size_t(expectedAvgDepthComplexity) * size_t(width) * size_t(height);
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
}

void ClearViewRenderer_FacesUnified::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    screenSpaceLensPixelRadius = std::min(width, height) * screenSpaceLensPixelRadiusWindowFactor;
    focusPointScreen = glm::vec2(width / 2.0f, height / 2.0f);
    windowWidth = width;
    windowHeight = height;

    reallocateFragmentBuffer();

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * width * height;
    startOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);

    reloadTexturesLoG();
}

void ClearViewRenderer_FacesUnified::setUniformData() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
    glm::vec3 lookingDirection(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, startOffsetBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShader->setUniform("viewportW", width);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    if (gatherShader->hasUniform("lookingDirection")) {
        gatherShader->setUniform("lookingDirection", lookingDirection);
    }
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
    if (gatherShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherShader->setUniform("foregroundColor", foregroundColor);
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
    shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgramSurface->setUniform("color", focusPointColor);

    resolveShader->setUniform("viewportW", width);
    clearShader->setUniform("viewportW", width);

    shaderFullScreenBlitLoG->setUniform("color", sgl::Color(255, 255, 255));
    shaderAttributesLoG->getShaderProgram()->setUniform("clearColor", sceneData.clearColor);
    if (shaderAttributesLoG->getShaderProgram()->hasUniform("weightTexture")) {
        shaderAttributesLoG->getShaderProgram()->setUniform("weightTexture", weightTextureLoG, 3);
    }
    shaderAttributesLoG->getShaderProgram()->setUniform(
            "weightTextureSize", glm::ivec2(weightTextureSize.x, weightTextureSize.y));
    if (outlineMode == OUTLINE_MODE_DEPTH) {
        depthTextureShaderLoG->setUniform("depthTexture", depthStencilTextureLoG, 2);
        depthTextureShaderLoG->setUniform("depthTextureSize", glm::ivec2(width, height));
        depthTextureShaderLoG->setUniform("zNear", sceneData.camera->getNearClipDistance());
        depthTextureShaderLoG->setUniform("zFar", sceneData.camera->getFarClipDistance());
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        colorTextureShaderLoG->setUniform("imageTexture", imageTextureLoG, 2);
        colorTextureShaderLoG->setUniform("imageTextureSize", glm::ivec2(width, height));
    }
}

void ClearViewRenderer_FacesUnified::clear() {
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

void ClearViewRenderer_FacesUnified::gather() {
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
    if (useWeightedVertexAttributes) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(shaderAttributes);
    if (useWeightedVertexAttributes) {
        glEnable(GL_CULL_FACE);
    }
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Render the focus point.
    if (!useScreenSpaceLens) {
        //sgl::Renderer->setModelMatrix(sgl::matrixTranslation(focusPoint));
        //sgl::Renderer->render(focusPointShaderAttributes);
        //sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        //glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void ClearViewRenderer_FacesUnified::renderLaplacianOfGaussianContours() {
    if (outlineMode == OUTLINE_MODE_DEPTH) {
        sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
        sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        sgl::Renderer->bindFBO(framebufferLoG);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glDisable(GL_STENCIL_TEST);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        sgl::Renderer->clearFramebuffer(GL_DEPTH_BUFFER_BIT);
        sgl::Renderer->render(meshShaderAttributesLoG);

        sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
        sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glEnable(GL_CULL_FACE);
        sgl::Renderer->bindFBO(sceneData.framebuffer);
        sgl::Renderer->render(shaderAttributesLoG);
        glDisable(GL_DEPTH_TEST);
    } else if (outlineMode == OUTLINE_MODE_STENCIL) {
        sgl::Renderer->bindFBO(framebufferLoG);
        sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT, sgl::Color(0, 0, 0));
        sgl::Renderer->render(shaderAttributesFullScreenBlitLoG);

        glDisable(GL_STENCIL_TEST);
        sgl::Renderer->bindFBO(sceneData.framebuffer);
        sgl::Renderer->render(shaderAttributesLoG);
        glEnable(GL_STENCIL_TEST);
    }
}

void ClearViewRenderer_FacesUnified::resolve() {
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

    renderLaplacianOfGaussianContours();

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void ClearViewRenderer_FacesUnified::renderGui() {
    ClearViewRenderer::renderGui();

    if (highlightEdges && useSingularEdgeColorMap && singularEdgeColorMapWidget.renderGui()) {
        reRender = true;
    }
}

void ClearViewRenderer_FacesUnified::childClassRenderGuiBegin() {
    if (ImGui::Checkbox("Use Screen Space Lens", &useScreenSpaceLens)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (useScreenSpaceLens && ImGui::SliderFloat(
            "Lens Pixel Radius", &screenSpaceLensPixelRadius, 0.0f, 2*std::max(windowWidth, windowHeight))) {
        reRender = true;
    }
}

void ClearViewRenderer_FacesUnified::childClassRenderGuiEnd() {
    if (ImGui::SliderFloat("LOD Value Focus", &selectedLodValueFocus, 0.0f, 1.0f)) {
        if (selectedLodValueFocus < selectedLodValueContext) {
            selectedLodValueContext = selectedLodValueFocus;
        }
        reRender = true;
    }
    if (ImGui::SliderFloat("LOD Value Context", &selectedLodValueContext, 0.0f, 1.0f)) {
        if (selectedLodValueFocus < selectedLodValueContext) {
            selectedLodValueFocus = selectedLodValueContext;
        }
        reRender = true;
    }
    if (ImGui::SliderFloat("Important Lines", &importantLineBoostFactor, 0.0f, 1.0f)) {
        reRender = true;
    }
    if (ImGui::Checkbox("Accentuate Edges", &accentuateAllEdges)) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
    if (ImGui::Checkbox("Per Line Attributes", &usePerLineAttributes)) {
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
    if (ImGui::Combo(
            "Outline Mode", (int*)&outlineMode, OUTLINE_MODE_NAMES, NUM_OUTLINE_MODES)) {
        if (outlineMode != OUTLINE_MODE_NONE) {
            if (outlineMode == OUTLINE_MODE_DEPTH) {
                shaderAttributesLoG = shaderAttributesLoG->copy(depthTextureShaderLoG);
            } else if (outlineMode == OUTLINE_MODE_STENCIL) {
                shaderAttributesLoG = shaderAttributesLoG->copy(colorTextureShaderLoG);
            }
            reloadTexturesLoG();
            reloadModelLoG();
        }
        reRender = true;
    }
    if (ImGui::Combo(
            "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        setSortingAlgorithmDefine();
        reloadResolveShader();
        reRender = true;
    }
    if (ImGui::Button("Reload Shader")) {
        reloadGatherShader();
        if (shaderAttributes) {
            shaderAttributes = shaderAttributes->copy(gatherShader);
        }
        reRender = true;
    }
}
