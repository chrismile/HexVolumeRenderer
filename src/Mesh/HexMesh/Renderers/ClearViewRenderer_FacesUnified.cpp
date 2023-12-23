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
#include <Graphics/OpenGL/SystemGL.hpp>
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

ClearViewRenderer_FacesUnified::ClearViewRenderer_FacesUnified(
        SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
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

    /*
     * In Vulkan, we could use the minimum out of physicalDeviceVulkan11Properties.maxMemoryAllocationSize and
     * physicalDeviceProperties.limits.maxStorageBufferRange. On NVIDIA hardware, this seems to be 4GiB - 1B, on AMD
     * hardware it seems to be 2GiB. We will just assume OpenGL allows allocations of size 4GiB.
     */
    maxStorageBufferSize = (1ull << 32ull) - 1ull;
    double availableMemoryFactor = 28.0 / 32.0;
    maxDeviceMemoryBudget = size_t(double(sgl::SystemGL::get()->getFreeMemoryBytes()) * availableMemoryFactor);
    // We only have binding {1,2,3,4,5}, so limit the maximum budget.
    maxDeviceMemoryBudget = std::min(maxDeviceMemoryBudget, maxStorageBufferSize * 5);

    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");
    sgl::ShaderManager->addPreprocessorDefine("DEPTH_TYPE_UINT", "");

    updateDepthCueMode();
    sgl::ShaderManager->addPreprocessorDefine("COMPUTE_DEPTH_CUES_GPU", "");
    computeDepthValuesShaderProgram = sgl::ShaderManager->getShaderProgram({"ComputeDepthValues.Compute"});
    minMaxReduceDepthShaderProgram = sgl::ShaderManager->getShaderProgram({"MinMaxReduceDepth.Compute"});

    shaderProgramSurface = sgl::ShaderManager->getShaderProgram(
            {"MeshShader.Vertex.Plain", "MeshShader.Fragment.Plain"});
    reloadSphereRenderData();

    reloadGatherShader(false);
    reloadResolveShader();
    reloadFocusOutlineGatherShader();
    createFocusOutlineRenderingData();
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClear.Vertex", "LinkedListClear.Fragment"});

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

ClearViewRenderer_FacesUnified::~ClearViewRenderer_FacesUnified() {
    if (sceneData.performanceMeasurer && !timerDataIsWritten && timer) {
        delete timer;
        sceneData.performanceMeasurer->setClearViewTimer(nullptr);
    }

    sgl::ShaderManager->removePreprocessorDefine("DEPTH_TYPE_UINT");

    if (useDepthCues) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
    sgl::ShaderManager->removePreprocessorDefine("COMPUTE_DEPTH_CUES_GPU");
    if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
        sgl::ShaderManager->removePreprocessorDefine("FRAGMENT_BUFFER_ARRAY");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGMENT_BUFFERS");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGS_PER_BUFFER");
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

void ClearViewRenderer_FacesUnified::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity));

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
            || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4;
        sgl::ShaderManager->addPreprocessorDefine("STACK_SIZE", sgl::toString(stackSize));
    }

    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void ClearViewRenderer_FacesUnified::reloadGatherShader(bool copyShaderAttributes) {
    sgl::ShaderManager->invalidateShaderCache();
    std::string lineRenderingStyleDefineName = "LINE_RENDERING_STYLE_HALO";
    sgl::ShaderManager->addPreprocessorDefine(lineRenderingStyleDefineName, "");
    if (modulateLineThicknessByDepth) {
        sgl::ShaderManager->addPreprocessorDefine("MODULATE_LINE_THICKNESS_BY_DEPTH", "");
    }
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
    if (showFocusFaces) {
        sgl::ShaderManager->addPreprocessorDefine("RENDER_FOCUS_FACES", "");
    }
    if (showFocusFaces && useLighting) {
        sgl::ShaderManager->addPreprocessorDefine("USE_LIGHTING", "");
    }
    //if (useFocusOutline) {
    //    sgl::ShaderManager->addPreprocessorDefine("USE_FOCUS_OUTLINE", "");
    //}

    if (useScreenSpaceLens) {
        gatherShader = sgl::ShaderManager->getShaderProgram(
                {"HexMeshUnified.Vertex", "HexMeshUnified.Fragment.ClearView_ScreenSpace"});
    } else {
        gatherShader = sgl::ShaderManager->getShaderProgram(
                {"HexMeshUnified.Vertex", "HexMeshUnified.Fragment.ClearView_ObjectSpace"});
    }

    sgl::ShaderManager->removePreprocessorDefine(lineRenderingStyleDefineName);
    if (modulateLineThicknessByDepth) {
        sgl::ShaderManager->removePreprocessorDefine("MODULATE_LINE_THICKNESS_BY_DEPTH");
    }
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
    if (showFocusFaces) {
        sgl::ShaderManager->removePreprocessorDefine("RENDER_FOCUS_FACES");
    }
    if (showFocusFaces && useLighting) {
        sgl::ShaderManager->removePreprocessorDefine("USE_LIGHTING");
    }
    //if (useFocusOutline) {
    //    sgl::ShaderManager->removePreprocessorDefine("USE_FOCUS_OUTLINE");
    //}

    if (copyShaderAttributes && shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
}

void ClearViewRenderer_FacesUnified::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if (sceneData.performanceMeasurer) {
        useFocusOutline = false;
    }
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
    bool reloadLineWidth = settings.getValueOpt("lineWidthBoostFactor", lineWidthBoostFactor);
    bool reloadFocusRadius = settings.getValueOpt("focusRadiusBoostFactor", focusRadiusBoostFactor);

    if (settings.hasValue("sortingAlgorithmMode")) {
        sortingAlgorithmMode = (SortingAlgorithmMode)settings.getIntValue("sortingAlgorithmMode");
        setSortingAlgorithmDefine();
        reloadResolveShader();
    }

    if (hexMesh && (reloadLineWidth || reloadFocusRadius)) {
        const float avgCellVolumeCbrt = std::cbrt(hexMesh->getAverageCellVolume());
        if (reloadLineWidth) {
            lineWidth = lineWidthBoostFactor * glm::clamp(
                    avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
        }
        if (reloadFocusRadius) {
            focusRadius = focusRadiusBoostFactor * glm::clamp(
                    avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
        }
    }

    // Replay script data.
    bool shallReloadGatherShader = false;
    if (settings.getValueOpt("line_width", lineWidth)) {
        manualLineWidthSet = true;
    }
    settings.getValueOpt("modulate_line_thickness_by_depth", modulateLineThicknessByDepth);
    settings.getValueOpt("lod_value_context", selectedLodValueContext);
    settings.getValueOpt("lod_value_focus", selectedLodValueFocus);
    if (settings.getValueOpt("use_screen_space_lens", useScreenSpaceLens)) {
        shallReloadGatherShader = true;
    }
    settings.getValueOpt("screen_space_lens_radius", screenSpaceLensPixelRadius);
    if (settings.getValueOpt("object_space_lens_radius", focusRadius)) {
        manualFocusRadiusSet = true;
    }
    settings.getValueOpt("focus_outline_width", focusOutlineWidth);
    settings.getValueOpt("clip_focus_outline", clipFocusOutline);
    settings.getValueOpt("focus_outline_color", focusOutlineColor);
    settings.getValueOpt("important_lines", importantLineBoostFactor);
    settings.getValueOpt("important_cells", importantCellFactor);

    glm::vec2 screenSpaceLensPositionRelative;
    if (settings.getValueOpt("screen_space_lens_position", screenSpaceLensPositionRelative)) {
        //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        //int height = window->getHeight();
        int height = (*sceneData.sceneTexture)->getH();
        focusPointScreen.x = (height * screenSpaceLensPositionRelative.x + (windowWidth - 1.0f)) / 2.0f;
        focusPointScreen.y = height * (screenSpaceLensPositionRelative.y + 1.0f) / 2.0f;
    }
    settings.getValueOpt("object_space_lens_position", focusPoint);

    LodSettings lodSettings = this->lodSettings;
    settings.getValueOpt("lod_merge_factor", lodSettings.lodMergeFactor);
    settings.getValueOpt("use_volume_and_area_measures", lodSettings.useVolumeAndAreaMeasures);
    settings.getValueOpt("use_weights_for_merging", lodSettings.useWeightsForMerging);
    settings.getValueOpt("use_num_cells_or_volume", lodSettings.useNumCellsOrVolume);
    if (lodSettings != this->lodSettings) {
        this->lodSettings = lodSettings;
        if (hexMesh) {
            uploadVisualizationMapping(hexMesh, false);
        }
    }

    if (shallReloadGatherShader) {
        reloadGatherShader(true);
    }
}

void ClearViewRenderer_FacesUnified::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (hexMesh->getNumCells() > 1e7) { // > 10m elements
        newMeshLargeMeshMode = MESH_SIZE_VERY_LARGE;
    } else if (hexMesh->getNumCells() > 1e6) { // > 1m elements
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

    hexMesh = meshIn;
    updateLargeMeshMode();
    const float avgCellVolumeCbrt = std::cbrt(meshIn->getAverageCellVolume());
    // Higher radius for recording...
    if (!manualLineWidthSet) {
        lineWidth = lineWidthBoostFactor * glm::clamp(
                avgCellVolumeCbrt * LINE_WIDTH_VOLUME_CBRT_FACTOR, MIN_LINE_WIDTH_AUTO, MAX_LINE_WIDTH_AUTO);
    } else {
        manualLineWidthSet = false;
    }
    if (!manualFocusRadiusSet) {
        focusRadius = focusRadiusBoostFactor * glm::clamp(
                avgCellVolumeCbrt * FOCUS_RADIUS_VOLUME_CBRT_FACTOR, MIN_FOCUS_RADIUS_AUTO, MAX_FOCUS_RADIUS_AUTO);
    } else {
        manualFocusRadiusSet = false;
    }
    reloadSphereRenderData();

    // Update the depth cue data.
    filteredCellVertices.clear();
    filteredCellVerticesBuffer = sgl::GeometryBufferPtr();
    depthMinMaxBuffers[0] = sgl::GeometryBufferPtr();
    depthMinMaxBuffers[1] = sgl::GeometryBufferPtr();
    if (useDepthCues) {
        updateDepthCueGeometryData();
    }

    // Don't highlight singular edges when we have far too many of them.
    bool tooMuchSingularEdgeModeNewMesh = meshIn->getNumberOfSingularEdges(true, 1) > 10000u;
    if (tooMuchSingularEdgeModeNewMesh != tooMuchSingularEdgeMode) {
        tooMuchSingularEdgeMode = tooMuchSingularEdgeModeNewMesh;
        reloadGatherShader(false);
    }

    // Get the information about the singularity structure.
    singularEdgeColorMapWidget.generateSingularityStructureInformation(hexMesh);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    hexahedralCellFacesBuffer = sgl::GeometryBufferPtr();
    hexahedralCellVerticesBuffer = sgl::GeometryBufferPtr();
    hexahedralCellEdgesBuffer = sgl::GeometryBufferPtr();
    hexahedralCellFacesCellLinksBuffer = sgl::GeometryBufferPtr();
    hexahedralCellsBuffer = sgl::GeometryBufferPtr();

    // Load the unified data for the focus and context region.
    std::vector<uint32_t> indices;
    std::vector<HexahedralCellFaceUnified> hexahedralCellFaces;
    std::vector<HexahedralCellVertexUnified> hexahedralCellVertices;
    std::vector<HexahedralCellEdgeUnified> hexahedralCellEdges;
    std::vector<glm::uvec2> hexahedralCellFacesCellLinks;
    std::vector<float> hexahedralCells;
    hexMesh->getSurfaceDataWireframeFacesUnified_AttributePerVertex(
            indices, hexahedralCellFaces, hexahedralCellVertices, hexahedralCellEdges,
            hexahedralCellFacesCellLinks, hexahedralCells,
            showFocusFaces, maxLodValue, useVolumeWeighting, lodSettings);

    size_t modelBufferSizeBytesNoCells =
            indices.size() * sizeof(uint32_t)
            + hexahedralCellFaces.size() * sizeof(HexahedralCellFaceUnified)
            + hexahedralCellVertices.size() * sizeof(HexahedralCellVertexUnified)
            + hexahedralCellEdges.size() * sizeof(HexahedralCellEdgeUnified);
    sgl::Logfile::get()->writeInfo(
            std::string() + "GPU model buffer size MiB (no solid face rendering): "
            + std::to_string(modelBufferSizeBytesNoCells / 1024.0 / 1024.0));

    size_t modelBufferSizeBytes =
            indices.size() * sizeof(uint32_t)
            + hexahedralCellFaces.size() * sizeof(HexahedralCellFaceUnified)
            + hexahedralCellVertices.size() * sizeof(HexahedralCellVertexUnified)
            + hexahedralCellEdges.size() * sizeof(HexahedralCellEdgeUnified)
            + hexahedralCellFacesCellLinks.size() * sizeof(glm::uvec2)
            + hexahedralCells.size() * sizeof(float);
    sgl::Logfile::get()->writeInfo(
            std::string() + "GPU model buffer size MiB: "
            + std::to_string(modelBufferSizeBytes / 1024.0 / 1024.0));

    shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);

    // Add the index buffer.
    sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*indices.size(), indices.data(), sgl::INDEX_BUFFER);
    shaderAttributes->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);

    // Create an SSBO for the hexahedral cell data.
    hexahedralCellFacesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellFaces.size()*sizeof(HexahedralCellFaceUnified), hexahedralCellFaces.data(),
            sgl::SHADER_STORAGE_BUFFER);
    hexahedralCellVerticesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellVertices.size()*sizeof(HexahedralCellVertexUnified), hexahedralCellVertices.data(),
            sgl::SHADER_STORAGE_BUFFER);
    hexahedralCellEdgesBuffer = sgl::Renderer->createGeometryBuffer(
            hexahedralCellEdges.size()*sizeof(HexahedralCellEdgeUnified), hexahedralCellEdges.data(),
            sgl::SHADER_STORAGE_BUFFER);
    if (showFocusFaces) {
        hexahedralCellFacesCellLinksBuffer = sgl::Renderer->createGeometryBuffer(
                hexahedralCellFacesCellLinks.size()*sizeof(glm::uvec2), hexahedralCellFacesCellLinks.data(),
                sgl::SHADER_STORAGE_BUFFER);
        hexahedralCellsBuffer = sgl::Renderer->createGeometryBuffer(
                hexahedralCells.size()*sizeof(float), hexahedralCells.data(),
                sgl::SHADER_STORAGE_BUFFER);
    }

    reloadModelEdgeDetection(hexMesh);

    dirty = false;
    reRender = true;
    hasHitInformation = false;
}

void ClearViewRenderer_FacesUnified::reallocateFragmentBuffer() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();

    fragmentBufferSize = size_t(expectedAvgDepthComplexity) * size_t(width) * size_t(height);
    size_t fragmentBufferSizeBytes = sizeof(LinkedListFragmentNode) * fragmentBufferSize;

    // Delete old data first (-> refcount 0)
    fragmentBuffers = {};
    fragmentBuffer = {};

    // We only need buffer arrays when the maximum allocation is larger than our budget.
    if (maxDeviceMemoryBudget < maxStorageBufferSize) {
        fragmentBufferMode = FragmentBufferMode::BUFFER;
    }

    if (fragmentBufferMode == FragmentBufferMode::BUFFER) {
        sgl::ShaderManager->removePreprocessorDefine("FRAGMENT_BUFFER_ARRAY");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGMENT_BUFFERS");
        sgl::ShaderManager->removePreprocessorDefine("NUM_FRAGS_PER_BUFFER");
    } else {
        sgl::ShaderManager->addPreprocessorDefine("FRAGMENT_BUFFER_ARRAY", "");
        sgl::ShaderManager->addPreprocessorDefine(
                "NUM_FRAGMENT_BUFFERS", std::to_string(cachedNumFragmentBuffers));
        sgl::ShaderManager->addPreprocessorDefine(
                "NUM_FRAGS_PER_BUFFER", std::to_string(maxStorageBufferSize / 12ull) + "u");
    }

    // We only need buffer arrays when the maximum allocation is larger than our budget.
    if (maxDeviceMemoryBudget < maxStorageBufferSize) {
        reloadGatherShader(true);
        reloadResolveShader();
    }
    size_t maxSingleBufferAllocation = std::min(maxDeviceMemoryBudget, maxStorageBufferSize);

    if (fragmentBufferMode == FragmentBufferMode::BUFFER) {
        if (fragmentBufferSizeBytes > maxSingleBufferAllocation) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Fragment buffer size was larger than maximum allocation size ("
                    + std::to_string(maxSingleBufferAllocation) + "). Clamping to maximum allocation size.",
                    false);
            fragmentBufferSize = maxSingleBufferAllocation / sizeof(LinkedListFragmentNode);
            fragmentBufferSizeBytes = fragmentBufferSize * sizeof(LinkedListFragmentNode);
        } else {
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Fragment buffer size GiB: "
                    + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
        }

        numFragmentBuffers = 1;
        cachedNumFragmentBuffers = 1;
        fragmentBuffer = sgl::Renderer->createGeometryBuffer(
                fragmentBufferSizeBytes, nullptr, sgl::SHADER_STORAGE_BUFFER);
    } else {
        if (fragmentBufferSizeBytes > maxDeviceMemoryBudget) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Fragment buffer size was larger than maximum allocation size ("
                    + std::to_string(maxDeviceMemoryBudget) + "). Clamping to maximum allocation size.",
                    false);
            fragmentBufferSize = maxDeviceMemoryBudget / 12ull;
            fragmentBufferSizeBytes = fragmentBufferSize * 12ull;
        } else {
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Fragment buffer size GiB: "
                    + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
        }

        numFragmentBuffers = sgl::sizeceil(fragmentBufferSizeBytes, maxStorageBufferSize);
        size_t fragmentBufferSizeBytesLeft = fragmentBufferSizeBytes;
        for (size_t i = 0; i < numFragmentBuffers; i++) {
            fragmentBuffers.emplace_back(sgl::Renderer->createGeometryBuffer(
                    std::min(fragmentBufferSizeBytesLeft, maxStorageBufferSize), nullptr, sgl::SHADER_STORAGE_BUFFER));
            fragmentBufferSizeBytesLeft -= maxStorageBufferSize;
        }

        if (numFragmentBuffers != cachedNumFragmentBuffers) {
            cachedNumFragmentBuffers = numFragmentBuffers;
            reloadGatherShader(true);
            reloadResolveShader();
        }
    }

    if (sceneData.performanceMeasurer) {
        sceneData.performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }
}

void ClearViewRenderer_FacesUnified::onResolutionChanged() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    //int height = window->getHeight();
    int width = (*sceneData.sceneTexture)->getW();
    int height = (*sceneData.sceneTexture)->getH();
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

    reloadTexturesEdgeDetection();
}

void ClearViewRenderer_FacesUnified::setUniformData() {
    //sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    //int width = window->getWidth();
    int width = (*sceneData.sceneTexture)->getW();

    glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
    glm::vec3 lookingDirection(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

    sgl::ShaderManager->bindShaderStorageBuffer(0, startOffsetBuffer);
    if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
        for (size_t i = 0; i < numFragmentBuffers; i++) {
            sgl::ShaderManager->bindShaderStorageBuffer(int(i + 1), fragmentBuffers.at(i));
        }
    } else {
        sgl::ShaderManager->bindShaderStorageBuffer(1, fragmentBuffer);
    }
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShader->setUniform("viewportW", width);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
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
    if (gatherShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherShader->setUniform("foregroundColor", foregroundColor);
    }

    //if (useFocusOutline) {
    //    gatherShader->setUniform("focusOutlineColor", focusOutlineColor);
    //}
    gatherShader->setUniform("fieldOfViewY", sceneData.camera->getFOVy());
    gatherShader->setUniform("viewportSize", glm::ivec2(windowWidth, windowHeight));

    if (useScreenSpaceLens) {
        gatherShader->setUniform("sphereCenterScreen", focusPointScreen);
        gatherShader->setUniform("sphereRadiusPixels", std::max(screenSpaceLensPixelRadius, 0.0f));
    } else {
        gatherShader->setUniform("sphereCenter", focusPoint);
        gatherShader->setUniform("sphereRadius", std::max(focusRadius, 0.0f));
    }

    if (showFocusFaces) {
        gatherShader->setUniform("importantCellFactor", importantCellFactor);
    }

    if (useDepthCues && hexMesh) {
        setUniformDataDepthCues(gatherShader);
    }

    shaderProgramSurface->setUniform("viewportW", width);
    shaderProgramSurface->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    shaderProgramSurface->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgramSurface->setUniform("color", focusPointColor);

    resolveShader->setUniform("viewportW", width);
    clearShader->setUniform("viewportW", width);

    setUniformDataEdgeDetection();
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
    sgl::ShaderManager->bindShaderStorageBuffer(7, hexahedralCellVerticesBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(8, hexahedralCellEdgesBuffer);
    if (showFocusFaces) {
        sgl::ShaderManager->bindShaderStorageBuffer(9, hexahedralCellFacesCellLinksBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(10, hexahedralCellsBuffer);
    }
    if (useWeightedVertexAttributes) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(shaderAttributes);
    if (useWeightedVertexAttributes) {
        glEnable(GL_CULL_FACE);
    }
    if (useFocusOutline) {
        if (useStencilBuffer && clipFocusOutline) {
            glStencilFunc(GL_EQUAL, 1, 0xFF);
            glStencilMask(0x00);
        }
        renderFocusOutline(fragmentBufferSize);
        if (useStencilBuffer && clipFocusOutline) {
            glStencilFunc(GL_ALWAYS, 1, 0xFF);
            glStencilMask(0xFF);
        }
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

    renderEdgeDetectionContours();

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
        reloadGatherShader(true);
        reRender = true;
    }
    if (useScreenSpaceLens && ImGui::SliderFloat(
            "Lens Pixel Radius", &screenSpaceLensPixelRadius,
            0.0f, 2*std::max(windowWidth, windowHeight))) {
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
    if (ImGui::Checkbox("Show Focus Faces", &showFocusFaces)) {
        if (hexMesh) {
            uploadVisualizationMapping(hexMesh, false);
        }
        reloadGatherShader(true);
        reRender = true;
    }
    if (showFocusFaces && ImGui::SliderFloat(
            "Important Cells", &importantCellFactor, 0.0f, 1.0f)) {
        reRender = true;
    }
    if (showFocusFaces && ImGui::Checkbox("Use Lighting", &useLighting)) {
        reloadGatherShader(true);
        reRender = true;
    }
    if (ImGui::Checkbox("Accentuate Edges", &accentuateAllEdges)) {
        reloadGatherShader(true);
        reRender = true;
    }
    if (ImGui::Checkbox("Per Line Attributes", &usePerLineAttributes)) {
        reloadGatherShader(true);
        reRender = true;
    }
    if (ImGui::Checkbox("Use Singular Edge Color Map", &useSingularEdgeColorMap)) {
        reloadGatherShader(true);
        reRender = true;
    }
    if (ImGui::Checkbox("Highlight Edges", &highlightEdges)) {
        reloadGatherShader(true);
        reRender = true;
    }
    reRender = renderGuiEdgeDetection() || reRender;
    if (ImGui::Combo(
            "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        setSortingAlgorithmDefine();
        reloadResolveShader();
        reRender = true;
    }
    if (ImGui::Combo(
            "Fragment Buffer Mode", (int*)&fragmentBufferMode,
            FRAGMENT_BUFFER_MODE_NAMES, IM_ARRAYSIZE(FRAGMENT_BUFFER_MODE_NAMES))) {
        reallocateFragmentBuffer();
        reloadResolveShader();
        reloadGatherShader(true);
        reRender = true;
    }
    if (ImGui::Button("Reload Shader")) {
        reloadGatherShader(true);
        reRender = true;
    }
}
