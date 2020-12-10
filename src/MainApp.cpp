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

#define GLM_ENABLE_EXPERIMENTAL
#include <climits>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <thread>

#include <glm/gtx/color_space.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <GL/glew.h>
#include <boost/algorithm/string/predicate.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include <Utils/AppSettings.hpp>
#include <Utils/Timer.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/Events/EventManager.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <Math/Math.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>

#include "Mesh/HexMesh/Loaders/VtkLoader.hpp"
#include "Mesh/HexMesh/Loaders/MeshLoader.hpp"
#include "Mesh/HexMesh/Loaders/HexaLabDatasets.hpp"
#include "Mesh/Filters/PlaneFilter.hpp"
#include "Mesh/Filters/PeelingFilter.hpp"
#include "Mesh/Filters/QualityFilter.hpp"
#include "Mesh/HexMesh/Renderers/SurfaceRenderer.hpp"
#include "Mesh/HexMesh/Renderers/WireframeRenderer.hpp"
#include "Mesh/HexMesh/Renderers/WireframeRenderer_Faces.hpp"
#include "Mesh/HexMesh/Renderers/VolumeRenderer_Faces.hpp"
#include "Mesh/HexMesh/Renderers/VolumeRenderer_FacesSlim.hpp"
#include "Mesh/HexMesh/Renderers/VolumeRenderer_Volume.hpp"
#include "Mesh/HexMesh/Renderers/ClearViewRenderer_Faces.hpp"
#include "Mesh/HexMesh/Renderers/ClearViewRenderer_FacesUnified.hpp"
#include "Mesh/HexMesh/Renderers/ClearViewRenderer_Volume.hpp"
#include "Mesh/HexMesh/Renderers/ClearViewRenderer_Volume2.hpp"
#include "Mesh/HexMesh/Renderers/DepthComplexityRenderer.hpp"
#include "Mesh/HexMesh/Renderers/SingularityRenderer.hpp"
#include "Mesh/HexMesh/Renderers/BaseComplexLineRenderer.hpp"
#include "Mesh/HexMesh/Renderers/BaseComplexSurfaceRenderer.hpp"
#include "Mesh/HexMesh/Renderers/PartitionLineRenderer.hpp"
#include "Mesh/HexMesh/Renderers/LodLineRendererPerFragment.hpp"
#include "Mesh/HexMesh/Renderers/LodLineRenderer.hpp"
#include "Mesh/HexMesh/Renderers/LodLinePreviewRenderer.hpp"
#include "Mesh/HexMesh/Renderers/LodLinePreviewRenderer_Sheets.hpp"
#include "Mesh/HexMesh/Renderers/LodLinePreviewRenderer_SheetsFaces.hpp"
#include "Mesh/HexMesh/Renderers/SingularityTypeCounterRenderer.hpp"
#include "Mesh/HexMesh/Renderers/LineDensityControlRenderer.hpp"
#include "Mesh/HexMesh/Renderers/HexSheetRenderer.hpp"
#ifdef USE_EMBREE
#include "Mesh/HexMesh/Renderers/Intersection/RayMeshIntersection_Embree.hpp"
#endif
#include "MainApp.hpp"

void openglErrorCallback() {
    std::cerr << "Application callback" << std::endl;
}

MainApp::MainApp()
#ifdef USE_EMBREE
        : rayMeshIntersection(new RayMeshIntersection_Embree(camera)),
#else
        : rayMeshIntersection(new RayMeshIntersection_NanoRT(camera)),
#endif
          sceneData(
                  sceneFramebuffer, sceneTexture, sceneDepthRBO, camera, clearColor, performanceMeasurer,
                  recording, useCameraFlight, *rayMeshIntersection) {
#ifdef USE_STEAMWORKS
    steamworks.initialize();
#endif

    clearColor = sgl::Color(0, 0, 0, 255);
    clearColorSelection = ImColor(clearColor.getColorRGBA());
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
    colorLegendWidget.setClearColor(clearColor);

    sgl::Renderer->setErrorCallback(&openglErrorCallback);
    sgl::Renderer->setDebugVerbosity(sgl::DEBUG_OUTPUT_CRITICAL_ONLY);
    resolutionChanged(sgl::EventPtr());

    selectedQualityMeasure = QUALITY_MEASURE_SCALED_JACOBIAN;
    changeQualityMeasureType();

    meshLoaderMap.insert(std::make_pair("vtk", new VtkLoader));
    meshLoaderMap.insert(std::make_pair("mesh", new MeshLoader));
    meshFilters.push_back(new PlaneFilter);
    meshFilters.push_back(new PeelingFilter);
    meshFilters.push_back(new QualityFilter);

    if (usePerformanceMeasurementMode) {
        useCameraFlight = true;
    }
    if (useCameraFlight && recording) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        window->setWindowSize(recordingResolution.x, recordingResolution.y);
        realTimeCameraFlight = false;
        transferFunctionWindow.loadFunctionFromFile("Data/TransferFunctions/Standard_PerVertex.xml");
        loadHexahedralMesh(
                "Data/Meshes/2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation/anc101_a1.mesh");
        renderingMode = RENDERING_MODE_CLEAR_VIEW_FACES_UNIFIED;
    }

    if (!recording && !usePerformanceMeasurementMode) {
        // Just for convenience...
        int desktopWidth = 0;
        int desktopHeight = 0;
        int refreshRate = 60;
        if (desktopWidth == 3840 && desktopHeight == 2160) {
            sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
            window->setWindowSize(2186, 1358);
        }
    }

    setRenderers();

    customMeshFileName = sgl::FileUtils::get()->getUserDirectory();
    hexaLabDataSetsDownloaded = sgl::FileUtils::get()->exists(meshDirectory + "index.json");
    loadAvailableDataSetSources();

    colorLegendWidget.setTransferFunctionColorMap(
            transferFunctionWindow.getTransferFunctionMap_sRGB());

    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    usesNewState = true;
    if (usePerformanceMeasurementMode) {
        sgl::FileUtils::get()->ensureDirectoryExists("images");
        if (testSortingPerformance) {
            performanceMeasurer = new AutomaticPerformanceMeasurer(
                    getTestModesSorting(), "performance.csv", "depth_complexity.csv",
                    [this](const InternalState &newState) { this->setNewState(newState); });
        } else {
            performanceMeasurer = new AutomaticPerformanceMeasurer(
                    getTestModesPaper(), "performance.csv", "depth_complexity.csv",
                    [this](const InternalState &newState) { this->setNewState(newState); });
        }
        performanceMeasurer->setInitialFreeMemKilobytes(gpuInitialFreeMemKilobytes);
    }
}

MainApp::~MainApp() {
    if (usePerformanceMeasurementMode) {
        delete performanceMeasurer;
        performanceMeasurer = nullptr;
    }

    for (auto& it : meshLoaderMap) {
        delete it.second;
    }
    meshLoaderMap.clear();
    for (HexahedralMeshFilter* meshFilter : meshFilters) {
        delete meshFilter;
    }
    meshFilters.clear();
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        delete meshRenderer;
    }
    meshRenderers.clear();

    delete rayMeshIntersection;

#ifdef USE_STEAMWORKS
    steamworks.shutdown();
#endif
}

void MainApp::setNewState(const InternalState &newState) {
    if (performanceMeasurer) {
        performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(0);
    }

    // 1. Change the window resolution?
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int currentWindowWidth = window->getWidth();
    int currentWindowHeight = window->getHeight();
    glm::ivec2 newResolution = newState.windowResolution;
    if (newResolution.x > 0 && newResolution.x > 0 && currentWindowWidth != newResolution.x
            && currentWindowHeight != newResolution.y) {
        window->setWindowSize(newResolution.x, newResolution.y);
    }

    // 1.1. Handle the new tiling mode for SSBO accesses (TODO).
    //setNewTilingMode(newState.tilingWidth, newState.tilingHeight, newState.useMortonCodeForTiling);

    // 1.2. Load the new transfer function if necessary.
    if (!newState.transferFunctionName.empty() && newState.transferFunctionName != lastState.transferFunctionName) {
        transferFunctionWindow.loadFunctionFromFile("Data/TransferFunctions/" + newState.transferFunctionName);
        colorLegendWidget.setTransferFunctionColorMap(
                transferFunctionWindow.getTransferFunctionMap_sRGB());
    }

    // 2.1. Do we need to load new renderers?
    if (firstState || newState.renderingMode != lastState.renderingMode
            || newState.rendererSettings != lastState.rendererSettings) {
        for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
            delete meshRenderer;
        }
        meshRenderers.clear();

        renderingMode = newState.renderingMode;
        setRenderers();
    }

    // 2.2. Pass state change to renderers to handle internally necessary state changes.
    for (HexahedralMeshRenderer* renderer : meshRenderers) {
        renderer->setNewState(newState);
    }
    for (size_t i = 0; i < newState.rendererSettings.size(); i++) {
        meshRenderers.at(i)->setNewSettings(newState.rendererSettings.at(i));
    }

    // 3. Pass state change to filters to handle internally necessary state changes.
    for (HexahedralMeshFilter* filter : meshFilters) {
        filter->setNewState(newState);
    }
    for (size_t i = 0; i < newState.filterSettings.size(); i++) {
        meshFilters.at(i)->setNewSettings(newState.filterSettings.at(i));
    }

    // 4. Load the correct mesh file.
    if (newState.meshDescriptor != lastState.meshDescriptor) {
        deformationFactor = newState.meshDescriptor.deformation;
        loadHexahedralMesh(newState.meshDescriptor.getFilename());
    }

    recordingTime = 0.0f;
    recordingTimeLast = 0.0f;
    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    lastState = newState;
    firstState = false;
    usesNewState = true;
}

void MainApp::setRenderers() {
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        delete meshRenderer;
    }
    meshRenderers.clear();

    if (renderingMode == RENDERING_MODE_SURFACE) {
        meshRenderers.push_back(new SurfaceRenderer(sceneData, transferFunctionWindow));
        meshRenderers.push_back(new WireframeRenderer_Faces(
                sceneData, transferFunctionWindow, false, true));
    } else if (renderingMode == RENDERING_MODE_WIREFRAME) {
        meshRenderers.push_back(new WireframeRenderer_Faces(
                sceneData, transferFunctionWindow, true, false));
    } else if (renderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        meshRenderers.push_back(new DepthComplexityRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_CLEAR_VIEW_FACES_UNIFIED) {
        meshRenderers.push_back(new ClearViewRenderer_FacesUnified(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_PSEUDO_VOLUME) {
        meshRenderers.push_back(new VolumeRenderer_FacesSlim(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_VOLUME) {
        meshRenderers.push_back(new VolumeRenderer_Volume(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_CLEAR_VIEW) {
        meshRenderers.push_back(new ClearViewRenderer_Volume(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_VOLUME_FACES) {
        meshRenderers.push_back(new VolumeRenderer_Faces(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_CLEAR_VIEW_FACES) {
        meshRenderers.push_back(new ClearViewRenderer_Faces(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_SINGULARITY) {
        meshRenderers.push_back(new SingularityRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_BASE_COMPLEX_LINES) {
        meshRenderers.push_back(new BaseComplexLineRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_BASE_COMPLEX_SURFACE) {
        meshRenderers.push_back(new BaseComplexSurfaceRenderer(sceneData, transferFunctionWindow));
        meshRenderers.push_back(new WireframeRenderer_Faces(
                sceneData, transferFunctionWindow, false, true));
    } else if (renderingMode == RENDERING_MODE_PARTITION_LINES) {
        meshRenderers.push_back(new PartitionLineRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_LOD_LINES) {
        meshRenderers.push_back(new LodLineRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_LOD_LINES_PER_FRAGMENT) {
        meshRenderers.push_back(new LodLineRendererPerFragment(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_LOD_LINES_PREVIEW) {
        meshRenderers.push_back(new LodLinePreviewRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_LOD_LINES_PREVIEW_SHEETS) {
        meshRenderers.push_back(new LodLinePreviewRenderer_SheetsFaces(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_SINGULARITY_TYPE_COUNTER) {
        meshRenderers.push_back(new LodLinePreviewRenderer_SheetsFaces(sceneData, transferFunctionWindow));
        meshRenderers.push_back(new SingularityTypeCounterRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_LINE_DENSITY_CONTROL) {
        meshRenderers.push_back(new LineDensityControlRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_HEX_SHEETS) {
        meshRenderers.push_back(new HexSheetRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_CLEAR_VIEW_VOLUME2) {
        meshRenderers.push_back(new ClearViewRenderer_Volume2(sceneData, transferFunctionWindow));
    }
}

void MainApp::resolutionChanged(sgl::EventPtr event) {
    SciVisApp::resolutionChanged(event);
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        meshRenderer->onResolutionChanged();
    }
}

void MainApp::updateColorSpaceMode() {
    SciVisApp::updateColorSpaceMode();
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
}

void MainApp::render() {
    SciVisApp::preRender();
    prepareVisualizationPipeline();

    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        reRender = reRender || meshRenderer->needsReRender();
    }

    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    glViewport(0, 0, width, height);

    // Set appropriate background alpha value.
    if (screenshot && screenshotTransparentBackground) {
        reRender = true;
        clearColor.setA(0);
        glDisable(GL_BLEND);
    }

    if (reRender || continuousRendering) {
        if (renderingMode != RENDERING_MODE_CLEAR_VIEW_FACES_UNIFIED && usePerformanceMeasurementMode) {
            performanceMeasurer->startMeasure(recordingTimeLast);
        }

        SciVisApp::prepareReRender();

        if (inputData.get() != nullptr) {
            for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                meshRenderer->render();
            }
        }

        if (renderingMode != RENDERING_MODE_CLEAR_VIEW_FACES_UNIFIED && usePerformanceMeasurementMode) {
            performanceMeasurer->endMeasure();
        }

        reRender = false;
    }

    SciVisApp::postRender();
}

void MainApp::renderGui() {
    sgl::ImGuiWrapper::get()->renderStart();

    if (showSettingsWindow) {
        if (ImGui::Begin("Settings", &showSettingsWindow)) {
            SciVisApp::renderGuiFpsCounter();

            // Selection of displayed model
            renderFileSelectionSettingsGui();

            ImGui::Separator();

            if (ImGui::CollapsingHeader("Scene Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen)) {
                renderSceneSettingsGui();
            }
        }
        ImGui::End();
    }

    if (transferFunctionWindow.renderGui()) {
        reRender = true;
        if (transferFunctionWindow.getTransferFunctionMapRebuilt()) {
            colorLegendWidget.setTransferFunctionColorMap(
                    transferFunctionWindow.getTransferFunctionMap_sRGB());
            if (inputData) {
                inputData->onTransferFunctionMapRebuilt();
            }
            for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                meshRenderer->onTransferFunctionMapRebuilt();
            }
        }
    }

    if (shallRenderColorLegendWidgets && inputData) {
        colorLegendWidget.setAttributeMinValue(transferFunctionWindow.getSelectedRangeMin());
        colorLegendWidget.setAttributeMaxValue(transferFunctionWindow.getSelectedRangeMax());
        colorLegendWidget.renderGui();
    }

    if (checkpointWindow.renderGui()) {
        fovDegree = camera->getFOVy() / sgl::PI * 180.0f;
        reRender = true;
    }

    for (HexahedralMeshFilter* meshFilter : meshFilters) {
        meshFilter->renderGui();
    }
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        meshRenderer->renderGui();
    }

    sgl::ImGuiWrapper::get()->renderEnd();
}

void MainApp::loadAvailableDataSetSources() {
    meshDataSetSources.clear();
    meshDataSetSources.push_back("Local file...");
    selectedFileSourceIndex = 0;

    if (!hexaLabDataSetsDownloaded) {
        return;
    }
    meshSourceDescriptions = parseSourceDescriptions();

    selectedFileSourceIndex = 1;
    for (MeshSourceDescription& sourceDescription  : meshSourceDescriptions) {
        meshDataSetSources.push_back(sourceDescription.label);
    }
    loadSelectedMeshDataSetNames();
}

void MainApp::loadSelectedMeshDataSetNames() {
    selectedMeshIndex = 0;
    selectedMeshDataSetNames.clear();
    selectedMeshDataSetNames.push_back("None");
    if (selectedFileSourceIndex != 0) {
        MeshSourceDescription& sourceDescription = meshSourceDescriptions.at(selectedFileSourceIndex - 1);
        for (std::string& dataSetName : sourceDescription.data) {
            selectedMeshDataSetNames.push_back(dataSetName);
        }
    }
}

std::string MainApp::getSelectedMeshFilename() {
    if (selectedFileSourceIndex == 0) {
        return customMeshFileName;
    }
    if (selectedMeshIndex == 0) {
        return ""; // None
    }

    MeshSourceDescription& sourceDescription = meshSourceDescriptions.at(selectedFileSourceIndex - 1);
    return meshDirectory + sourceDescription.path + "/" + sourceDescription.data.at(selectedMeshIndex - 1);
}

bool MainApp::getFileSourceContainsDeformationMeshes() {
    return selectedFileSourceIndex == 2
            && boost::ends_with(meshDataSetSources[selectedFileSourceIndex], "Deformation");
}

void MainApp::renderFileSelectionSettingsGui() {
    if (ImGui::Combo(
            "Source", &selectedFileSourceIndex, meshDataSetSources.data(),
            meshDataSetSources.size())) {
        loadSelectedMeshDataSetNames();
    }

    if (selectedFileSourceIndex == 0) {
        ImGui::InputText("##meshfilenamelabel", &customMeshFileName);
        ImGui::SameLine();
        if (ImGui::Button("Load File")) {
            loadHexahedralMesh(getSelectedMeshFilename());
        }
    } else {
        if (ImGui::Combo(
                "Mesh", &selectedMeshIndex, selectedMeshDataSetNames.data(),
                selectedMeshDataSetNames.size())) {
            loadHexahedralMesh(getSelectedMeshFilename());
        }
    }

    // Assume deformed meshes only in source at index 2 for now (don't clutter the UI for other sources).
    if (getFileSourceContainsDeformationMeshes()) {
        if (ImGui::SliderFloat("deformationFactor", &deformationFactor, 0.0f, 1.0f)) {
            if (selectedFileSourceIndex == currentlyLoadedFileSourceIndex
                    && selectedMeshIndex == currentlySelectedMeshIndex) {
                // Reload
                onDeformationFactorChanged();
            }
        }
    }

    if (!hexaLabDataSetsDownloaded || selectedFileSourceIndex > 0) {
        ImGui::Text(
                "By clicking the button below you confirm that you have\nthe right to download the data sets from "
                "hexalab.com.");
        if (ImGui::Button("Download HexaLab Datasets")) {
            downloadHexaLabDataSets([this]() {
                this->hexaLabDataSetsDownloaded = true;
                this->loadAvailableDataSetSources();
            }, loaderThread);
        }
    }
}

void MainApp::renderSceneSettingsGui() {
    if (ImGui::ColorEdit3("Clear Color", (float*)&clearColorSelection, 0)) {
        clearColor = sgl::colorFromFloat(
                clearColorSelection.x, clearColorSelection.y, clearColorSelection.z, clearColorSelection.w);
        transferFunctionWindow.setClearColor(clearColor);
        colorLegendWidget.setClearColor(clearColor);
        reRender = true;
    }

    SciVisApp::renderSceneSettingsGuiPre();
    ImGui::Checkbox("Show Transfer Function Window", &transferFunctionWindow.getShowTransferFunctionWindow());
    ImGui::Checkbox("Render Color Legend", &shallRenderColorLegendWidgets);

    if (ImGui::Combo(
            "Rendering Mode", (int*)&renderingMode, RENDERING_MODE_NAMES,
            IM_ARRAYSIZE(RENDERING_MODE_NAMES))) {
        setRenderers();
        reRender = true;
    }

    // Switch importance criterion
    if (ImGui::Combo(
            "Quality Measure", (int*)&selectedQualityMeasure, QUALITY_MEASURE_NAMES,
            IM_ARRAYSIZE(QUALITY_MEASURE_NAMES))) {
        changeQualityMeasureType();
        reRender = true;
    }

    SciVisApp::renderSceneSettingsGuiPost();
}

void MainApp::update(float dt) {
    sgl::SciVisApp::update(dt);

    if (usePerformanceMeasurementMode && !performanceMeasurer->update(recordingTime)) {
        // All modes were tested -> quit.
        quit();
    }

    updateCameraFlight(inputData.get() != nullptr, usesNewState);

    transferFunctionWindow.update(dt);

    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureKeyboard && !recording) {
        // Ignore inputs below
        return;
    }

    moveCameraKeyboard(dt);
    if (sgl::Keyboard->isKeyDown(SDLK_u)) {
        transferFunctionWindow.setShow(showSettingsWindow);
    }

    if (io.WantCaptureMouse) {
        // Ignore inputs below
        return;
    }

    moveCameraMouse(dt);

    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        meshRenderer->update(dt);
    }
}



// --- Visualization pipeline ---

sgl::AABB3 MainApp::computeAABB3(const std::vector<glm::vec3>& vertices) {
    sgl::AABB3 aabb;
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
    #pragma omp parallel for reduction(min: minX) reduction(min: minY) reduction(min: minZ) reduction(max: maxX) reduction(max: maxY) reduction(max: maxZ)
    for (size_t i = 0; i < vertices.size(); i++) {
        const glm::vec3& pt = vertices.at(i);
        minX = std::min(minX, pt.x);
        minY = std::min(minY, pt.y);
        minZ = std::min(minZ, pt.z);
        maxX = std::max(maxX, pt.x);
        maxY = std::max(maxY, pt.y);
        maxZ = std::max(maxZ, pt.z);
    }
    aabb.min = glm::vec3(minX, minY, minZ);
    aabb.max = glm::vec3(maxX, maxY, maxZ);
    return aabb;
}

void MainApp::normalizeVertexPositions(std::vector<glm::vec3>& vertices) {
    sgl::AABB3 aabb = computeAABB3(vertices);
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    #pragma omp parallel for
    for (size_t i = 0; i < vertices.size(); i++) {
        vertices.at(i) = (vertices.at(i) + translation) * scale;
    }

    if (rotateModelBy90DegreeTurns != 0) {
        glm::mat4 rotationMatrix = glm::rotate(rotateModelBy90DegreeTurns * sgl::HALF_PI, modelRotationAxis);

        #pragma omp parallel for
        for (size_t i = 0; i < vertices.size(); i++) {
            glm::vec4 rotatedVertex = rotationMatrix * glm::vec4(
                    vertices.at(i).x, vertices.at(i).y, vertices.at(i).z, 1.0f);
            vertices.at(i) = rotatedVertex;
        }
    }
}

void MainApp::applyVertexDeformations(
        std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& deformations, const float deformationFactor) {
    float maxDeformation = -FLT_MAX;
    #pragma omp parallel for reduction(max: maxDeformation)
    for (size_t i = 0; i < deformations.size(); i++) {
        maxDeformation = std::max(maxDeformation, glm::length(deformations.at(i)));
    }

    sgl::AABB3 aabb = computeAABB3(vertices);
    const float deformationScalingFactor = glm::length(aabb.getDimensions()) * deformationFactor / maxDeformation;
    #pragma omp parallel for
    for (size_t i = 0; i < vertices.size(); i++) {
        vertices.at(i) = vertices.at(i) + deformationScalingFactor * deformations.at(i);
    }
}

void MainApp::loadHexahedralMesh(const std::string &fileName) {
    if (fileName.size() == 0) {
        inputData = HexMeshPtr();
        return;
    }
    currentlyLoadedFileSourceIndex = selectedFileSourceIndex;
    currentlySelectedMeshIndex = selectedMeshIndex;

    size_t extensionPos = fileName.find_last_of('.');
    if (extensionPos == std::string::npos) {
        sgl::Logfile::get()->writeError("Error: Mesh file name has no extension.");
        return;
    }
    std::string extension = fileName.substr(extensionPos + 1);
    auto it = meshLoaderMap.find(extension);
    if (it == meshLoaderMap.end()) {
        sgl::Logfile::get()->writeError("Error: Unknown extension: ." + extension);
        return;
    }

    hexMeshVertices.clear();
    hexMeshCellIndices.clear();
    hexMeshDeformations.clear();
    hexMeshAnistropyMetricList.clear();
    bool loadingSuccessful = it->second->loadHexahedralMeshFromFile(
            fileName, hexMeshVertices, hexMeshCellIndices, hexMeshDeformations, hexMeshAnistropyMetricList);
    if (loadingSuccessful) {
        newMeshLoaded = true;
        checkpointWindow.onLoadDataSet(fileName);

        // A copy of the mesh data is stored for allowing the user to alter the deformation factor also after loading.
        std::vector<glm::vec3> vertices;
        vertices = hexMeshVertices;

        // Assume deformed meshes only in source at index 2 for now (don't clutter the UI for other sources).
        if (deformationFactor != 0.0f && (getFileSourceContainsDeformationMeshes() || usePerformanceMeasurementMode)
                && hexMeshDeformations.size() == vertices.size()) {
            applyVertexDeformations(vertices, hexMeshDeformations, deformationFactor);
        }

        normalizeVertexPositions(vertices);
        modelBoundingBox = computeAABB3(vertices);

        inputData = HexMeshPtr(new HexMesh(transferFunctionWindow, *rayMeshIntersection));
        inputData->setHexMeshData(vertices, hexMeshCellIndices, false);
        if (hexMeshAnistropyMetricList.empty()) {
            inputData->setQualityMeasure(selectedQualityMeasure);
        } else {
            inputData->setManualVertexAttributes(hexMeshAnistropyMetricList);
        }

        for (HexahedralMeshFilter* meshFilter : meshFilters) {
            meshFilter->onMeshLoaded(inputData);
        }

        if (true) { // useCameraFlight
            std::string cameraPathFilename =
                    saveDirectoryCameraPaths + sgl::FileUtils::get()->getPathAsList(fileName).back() + ".binpath";
            if (sgl::FileUtils::get()->exists(cameraPathFilename)) {
                cameraPath.fromBinaryFile(cameraPathFilename);
            } else {
                cameraPath.fromCirclePath(modelBoundingBox, fileName, usePerformanceMeasurementMode
                        ? CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT : CAMERA_PATH_TIME_RECORDING,
                        usePerformanceMeasurementMode);
                //cameraPath.saveToBinaryFile(cameraPathFilename);
            }
        }
    }
}

void MainApp::reloadDataSet() {
    loadHexahedralMesh(getSelectedMeshFilename());
}

void MainApp::onDeformationFactorChanged() {
    //loadHexahedralMesh(getSelectedMeshFilename());
    std::vector<glm::vec3> vertices = hexMeshVertices;
    if (deformationFactor != 0.0f && hexMeshDeformations.size() == vertices.size()) {
        applyVertexDeformations(vertices, hexMeshDeformations, deformationFactor);
    }
    normalizeVertexPositions(vertices);
    modelBoundingBox = computeAABB3(vertices);
    inputData->updateVertexPositions(vertices);
}

void MainApp::prepareVisualizationPipeline() {
    if (inputData != nullptr) {
        bool isPreviousNodeDirty = inputData->isDirty();
        HexMeshPtr filteredMesh;
        if (inputData->isBaseComplexMeshLoaded()) {
            filteredMesh = getFilteredMesh(isPreviousNodeDirty);
        } else {
            filteredMesh = inputData;
        }
        // Generate the visualization mapping for all renderers that have the dirty flag set (or if the filtered data
        // changed).
        for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
            if (meshRenderer->isDirty() || isPreviousNodeDirty) {
                meshRenderer->uploadVisualizationMapping(filteredMesh, newMeshLoaded);
            }
        }
    }
    newMeshLoaded = false;
}

HexMeshPtr MainApp::getFilteredMesh(bool& isDirty) {
    HexMeshPtr filteredMesh = inputData;

    // Test if we need to re-run the filters.
    for (HexahedralMeshFilter* meshFilter : meshFilters) {
        if (!meshFilter->isEnabled()) {
            continue;
        }
        if (meshFilter->isDirty()) {
            isDirty = true;
            reRender = true;
        }
    }

    if (isDirty) {
        filteredMesh->unmark();
        // Pass the output of each filter to the next filter.
        for (HexahedralMeshFilter* meshFilter : meshFilters) {
            if (!meshFilter->isEnabled()) {
                continue;
            }
            meshFilter->filterMesh(filteredMesh);
            filteredMesh = meshFilter->getOutput();
        }
    }

    return filteredMesh;
}

void MainApp::changeQualityMeasureType() {
    if (inputData) {
        inputData->setQualityMeasure(selectedQualityMeasure);
    }
    colorLegendWidget.setAttributeDisplayName(QUALITY_MEASURE_NAMES[int(selectedQualityMeasure)]);
}
