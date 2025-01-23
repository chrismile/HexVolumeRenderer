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

#include <GL/glew.h>

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
#include <Graphics/OpenGL/Texture.hpp>

#include "Widgets/DataView.hpp"

#include "Mesh/HexMesh/Loaders/VtkLoader.hpp"
#include "Mesh/HexMesh/Loaders/MeshLoader.hpp"
#include "Mesh/HexMesh/Loaders/DatLoader.hpp"
#include "Mesh/HexMesh/Loaders/HexaLabDatasets.hpp"
#include "Mesh/HexMesh/Loaders/DegStressLoader.hpp"
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
                  &sceneFramebuffer, &sceneTexture, &sceneDepthRBO, camera, clearColor, performanceMeasurer,
                  recording, useCameraFlight, *rayMeshIntersection)
#ifdef USE_PYTHON
        , replayWidget(sceneData, transferFunctionWindow, checkpointWindow)
#endif
{
#ifdef USE_STEAMWORKS
    steamworks.initialize();
#endif

#ifdef USE_PYTHON
    replayWidget.setLoadMeshCallback([this](const MeshDescriptor& meshDescriptor) {
        std::string meshFilenameNew = meshDescriptor.getFilename();
        if (loadedMeshFilename != meshFilenameNew) {
            loadHexahedralMesh(meshFilenameNew);
        }
    });
    replayWidget.setLoadRendererCallback([this](const std::string& rendererName) {
        RenderingMode renderingModeNew = renderingMode;
        int i;
        for (i = 0; i < IM_ARRAYSIZE(RENDERING_MODE_NAMES); i++) {
            if (RENDERING_MODE_NAMES[i] == rendererName) {
                renderingModeNew = RenderingMode(i);
                break;
            }
        }
        if (i == IM_ARRAYSIZE(RENDERING_MODE_NAMES)) {
            sgl::Logfile::get()->writeError(
                    std::string() + "ERROR in replay widget load renderer callback: Unknown renderer name \""
                    + rendererName + "\".");
        }
        if (renderingModeNew != renderingMode) {
            renderingMode = renderingModeNew;
            setRenderers();
        }
    });
#endif

    cameraPath.setApplicationCallback([this](
            const std::string& modelFilename, glm::vec3& centerOffset, float& startAngle, float& pulseFactor,
            float& standardZoom) {
        if (sgl::startsWith(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory() +
                "Meshes/2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation/anc101_a1.mesh")) {
            centerOffset = glm::vec3(0.0f, -0.02f, 0.0f);
            pulseFactor = 3.0f;
            standardZoom = 1.6f;
        }
        if (sgl::startsWith(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory() +
                "Meshes/2014 - l1-Based Construction of Polycube Maps from Complex Shapes/cognit/hex.vtk")) {
            centerOffset = glm::vec3(0.0f, -0.02f, 0.0f);
            pulseFactor = 3.0f;
            standardZoom = 1.6f;
        }
        if (usePerformanceMeasurementMode && sgl::endsWith(modelFilename, "cubic128.vtk")) {
            standardZoom = 1.3f;
        }
    });

    sgl::ColorLegendWidget::setFontScaleStandard(1.0f);

    useDockSpaceMode = true;
    sgl::AppSettings::get()->getSettings().getValueOpt("useDockSpaceMode", useDockSpaceMode);
    sgl::AppSettings::get()->getSettings().getValueOpt("useFixedSizeViewport", useFixedSizeViewport);
    sgl::AppSettings::get()->getSettings().getValueOpt("fixedViewportSizeX", fixedViewportSize.x);
    sgl::AppSettings::get()->getSettings().getValueOpt("fixedViewportSizeY", fixedViewportSize.y);
    fixedViewportSizeEdit = fixedViewportSize;
    showPropertyEditor = true;
    sgl::ImGuiWrapper::get()->setUseDockSpaceMode(useDockSpaceMode);

    dataView = std::make_shared<DataView>(camera, screenshotTransparentBackground, useLinearRGB, gammaCorrectionShader);
    if (useDockSpaceMode) {
        sceneData.framebuffer = &dataView->sceneFramebuffer;
        sceneData.sceneTexture = &dataView->sceneTexture;
        sceneData.sceneDepthRBO = &dataView->sceneDepthRBO;
    } else {
        sceneData.framebuffer = &sceneFramebuffer;
        sceneData.sceneTexture = &sceneTexture;
        sceneData.sceneDepthRBO = &sceneDepthRBO;
    }

#ifdef NDEBUG
    showFpsOverlay = false;
#else
    showFpsOverlay = true;
#endif
    sgl::AppSettings::get()->getSettings().getValueOpt("showFpsOverlay", showFpsOverlay);
    sgl::AppSettings::get()->getSettings().getValueOpt("showCoordinateAxesOverlay", showCoordinateAxesOverlay);

    clearColor = sgl::Color(0, 0, 0, 255);
    clearColorSelection = ImColor(clearColor.getColorRGBA());
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
    colorLegendWidget.setClearColor(clearColor);
    coordinateAxesOverlayWidget.setClearColor(clearColor);
    dataView->setClearColor(clearColor);

    sgl::Renderer->setErrorCallback(&openglErrorCallback);
    sgl::Renderer->setDebugVerbosity(sgl::DEBUG_OUTPUT_CRITICAL_ONLY);
    resolutionChanged(sgl::EventPtr());

    rendererWindowNames = {
            SurfaceRenderer::getWindowName(),
            WireframeRenderer_Faces::getWindowName(),
            DepthComplexityRenderer::getWindowName(),
            ClearViewRenderer_FacesUnified::getWindowName(),
            VolumeRenderer_FacesSlim::getWindowName(),
            VolumeRenderer_Volume::getWindowName(),
            ClearViewRenderer_Volume::getWindowName(),
            VolumeRenderer_Faces::getWindowName(),
            ClearViewRenderer_Faces::getWindowName(),
            SingularityRenderer::getWindowName(),
            BaseComplexLineRenderer::getWindowName(),
            BaseComplexSurfaceRenderer::getWindowName(),
            PartitionLineRenderer::getWindowName(),
            LodLineRenderer::getWindowName(),
            LodLineRendererPerFragment::getWindowName(),
            LodLinePreviewRenderer::getWindowName(),
            LodLinePreviewRenderer_SheetsFaces::getWindowName(),
            SingularityTypeCounterRenderer::getWindowName(),
            LineDensityControlRenderer::getWindowName(),
            HexSheetRenderer::getWindowName(),
            ClearViewRenderer_Volume2::getWindowName(),
    };

    selectedQualityMeasure = QUALITY_MEASURE_SCALED_JACOBIAN;
    changeQualityMeasureType();

    meshLoaderMap.insert(std::make_pair("vtk", new VtkLoader));
    meshLoaderMap.insert(std::make_pair("mesh", new MeshLoader));
    meshLoaderMap.insert(std::make_pair("dat", new DatCartesianGridLoader));
    meshLoaderMap.insert(std::make_pair("degStress", new DegStressLoader));
    meshFilters.push_back(new PlaneFilter);
    meshFilters.push_back(new PeelingFilter);
    meshFilters.push_back(new QualityFilter);

    if (usePerformanceMeasurementMode) {
        useCameraFlight = true;
    }
    if (useCameraFlight && recording) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        if (useRecordingResolution) {
            window->setWindowSize(recordingResolution.x, recordingResolution.y);
        }
        realTimeCameraFlight = false;
        transferFunctionWindow.loadFunctionFromFile(
                sgl::AppSettings::get()->getDataDirectory() + "TransferFunctions/Standard_PerVertex.xml");
        loadHexahedralMesh(
                sgl::AppSettings::get()->getDataDirectory()
                + "Meshes/2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation/anc101_a1.mesh");
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
    const std::string meshDirectory = sgl::AppSettings::get()->getDataDirectory() + "Meshes/";
    hexaLabDataSetsDownloaded = sgl::FileUtils::get()->exists(meshDirectory + "index.json");
    loadAvailableDataSetSources();

    if (!sgl::AppSettings::get()->getSettings().hasKey("cameraNavigationMode")) {
        cameraNavigationMode = sgl::CameraNavigationMode::TURNTABLE;
        updateCameraNavigationMode();
    }

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

    sgl::AppSettings::get()->getSettings().addKeyValue("useDockSpaceMode", useDockSpaceMode);
    if (!usePerformanceMeasurementMode) {
        sgl::AppSettings::get()->getSettings().addKeyValue("useFixedSizeViewport", useFixedSizeViewport);
        sgl::AppSettings::get()->getSettings().addKeyValue("fixedViewportSizeX", fixedViewportSize.x);
        sgl::AppSettings::get()->getSettings().addKeyValue("fixedViewportSizeY", fixedViewportSize.y);
    }
    sgl::AppSettings::get()->getSettings().addKeyValue("showFpsOverlay", showFpsOverlay);
    sgl::AppSettings::get()->getSettings().addKeyValue("showCoordinateAxesOverlay", showCoordinateAxesOverlay);
}

void MainApp::loadReplicabilityStampState() {
#ifdef USE_PYTHON
    replayWidget.loadReplicabilityStampState();
    recordingTime = 0.0f;
    realTimeReplayUpdates = false;
#else
    throw std::runtime_error("Error in MainApp::loadReplicabilityStampState: Program built without Python support!");
#endif
}

void MainApp::setNewState(const InternalState &newState) {
    if (performanceMeasurer) {
        performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(0);
    }

    // 1. Change the window resolution?
    glm::ivec2 newResolution = newState.windowResolution;
    if (useDockSpaceMode) {
        useFixedSizeViewport = true;
        fixedViewportSizeEdit = newResolution;
        fixedViewportSize = newResolution;
    } else {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        int currentWindowWidth = window->getWidth();
        int currentWindowHeight = window->getHeight();
        if (newResolution.x > 0 && newResolution.y > 0 && currentWindowWidth != newResolution.x
            && currentWindowHeight != newResolution.y) {
            window->setWindowSize(newResolution.x, newResolution.y);
        }
    }

    // 1.1. Handle the new tiling mode for SSBO accesses (TODO).
    //setNewTilingMode(newState.tilingWidth, newState.tilingHeight, newState.useMortonCodeForTiling);

    // 1.2. Load the new transfer function if necessary.
    if (!newState.transferFunctionName.empty() && newState.transferFunctionName != lastState.transferFunctionName) {
        transferFunctionWindow.loadFunctionFromFile(
                sgl::AppSettings::get()->getDataDirectory()
                + "TransferFunctions/" + newState.transferFunctionName);
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

    if (!useDockSpaceMode) {
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
    }

    SciVisApp::postRender();
}

void MainApp::renderGui() {
    focusedWindowIndex = -1;
    mouseHoverWindowIndex = -1;
    sceneData.pickingOffsetX = 0;
    sceneData.pickingOffsetY = 0;

    if (useDockSpaceMode) {
        ImGuiID dockSpaceId = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        ImGuiDockNode* centralNode = ImGui::DockBuilderGetNode(dockSpaceId);
        static bool isProgramStartup = true;
        if (isProgramStartup && centralNode->IsEmpty()) {
            const float tabWidth = 0.3f;
            ImGuiID dockLeftId, dockRightId, dockMainRightId, dockMainId;
            ImGui::DockBuilderSplitNode(
                    dockSpaceId, ImGuiDir_Left, tabWidth,
                    &dockLeftId, &dockMainRightId);
            ImGui::DockBuilderSplitNode(
                    dockMainRightId, ImGuiDir_Right, tabWidth / (1.0f - tabWidth),
                    &dockRightId, &dockMainId);
            ImGui::DockBuilderDockWindow("Renderer Window", dockMainId);

            ImGuiID dockLeftUpId, dockLeftDownId;
            ImGui::DockBuilderSplitNode(
                    dockLeftId, ImGuiDir_Up, 0.6f,
                    &dockLeftUpId, &dockLeftDownId);
            ImGui::DockBuilderDockWindow("Settings", dockLeftUpId);
            ImGui::DockBuilderDockWindow("Transfer Function", dockLeftDownId);

            ImGuiID dockRightUpId, dockRightDown1Id;
            ImGui::DockBuilderSplitNode(
                    dockRightId, ImGuiDir_Up, 0.6f,
                    &dockRightUpId, &dockRightDown1Id);
            for (const std::string& rendererName : rendererWindowNames) {
                ImGui::DockBuilderDockWindow(rendererName.c_str(), dockRightUpId);
            }

            const float sizeReplay = 0.4f;
            ImGuiID dockReplayCheckpointId, dockRightDown2Id;
            ImGui::DockBuilderSplitNode(
                    dockRightDown1Id, ImGuiDir_Up, sizeReplay,
                    &dockReplayCheckpointId, &dockRightDown2Id);
            ImGui::DockBuilderDockWindow("Replay Widget", dockReplayCheckpointId);
            ImGui::DockBuilderDockWindow("Camera Checkpoints", dockReplayCheckpointId);

            ImGuiID dockPlaneFilterId, dockRightDown3Id;
            ImGui::DockBuilderSplitNode(
                    dockRightDown2Id, ImGuiDir_Up, 1.0f/3.0f,
                    &dockPlaneFilterId, &dockRightDown3Id);
            ImGuiID dockPeelingFilterId, dockQualityFilterId;
            ImGui::DockBuilderSplitNode(
                    dockRightDown3Id, ImGuiDir_Up, 1.0f/2.0f,
                    &dockPeelingFilterId, &dockQualityFilterId);
            ImGui::DockBuilderDockWindow("Plane Filter", dockPlaneFilterId);
            ImGui::DockBuilderDockWindow("Peeling Filter", dockPeelingFilterId);
            ImGui::DockBuilderDockWindow("Quality Filter", dockQualityFilterId);

            ImGui::DockBuilderFinish(dockSpaceId);
        }
        isProgramStartup = false;

        //renderGuiMenuBar();

        if (showRendererWindow) {
            bool isViewOpen = true;
            sgl::ImGuiWrapper::get()->setNextWindowStandardSize(800, 600);
            if (ImGui::Begin("Renderer Window", &isViewOpen)) {
                if (ImGui::IsWindowFocused()) {
                    focusedWindowIndex = 0;
                }
                sgl::ImGuiWrapper::get()->setWindowViewport(0, ImGui::GetWindowViewport());
                sgl::ImGuiWrapper::get()->setWindowPosAndSize(0, ImGui::GetWindowPos(), ImGui::GetWindowSize());

                ImVec2 pos = ImGui::GetCursorScreenPos();
                sceneData.pickingOffsetX = int(pos.x);
                sceneData.pickingOffsetY = int(pos.y);

                ImVec2 sizeContent = ImGui::GetContentRegionAvail();
                if (useFixedSizeViewport) {
                    sizeContent = ImVec2(float(fixedViewportSize.x), float(fixedViewportSize.y));
                }
                if (int(sizeContent.x) != int(dataView->viewportWidth)
                        || int(sizeContent.y) != int(dataView->viewportHeight)) {
                    dataView->resize(int(sizeContent.x), int(sizeContent.y));
                    if (dataView->viewportWidth > 0 && dataView->viewportHeight > 0) {
                        for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                            meshRenderer->onResolutionChanged();
                        }
                    }
                    reRender = true;
                }

                for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                    reRender = reRender || meshRenderer->needsReRender();
                }

                if (reRender || continuousRendering) {
                    if (dataView->viewportWidth > 0 && dataView->viewportHeight > 0) {
                        // Set appropriate background alpha value.
                        if (screenshot && screenshotTransparentBackground) {
                            reRender = true;
                            clearColor.setA(0);
                            glDisable(GL_BLEND);
                        }

                        dataView->beginRender();
                        if (inputData.get() != nullptr) {
                            for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                                meshRenderer->render();
                            }
                        }
                        dataView->endRender();
                    }
                    reRender = false;
                }

                if (dataView->viewportWidth > 0 && dataView->viewportHeight > 0) {
                    if (!uiOnScreenshot && screenshot) {
                        printNow = true;
                        sgl::Renderer->bindFBO(dataView->getSceneFramebuffer());
                        customScreenshotWidth = int(dataView->viewportWidth);
                        customScreenshotHeight = int(dataView->viewportHeight);
                        std::string screenshotFilename =
                                saveDirectoryScreenshots + saveFilenameScreenshots
                                + "_" + sgl::toString(screenshotNumber);
                        screenshotFilename += ".png";
                        saveScreenshot(screenshotFilename);
                        customScreenshotWidth = -1;
                        customScreenshotHeight = -1;
                        sgl::Renderer->unbindFBO();
                        printNow = false;
                        screenshot = true;
                    }

                    if (!uiOnScreenshot && recording && !isFirstRecordingFrame) {
                        videoWriter->pushFramebuffer(dataView->getSceneFramebuffer());
                    }

                    if (isViewOpen) {
                        ImGui::Image(
                                ImTextureID(static_cast<sgl::TextureGL*>(
                                        dataView->getSceneTextureResolved().get())->getTexture()),
                                sizeContent, ImVec2(0, 1), ImVec2(1, 0));
                        if (ImGui::IsItemHovered()) {
                            mouseHoverWindowIndex = 0;
                        }
                    }

                    if (useDockSpaceMode && shallRenderColorLegendWidgets && inputData) {
                        colorLegendWidget.setAttributeMinValue(transferFunctionWindow.getSelectedRangeMin());
                        colorLegendWidget.setAttributeMaxValue(transferFunctionWindow.getSelectedRangeMax());
                        colorLegendWidget.renderGui();
                    }

                    if (showFpsOverlay) {
                        renderGuiFpsOverlay();
                    }
                    if (showCoordinateAxesOverlay) {
                        renderGuiCoordinateAxesOverlay(dataView->camera);
                    }
                }
            }
            ImGui::End();
        }

        if (!uiOnScreenshot && screenshot) {
            screenshot = false;
            screenshotNumber++;
        }
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        int width = window->getWidth();
        int height = window->getHeight();
        glViewport(0, 0, width, height);
        reRender = false;
    }

    if (showSettingsWindow) {
        sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3085, 39, 744, 1348);
        if (ImGui::Begin("Settings", &showSettingsWindow)) {
            SciVisApp::renderGuiFpsCounter();

            // Selection of displayed model
            renderFileSelectionSettingsGui();

            ImGui::Separator();

            if (ImGui::CollapsingHeader("Scene Settings", nullptr, ImGuiTreeNodeFlags_DefaultOpen)) {
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

    if (!useDockSpaceMode && shallRenderColorLegendWidgets && inputData) {
        colorLegendWidget.setAttributeMinValue(transferFunctionWindow.getSelectedRangeMin());
        colorLegendWidget.setAttributeMaxValue(transferFunctionWindow.getSelectedRangeMax());
        colorLegendWidget.renderGui();
    }

    if (checkpointWindow.renderGui()) {
        fovDegree = camera->getFOVy() / sgl::PI * 180.0f;
        reRender = true;
    }

#ifdef USE_PYTHON
    ReplayWidget::ReplayWidgetUpdateType replayWidgetUpdateType = replayWidget.renderGui();
    if (replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_LOAD) {
        recordingTime = 0.0f;
        //realTimeReplayUpdates = true;
        realTimeReplayUpdates = false;
    }
    if (replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_START_RECORDING) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        if (useRecordingResolution && window->getWindowResolution() != recordingResolution) {
            window->setWindowSize(recordingResolution.x, recordingResolution.y);
        }

        if (videoWriter) {
            delete videoWriter;
            videoWriter = nullptr;
        }

        recordingTime = 0.0f;
        realTimeReplayUpdates = false;
        recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();

        recording = true;
        isFirstRecordingFrame = true;
        videoWriter = new sgl::VideoWriter(
                saveDirectoryVideos + saveFilenameVideos
                + "_" + sgl::toString(videoNumber++) + ".mp4", FRAME_RATE_VIDEOS);
    }
    if (replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING) {
        recording = false;
        if (videoWriter) {
            delete videoWriter;
            videoWriter = nullptr;
        }
    }
    if (replayWidget.getUseCameraFlight()
            && replayWidgetUpdateType != ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING) {
        useCameraFlight = true;
        startedCameraFlightPerUI = true;
        realTimeCameraFlight = false;
        cameraPath.resetTime();
    }
    if (replayWidget.getUseCameraFlight()
            && replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING) {
        useCameraFlight = false;
        cameraPath.resetTime();
    }
    if (replayWidgetUpdateType != ReplayWidget::REPLAY_WIDGET_UPDATE_NONE) {
        reRender = true;
    }
#endif

    for (HexahedralMeshFilter* meshFilter : meshFilters) {
        meshFilter->renderGui();
    }
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        meshRenderer->renderGui();
    }
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
    const std::string meshDirectory = sgl::AppSettings::get()->getDataDirectory() + "Meshes/";
    return meshDirectory + sourceDescription.path + "/" + sourceDescription.data.at(selectedMeshIndex - 1);
}

bool MainApp::getFileSourceContainsDeformationMeshes() {
    return sgl::startsWith(meshDataSetSources[selectedFileSourceIndex], "00")
            && meshDataSetSources[selectedFileSourceIndex].find("Deformation") != std::string::npos;
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
        if (getIsDataSetDownloadRunning()) {
            //float windowContentRegionWidth =
            //        ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x;
            //ImGui::SetCursorPosX(windowContentRegionWidth - ImGui::GetTextLineHeight());
            ImGui::SameLine();
            ImGui::ProgressSpinner(
                    "##progress-spinner", -1.0f, -1.0f, 4.0f,
                    ImVec4(0.1f, 0.5f, 1.0f, 1.0f));
        }
    }
}

void MainApp::renderSceneSettingsGui() {
    if (ImGui::ColorEdit3("Clear Color", (float*)&clearColorSelection, 0)) {
        clearColor = sgl::colorFromFloat(
                clearColorSelection.x, clearColorSelection.y, clearColorSelection.z, clearColorSelection.w);
        transferFunctionWindow.setClearColor(clearColor);
        colorLegendWidget.setClearColor(clearColor);
        coordinateAxesOverlayWidget.setClearColor(clearColor);
        dataView->setClearColor(clearColor);
        reRender = true;
    }

    SciVisApp::renderSceneSettingsGuiPre();
    ImGui::Checkbox("Show Transfer Function Window", &transferFunctionWindow.getShowWindow());
    ImGui::Checkbox("Render Color Legend", &shallRenderColorLegendWidgets);
    newDockSpaceMode = useDockSpaceMode;
    if (ImGui::Checkbox("Use Docking Mode", &newDockSpaceMode)) {
        scheduledDockSpaceModeChange = true;
    }

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

    if (scheduledDockSpaceModeChange) {
        if (useDockSpaceMode) {
            sceneData.framebuffer = &dataView->sceneFramebuffer;
            sceneData.sceneTexture = &dataView->sceneTexture;
            sceneData.sceneDepthRBO = &dataView->sceneDepthRBO;
        } else {
            sceneData.framebuffer = &sceneFramebuffer;
            sceneData.sceneTexture = &sceneTexture;
            sceneData.sceneDepthRBO = &sceneDepthRBO;
        }

        useDockSpaceMode = newDockSpaceMode;
        scheduledDockSpaceModeChange = false;
        resolutionChanged(sgl::EventPtr());
        reRender = true;
    }

    if (usePerformanceMeasurementMode && !performanceMeasurer->update(recordingTime)) {
        // All modes were tested -> quit.
        quit();
    }

    updateCameraFlight(inputData.get() != nullptr, usesNewState);

    transferFunctionWindow.update(dt);

#ifdef USE_PYTHON
    bool stopRecording = false;
    bool stopCameraFlight = false;
    if (replayWidget.update(recordingTime, stopRecording, stopCameraFlight)) {
        if (!useCameraFlight) {
            camera->overwriteViewMatrix(replayWidget.getViewMatrix());
            if (camera->getFOVy() != replayWidget.getCameraFovy()) {
                camera->setFOVy(replayWidget.getCameraFovy());
            }
        }
        SettingsMap currentRendererSettings = replayWidget.getCurrentRendererSettings();
        for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
            meshRenderer->setNewSettings(currentRendererSettings);
        }
        reRender = true;

        if (!useCameraFlight) {
            if (realTimeReplayUpdates) {
                uint64_t currentTimeStamp = sgl::Timer->getTicksMicroseconds();
                uint64_t timeElapsedMicroSec = currentTimeStamp - recordingTimeStampStart;
                recordingTime = timeElapsedMicroSec * 1e-6;
            } else {
                recordingTime += FRAME_TIME_CAMERA_PATH;
            }
        }
    }
    if (stopRecording) {
        recording = false;
        if (videoWriter) {
            delete videoWriter;
            videoWriter = nullptr;
        }
        if (useCameraFlight) {
            useCameraFlight = false;
            cameraPath.resetTime();
        }
    }
    if (stopCameraFlight) {
        useCameraFlight = false;
        cameraPath.resetTime();
    }
#endif

    ImGuiIO &io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard || recording || focusedWindowIndex != -1) {
        moveCameraKeyboard(dt);
    }

    if (!io.WantCaptureKeyboard || recording) {
#ifdef SGL_INPUT_API_V2
        if (sgl::Keyboard->isKeyDown(ImGuiKey_U)) {
            transferFunctionWindow.setShowWindow(showSettingsWindow);
        }
#else
        if (sgl::Keyboard->isKeyDown(SDLK_u)) {
            transferFunctionWindow.setShowWindow(showSettingsWindow);
        }
#endif
    }

    if (!io.WantCaptureMouse || mouseHoverWindowIndex != -1) {
        moveCameraMouse(dt);
    }

    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        meshRenderer->update(dt);
    }
}

bool MainApp::checkHasValidExtension(const std::string& filenameLower) {
    if (sgl::endsWith(filenameLower, ".vtk")
            || sgl::endsWith(filenameLower, ".mesh")
            || sgl::endsWith(filenameLower, ".dat")
            || sgl::endsWith(filenameLower, ".degStress")) {
        return true;
    }
    return false;
}

void MainApp::onFileDropped(const std::string& droppedFileName) {
    std::string filenameLower = sgl::toLowerCopy(droppedFileName);
    if (checkHasValidExtension(filenameLower)) {
        selectedFileSourceIndex = 0;
        customMeshFileName = droppedFileName;
        loadHexahedralMesh(getSelectedMeshFilename());
    } else {
        sgl::Logfile::get()->writeError(
                "The dropped file name has an unknown extension \""
                + sgl::FileUtils::get()->getFileExtension(filenameLower) + "\".");
    }
}



// --- Visualization pipeline ---

sgl::AABB3 MainApp::computeAABB3(const std::vector<glm::vec3>& vertices) {
    sgl::AABB3 aabb;
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(min: minX) reduction(min: minY) reduction(min: minZ) reduction(max: maxX) \
    reduction(max: maxY) reduction(max: maxZ) shared(vertices) default(none)
#endif
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

#if _OPENMP >= 200805
    #pragma omp parallel for shared(vertices, translation, scale) default(none)
#endif
    for (size_t i = 0; i < vertices.size(); i++) {
        vertices.at(i) = (vertices.at(i) + translation) * scale;
    }

    if (rotateModelBy90DegreeTurns != 0) {
        glm::mat4 rotationMatrix = glm::rotate(rotateModelBy90DegreeTurns * sgl::HALF_PI, modelRotationAxis);

#if _OPENMP >= 200805
        #pragma omp parallel for shared(vertices, rotationMatrix) default(none)
#endif
        for (size_t i = 0; i < vertices.size(); i++) {
            glm::vec4 rotatedVertex = rotationMatrix * glm::vec4(
                    vertices.at(i).x, vertices.at(i).y, vertices.at(i).z, 1.0f);
            vertices.at(i) = glm::vec3(rotatedVertex.x, rotatedVertex.y, rotatedVertex.z);
        }
    }
}

void MainApp::applyVertexDeformations(
        std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& deformations, const float deformationFactor) {
    float maxDeformation = -FLT_MAX;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(max: maxDeformation) shared(vertices, deformations) default(none)
#endif
    for (size_t i = 0; i < deformations.size(); i++) {
        maxDeformation = std::max(maxDeformation, glm::length(deformations.at(i)));
    }

    sgl::AABB3 aabb = computeAABB3(vertices);
    const float deformationScalingFactor = glm::length(aabb.getDimensions()) * deformationFactor / maxDeformation;
#if _OPENMP >= 200805
    #pragma omp parallel for shared(vertices, deformations, deformationScalingFactor) default(none)
#endif
    for (size_t i = 0; i < vertices.size(); i++) {
        vertices.at(i) = vertices.at(i) + deformationScalingFactor * deformations.at(i);
    }
}

void MainApp::loadHexahedralMesh(const std::string &fileName) {
    if (fileName.empty()) {
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

    auto loadingStartTime = std::chrono::system_clock::now();

    hexMeshVertices.clear();
    hexMeshCellIndices.clear();
    hexMeshDeformations.clear();
    hexMeshAttributeList.clear();
    bool isPerVertexData = true;
    bool loadingSuccessful = it->second->loadHexahedralMeshFromFile(
            fileName, hexMeshVertices, hexMeshCellIndices, hexMeshDeformations,
            hexMeshAttributeList, isPerVertexData);
    if (loadingSuccessful) {
        sgl::ColorLegendWidget::resetStandardSize();
        newMeshLoaded = true;
        printLoadingTime = true;
        loadingTimeSeconds = 0.0;
        checkpointWindow.onLoadDataSet(fileName);
        loadedMeshFilename = fileName;

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

        // Delete old data to get more free RAM.
        inputData = HexMeshPtr();
        for (HexahedralMeshFilter* meshFilter : meshFilters) {
            meshFilter->removeOldMesh();
        }
        for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
            meshRenderer->removeOldMesh();
        }

        inputData = HexMeshPtr(new HexMesh(transferFunctionWindow, *rayMeshIntersection));
        bool loadMeshRepresentation =
                renderingMode != RENDERING_MODE_PSEUDO_VOLUME && renderingMode != RENDERING_MODE_DEPTH_COMPLEXITY;
        inputData->setHexMeshData(vertices, hexMeshCellIndices, loadMeshRepresentation);
        inputData->setQualityMeasure(selectedQualityMeasure);
        MeshSourceDescription sourceDescription;
        std::vector<std::string> dataAdditionalFiles;
        if (selectedMeshIndex != 0) {
            sourceDescription = meshSourceDescriptions.at(selectedFileSourceIndex - 1);
            dataAdditionalFiles = sourceDescription.dataAdditionalFiles.at(selectedMeshIndex - 1);
        }
        if (!hexMeshAttributeList.empty()) {
            if (isPerVertexData) {
                if (extension == "degStress") {
                    inputData->addManualVertexAttribute(hexMeshAttributeList, "Degeneracy Metric");
                } else {
                    inputData->addManualVertexAttribute(hexMeshAttributeList, "Anisotropy");
                }
            } else {
                inputData->addManualCellAttribute(hexMeshAttributeList, "Convergence");
                //std::vector<float> unconvergence(hexMeshAttributeList.size());
                //for (size_t i = 0; i < hexMeshAttributeList.size(); i++) {
                //    unconvergence.at(i) = 4.0f * hexMeshAttributeList.at(i) * (1.0f - hexMeshAttributeList.at(i));
                //}
                //inputData->addManualCellAttribute(unconvergence, "Unconvergence");
            }
        }
        if (!dataAdditionalFiles.empty()) {
            for (std::string& additionalDataName : dataAdditionalFiles) {
                if (sgl::endsWith(additionalDataName, ".dat")) {
                    const std::string meshDirectory = sgl::AppSettings::get()->getDataDirectory() + "Meshes/";
                    std::string additionalDataFilename =
                            meshDirectory + sourceDescription.path + "/" + additionalDataName;
                    std::vector<std::vector<float>> datData = loadDatData(additionalDataFilename);

                    std::string dataType = "Unknown";
                    std::string filenameLowerCase = sgl::toLowerCopy(additionalDataName);
                    if (filenameLowerCase.find("stress") != std::string::npos) {
                        dataType = "";
                        if (filenameLowerCase.find("cartesian") != std::string::npos) {
                            dataType = "Cartesian ";
                        } else if (filenameLowerCase.find("principal") != std::string::npos) {
                            dataType = "Principal ";
                        } else if (filenameLowerCase.find("mises") != std::string::npos) {
                            dataType = "von Mises ";
                        }
                        dataType += "Stress";

                        // Compute absolute data.
                        for (std::vector<float>& attrList : datData) {
#if _OPENMP >= 200805
                            #pragma omp parallel for shared(attrList) default(none)
#endif
                            for (size_t i = 0; i < attrList.size(); i++) {
                                attrList.at(i) = std::abs(attrList.at(i));
                            }
                        }
                    }
                    if (filenameLowerCase.find("vertex") != std::string::npos
                            && filenameLowerCase.find("displacement") != std::string::npos) {
                        dataType = "Vertex Displacement";
                    }

                    int i = 0;
                    for (std::vector<float>& attributeList : datData) {
                        inputData->addManualVertexAttribute(attributeList, dataType + " (" + std::to_string(i) + ")");
                        i++;
                    }
                }
            }
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
    auto loadingFinishedTime = std::chrono::system_clock::now();
    auto elapsedTimeLoad = std::chrono::duration_cast<std::chrono::milliseconds>(
            loadingFinishedTime - loadingStartTime);
    loadingTimeSeconds += elapsedTimeLoad.count() * 1e-3;

    reRender = true;
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
    std::chrono::time_point<std::chrono::system_clock> prepareVisPipelineTimeStart;
    if (printLoadingTime) {
        prepareVisPipelineTimeStart = std::chrono::system_clock::now();
    }

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

    if (printLoadingTime) {
        printLoadingTime = false;
        auto prepareVisPipelineTimeEnd = std::chrono::system_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                prepareVisPipelineTimeEnd - prepareVisPipelineTimeStart);
        loadingTimeSeconds += elapsedTime.count() * 1e-3;
        sgl::Logfile::get()->writeInfo(
                std::string() + "Total loading time until first frame: " + std::to_string(loadingTimeSeconds) + "s");
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
