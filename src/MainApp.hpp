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

#ifndef MAINAPP_HPP
#define MAINAPP_HPP

#include <string>
#include <vector>
#include <map>

#include <Utils/SciVis/SciVisApp.hpp>
#include <Utils/SciVis/CameraPath.hpp>
#include <Graphics/Shader/Shader.hpp>
#include <Graphics/Video/VideoWriter.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/ColorLegendWidget.hpp>
#include <ImGui/Widgets/CheckpointWindow.hpp>

#include "Mesh/HexMesh/QualityMeasure/QualityMeasure.hpp"
#include "Mesh/HexMesh/Loaders/HexaLabDatasets.hpp"
#include "Mesh/HexMesh/Loaders/HexahedralMeshLoader.hpp"
#include "Mesh/Filters/HexahedralMeshFilter.hpp"
#include "Mesh/HexMesh/Renderers/SceneData.hpp"
#include "Mesh/HexMesh/Renderers/HexahedralMeshRenderer.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"

#ifdef USE_PYTHON
#include "Widgets/ReplayWidget.hpp"
#endif

#ifdef USE_STEAMWORKS
#include "Utils/Steamworks.hpp"
#endif

class DataView;
typedef std::shared_ptr<DataView> DataViewPtr;

class MainApp : public sgl::SciVisApp {
public:
    MainApp();
    ~MainApp();

    /// Replicability stamp mode.
    void loadReplicabilityStampState();

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState);

    void render();
    void update(float dt);
    void resolutionChanged(sgl::EventPtr event);

private:
    /// Renders the GUI of the scene settings and all filters and renderers.
    void renderGui();
    /// Renders the GUI for selecting an input file.
    void renderFileSelectionSettingsGui();
    /// Render the scene settings GUI, e.g. for setting the background clear color.
    void renderSceneSettingsGui();
    /// Update the color space (linear RGB vs. sRGB).
    void updateColorSpaceMode();
    /// Callback when a file has been dropped on the program.
    void onFileDropped(const std::string& droppedFileName) override;
    bool checkHasValidExtension(const std::string& filenameLower);

#ifdef USE_STEAMWORKS
    Steamworks steamworks;
#endif

    // Dock space mode.
    //void renderGuiMenuBar();
    //void renderGuiPropertyEditorBegin() override;
    //void renderGuiPropertyEditorCustomNodes() override;
    bool scheduledDockSpaceModeChange = false;
    bool newDockSpaceMode = false;
    int focusedWindowIndex = -1;
    int mouseHoverWindowIndex = -1;
    bool showRendererWindow = true;
    DataViewPtr dataView;
    sgl::CameraPtr cameraHandle;
    std::vector<std::string> rendererWindowNames;

    /// Scene data (e.g., camera, main framebuffer, ...).
    RayMeshIntersection* rayMeshIntersection;
    SceneData sceneData;

    // This setting lets all data views use the same viewport resolution.
    bool useFixedSizeViewport = false;
    glm::ivec2 fixedViewportSizeEdit{ 2186, 1358 };
    glm::ivec2 fixedViewportSize{ 2186, 1358 };

    /// Scene data used in user interface.
    RenderingMode renderingMode = RENDERING_MODE_SURFACE;

    // Data set GUI information.
    void loadAvailableDataSetSources();
    void loadSelectedMeshDataSetNames();
    std::string getSelectedMeshFilename();
    /// Returns true if the selected file source supports deformation meshes (for now a hardcoded feature).
    bool getFileSourceContainsDeformationMeshes();
    bool hexaLabDataSetsDownloaded = false;
    std::vector<MeshSourceDescription> meshSourceDescriptions;
    std::vector<std::string> meshDataSetSources; //< Contains "Local file..." at beginning, thus starts actually at 1.
    std::vector<std::string> selectedMeshDataSetNames; //< Contains "None" at beginning, thus starts actually at 1.
    int selectedFileSourceIndex = 0; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int selectedMeshIndex = 0; //< Contains "None" at beginning, thus starts actually at 1.
    int currentlyLoadedFileSourceIndex = -1;
    int currentlySelectedMeshIndex = -1;
    std::string loadedMeshFilename;
    std::string customMeshFileName;
    float deformationFactor = 0.0f;

    // Coloring & filtering dependent on importance criteria.
    QualityMeasure selectedQualityMeasure;
    sgl::TransferFunctionWindow transferFunctionWindow;

    // Color legend widgets for different attributes.
    bool shallRenderColorLegendWidgets = true;
    sgl::ColorLegendWidget colorLegendWidget;

    // For downloading files in the background.
    LoaderThread loaderThread;

    // For making performance measurements.
    bool testSortingPerformance = true;
    AutomaticPerformanceMeasurer *performanceMeasurer = nullptr;
    InternalState lastState;
    bool firstState = true;
    bool usesNewState = true;

#ifdef USE_PYTHON
    ReplayWidget replayWidget;
    bool realTimeReplayUpdates = false;
#endif


    /// --- Visualization pipeline ---

    /// Loads a hexahedral mesh from a file.
    void loadHexahedralMesh(const std::string &fileName);
    /// Reload the currently loaded data set.
    virtual void reloadDataSet() override;
    /// Updates the internal representation of a deformable mesh.
    void onDeformationFactorChanged();
    /// Prepares the visualization pipeline for rendering.
    void prepareVisualizationPipeline();
    /// Returns the filtered mesh that is passed to the renderers.
    HexMeshPtr getFilteredMesh(bool& isDirty);
    /// Change the importance criterion used for coloring.
    void changeQualityMeasureType();
    /// Sets the used renderers
    void setRenderers();

    /// Helpers.
    sgl::AABB3 computeAABB3(const std::vector<glm::vec3>& vertices);
    void normalizeVertexPositions(std::vector<glm::vec3>& vertices);
    void applyVertexDeformations(
            std::vector<glm::vec3>& vertices,
            const std::vector<glm::vec3>& deformations,
            const float deformationFactor);
    sgl::AABB3 modelBoundingBox;

    /// The data loaded from the input file (or a wrapped nullptr).
    HexMeshPtr inputData;
    bool newMeshLoaded = true;

    // Timer for measuring how long data loading & mesh processing takes.
    bool printLoadingTime = false;
    double loadingTimeSeconds = 0.0;

    /// A copy of the mesh data is stored for allowing the user to alter the deformation factor also after loading.
    std::vector<glm::vec3> hexMeshVertices;
    std::vector<uint32_t> hexMeshCellIndices;
    std::vector<glm::vec3> hexMeshDeformations;
    std::vector<float> hexMeshAttributeList;

    /// A list of filters that are applied sequentially on the data.
    std::vector<HexahedralMeshFilter*> meshFilters;
    /// A list of rendering methods that use the output of the concatenation of all mesh filters for rendering.
    std::vector<HexahedralMeshRenderer*> meshRenderers;

    // Visualization pipeline node types.
    std::map<std::string, HexahedralMeshLoader*> meshLoaderMap;
    //std::map<std::string, HexahedralMeshFilter*> meshFilterMap;
    //std::map<std::string, HexahedralMeshRenderer*> meshRendererMap;
};

#endif // MAINAPP_HPP