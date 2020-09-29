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

#include <Utils/AppLogic.hpp>
#include <Graphics/Shader/Shader.hpp>
#include <Graphics/Video/VideoWriter.hpp>

#include "Mesh/HexMesh/QualityMeasure/QualityMeasure.hpp"
#include "Mesh/HexMesh/Loaders/HexaLabDatasets.hpp"
#include "Mesh/HexMesh/Loaders/HexahedralMeshLoader.hpp"
#include "Mesh/Filters/HexahedralMeshFilter.hpp"
#include "Mesh/HexMesh/Renderers/SceneData.hpp"
#include "Mesh/HexMesh/Renderers/HexahedralMeshRenderer.hpp"
#include "Mesh/HexMesh/Renderers/Widgets/TransferFunctionWindow.hpp"
#include "Mesh/HexMesh/Renderers/Widgets/CheckpointWindow.hpp"
#include "Utils/CameraPath.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"

#ifdef USE_STEAMWORKS
#include "Utils/Steamworks.hpp"
#endif

class MainApp : public sgl::AppLogic {
public:
    MainApp();
    ~MainApp();

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState);

    void render();
    void update(float dt);
    void resolutionChanged(sgl::EventPtr event);
    void processSDLEvent(const SDL_Event &event);

private:
    /// Renders the GUI of the scene settings and all filters and renderers.
    void renderGUI();
    /// Renders the GUI for selecting an input file.
    void renderFileSelectionSettingsGui();
    /// Render the scene settings GUI, e.g. for setting the background clear color.
    void renderSceneSettingsGUI();
    /// Update the color space (linear RGB vs. sRGB).
    void updateColorSpaceMode();
    /// Override screenshot function to exclude GUI (if wanted by the user)
    void saveScreenshot(const std::string &filename);
    void makeScreenshot() {}

#ifdef USE_STEAMWORKS
    Steamworks steamworks;
#endif

    /// Scene data (e.g., camera, main framebuffer, ...).
    sgl::CameraPtr camera;
    RayMeshIntersection* rayMeshIntersection;
    SceneData sceneData;

    // Off-screen rendering
    sgl::FramebufferObjectPtr sceneFramebuffer;
    sgl::TexturePtr sceneTexture;
    sgl::RenderbufferObjectPtr sceneDepthRBO;
    sgl::ShaderProgramPtr gammaCorrectionShader;

    /// Scene data used in user interface.
    bool showSettingsWindow = true;
    sgl::Color clearColor;
    ImVec4 clearColorSelection = ImColor(0, 0, 0, 255);
    RenderingMode renderingMode = RENDERING_MODE_SURFACE;
    bool useLinearRGB = true;
    std::vector<float> fpsArray;
    size_t fpsArrayOffset = 0;
    bool uiOnScreenshot = false;
    bool printNow = false;
    std::string saveDirectoryScreenshots = "Data/Screenshots/";
    std::string saveFilenameScreenshots = "Screenshot";
    int screenshotNumber = 0;
    std::string saveDirectoryVideos = "Data/Videos/";
    std::string saveFilenameVideos = "Video";
    int videoNumber = 0;
    float MOVE_SPEED = 0.2f;
    float MOUSE_ROT_SPEED = 0.05f;
    float fovDegree = 90.0f;
    glm::vec3 modelRotationAxis = glm::vec3(1.0f, 0.0f, 0.0f);
    int rotateModelBy90DegreeTurns = 0;
    glm::ivec2 windowResolution;

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
    std::string customMeshFileName;
    float deformationFactor = 0.0f;

    // Continuous rendering: Re-render each frame or only when scene changes?
    bool continuousRendering = false;
    bool reRender = true;

    // Coloring & filtering dependent on importance criteria.
    QualityMeasure selectedQualityMeasure;
    TransferFunctionWindow transferFunctionWindow;

    // For loading and saving camera checkpoints.
    CheckpointWindow checkpointWindow;

    // For downloading files in the background.
    LoaderThread loaderThread;

    // For recording videos.
    bool recording = false;
    glm::ivec2 recordingResolution = glm::ivec2(2560, 1440); // 1920, 1080
    sgl::VideoWriter* videoWriter = nullptr;
    const int FRAME_RATE_VIDEOS = 30;
    float recordingTime = 0.0f;
    float recordingTimeLast = 0.0f;
    uint64_t recordingTimeStampStart;

    // Camera paths for recording videos without human interaction.
    CameraPath cameraPath;
    bool useCameraFlight = false;
    bool startedCameraFlightPerUI = false;
    bool realTimeCameraFlight = true; // Move camera in real elapsed time or camera frame rate?
    const std::string saveDirectoryCameraPaths = "Data/CameraPaths/";
    float FRAME_TIME_CAMERA_PATH = 1.0f / FRAME_RATE_VIDEOS; ///< Simulate constant frame rate.
    const float CAMERA_PATH_TIME_RECORDING = 30.0f;
    const float CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT = TIME_PERFORMANCE_MEASUREMENT;

    // For making performance measurements.
    bool usePerformanceMeasurementMode = false;
    bool testSortingPerformance = true;
    AutomaticPerformanceMeasurer *performanceMeasurer = nullptr;
    InternalState lastState;
    bool firstState = true;
    bool usesNewState = true;


    /// --- Visualization pipeline ---

    /// Loads a hexahedral mesh from a file.
    void loadHexahedralMesh(const std::string &fileName);
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

    /// A copy of the mesh data is stored for allowing the user to alter the deformation factor also after loading.
    std::vector<glm::vec3> hexMeshVertices;
    std::vector<uint32_t> hexMeshCellIndices;
    std::vector<glm::vec3> hexMeshDeformations;
    std::vector<float> hexMeshAnistropyMetricList;

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