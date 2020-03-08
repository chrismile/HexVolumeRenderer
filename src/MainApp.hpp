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

#include "QualityMeasure/QualityMeasure.hpp"
#include "Loaders/HexaLabDatasets.hpp"
#include "Loaders/HexahedralMeshLoader.hpp"
#include "Filters/HexahedralMeshFilter.hpp"
#include "Renderers/SceneData.hpp"
#include "Renderers/TransferFunctionWindow.hpp"
#include "Renderers/HexahedralMeshRenderer.hpp"

enum RenderingMode {
    RENDERING_MODE_SURFACE, RENDERING_MODE_VOLUME, RENDERING_MODE_DEPTH_COMPLEXITY,
    RENDERING_MODE_SINGULARITY, RENDERING_MODE_BASE_COMPLEX_LINES, RENDERING_MODE_BASE_COMPLEX_SURFACE,
    RENDERING_MODE_PARTITION_LINES, RENDERING_MODE_LOD_LINES
};
const char *const RENDERING_MODE_NAMES[] = {
        "Surface", "Volume", "Depth Complexity",
        "Singularity", "Base Complex (Lines)", "Base Complex (Surface)",
        "Partition Lines", "LOD Lines"
};
const int NUM_RENDERING_MODES = ((int)(sizeof(QUALITY_MEASURE_NAMES) / sizeof(*QUALITY_MEASURE_NAMES)));


class MainApp : public sgl::AppLogic {
public:
    MainApp();
    ~MainApp();
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
    // Override screenshot function to exclude GUI (if wanted by the user)
    void saveScreenshot(const std::string &filename);

    /// Scene data (e.g., camera, main framebuffer, ...).
    sgl::CameraPtr camera;
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
    bool cullBackface = true;
    RenderingMode renderingMode = RENDERING_MODE_SURFACE;
    bool useLinearRGB = true;
    std::vector<float> fpsArray;
    size_t fpsArrayOffset = 0;
    glm::vec3 lightDirection = glm::vec3(1.0, 0.0, 0.0);
    bool uiOnScreenshot = false;
    bool printNow = false;
    std::string saveDirectoryScreenshots = "Data/Screenshots/";
    std::string saveFilenameScreenshots = "Screenshot";
    int screenshotNumber = 0;
    float MOVE_SPEED = 0.2f;
    float ROT_SPEED = 1.0f;
    float MOUSE_ROT_SPEED = 0.05f;

    // Data set GUI information.
    void loadAvailableDataSetSources();
    void loadSelectedMeshDataSetNames();
    std::string getSelectedMeshFilename();
    bool hexaLabDataSetsDownloaded = false;
    std::vector<MeshSourceDescription> meshSourceDescriptions;
    std::vector<std::string> meshDataSetSources; //< Contains "Local file..." at beginning, thus starts actually at 1.
    std::vector<std::string> selectedMeshDataSetNames; //< Contains "None" at beginning, thus starts actually at 1.
    int selectedFileSourceIndex = 0; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int selectedMeshIndex = 0; //< Contains "None" at beginning, thus starts actually at 1.
    std::string customMeshFileName;

    // Continuous rendering: Re-render each frame or only when scene changes?
    bool continuousRendering = false;
    bool reRender = true;

    // Coloring & filtering dependent on importance criteria.
    QualityMeasure selectedQualityMeasure;
    TransferFunctionWindow transferFunctionWindow;

    // For downloading files in the background.
    LoaderThread loaderThread;


    /// --- Visualization pipeline ---

    /// Loads a hexahedral mesh from a file.
    void loadHexahedralMesh(const std::string &fileName);
    /// Prepares the visualization pipeline for rendering.
    void prepareVisualizationPipeline();
    /// Returns the filtered mesh that is passed to the renderers.
    HexMeshPtr getFilteredMesh(bool& isDirty);
    /// Change the importance criterion used for coloring.
    void changeQualityMeasureType();
    /// Sets the used renderers
    void setRenderers();

    /// The data loaded from the input file (or a wrapped nullptr).
    HexMeshPtr inputData;

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