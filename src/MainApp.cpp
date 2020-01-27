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
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Texture/Bitmap.hpp>

#include "Loaders/VtkLoader.hpp"
#include "Loaders/MeshLoader.hpp"
#include "Loaders/HexaLabDatasets.hpp"
#include "Filters/PlaneFilter.hpp"
#include "Filters/QualityFilter.hpp"
#include "Renderers/SurfaceRenderer.hpp"
#include "Renderers/WireframeRenderer.hpp"
#include "Renderers/VolumeRenderer.hpp"
#include "Renderers/DepthComplexityRenderer.hpp"
#include "MainApp.hpp"

void openglErrorCallback() {
    std::cerr << "Application callback" << std::endl;
}

MainApp::MainApp() : camera(new sgl::Camera()), sceneData(sceneFramebuffer, camera, lightDirection) {
    sgl::FileUtils::get()->ensureDirectoryExists(saveDirectoryScreenshots);

    gammaCorrectionShader = sgl::ShaderManager->getShaderProgram({"GammaCorrection.Vertex", "GammaCorrection.Fragment"});

    sgl::EventManager::get()->addListener(sgl::RESOLUTION_CHANGED_EVENT,
            [this](sgl::EventPtr event){ this->resolutionChanged(event); });

    camera->setNearClipDistance(0.01f);
    camera->setFarClipDistance(100.0f);
    camera->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    float fovy = atanf(1.0f / 2.0f) * 2.0f;
    camera->setFOVy(fovy);
    camera->setPosition(glm::vec3(0.0f, 0.1f, 0.8f));

    clearColor = sgl::Color(255, 255, 255, 255);
    clearColorSelection = ImColor(clearColor.getColorRGBA());
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);

    bool useVsync = sgl::AppSettings::get()->getSettings().getBoolValue("window-vSync");
    if (useVsync) {
        sgl::Timer->setFPSLimit(true, 60);
    } else {
        sgl::Timer->setFPSLimit(false, 60);
    }

    fpsArray.resize(16, 60.0f);
    framerateSmoother = FramerateSmoother(1);

    if (cullBackface) {
        glCullFace(GL_BACK);
        glEnable(GL_CULL_FACE);
    } else {
        glCullFace(GL_BACK);
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->setErrorCallback(&openglErrorCallback);
    sgl::Renderer->setDebugVerbosity(sgl::DEBUG_OUTPUT_CRITICAL_ONLY);
    resolutionChanged(sgl::EventPtr());

    selectedQualityMeasure = QUALITY_MEASURE_SCALED_JACOBIAN;
    changeQualityMeasureType();

    meshLoaderMap.insert(std::make_pair("vtk", new VtkLoader));
    meshLoaderMap.insert(std::make_pair("mesh", new MeshLoader));
    meshFilters.push_back(new PlaneFilter);
    meshFilters.push_back(new QualityFilter);
    setRenderers();

    customMeshFileName = sgl::FileUtils::get()->getUserDirectory();
    hexaLabDataSetsDownloaded = sgl::FileUtils::get()->exists(meshDirectory + "index.json");
    loadAvailableDataSetSources();
}

MainApp::~MainApp() {
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
}

void MainApp::setRenderers() {
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        delete meshRenderer;
    }
    meshRenderers.clear();

    if (renderingMode == RENDERING_MODE_SURFACE) {
        meshRenderers.push_back(new SurfaceRenderer(sceneData, transferFunctionWindow));
        meshRenderers.push_back(new WireframeRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_VOLUME) {
        meshRenderers.push_back(new VolumeRenderer(sceneData, transferFunctionWindow));
    } else if (renderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        meshRenderers.push_back(new DepthComplexityRenderer(sceneData, transferFunctionWindow));
    }
}

void MainApp::resolutionChanged(sgl::EventPtr event) {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    glViewport(0, 0, width, height);

    // Buffers for off-screen rendering
    sceneFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA16;
    } else {
        textureSettings.internalFormat = GL_RGBA8; // GL_RGBA8 For i965 driver to accept image load/store (legacy)
    }
    textureSettings.pixelType = GL_UNSIGNED_BYTE;
    textureSettings.pixelFormat = GL_RGB;
    sceneTexture = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
    sceneFramebuffer->bindTexture(sceneTexture);
    sceneDepthRBO = sgl::Renderer->createRBO(width, height, sgl::DEPTH24_STENCIL8);
    sceneFramebuffer->bindRenderbuffer(sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);

    camera->onResolutionChanged(event);
    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        meshRenderer->onResolutionChanged();
    }

    reRender = true;
}

void MainApp::saveScreenshot(const std::string &filename) {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    sgl::BitmapPtr bitmap(new sgl::Bitmap(width, height, 32));
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, bitmap->getPixels());
    bitmap->savePNG(filename.c_str(), true);
}

void MainApp::updateColorSpaceMode() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    // Buffers for off-screen rendering
    sceneFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA16;
    } else {
        textureSettings.internalFormat = GL_RGBA8; // GL_RGBA8 For i965 driver to accept image load/store (legacy)
    }
    textureSettings.pixelType = GL_UNSIGNED_BYTE;
    textureSettings.pixelFormat = GL_RGB;
    sceneTexture = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
    sceneFramebuffer->bindTexture(sceneTexture);
    sceneDepthRBO = sgl::Renderer->createRBO(width, height, sgl::DEPTH24_STENCIL8);
    sceneFramebuffer->bindRenderbuffer(sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);

    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
}

void MainApp::processSDLEvent(const SDL_Event &event) {
    sgl::ImGuiWrapper::get()->processSDLEvent(event);
}

void MainApp::render() {
    prepareVisualizationPipeline();

    for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
        reRender = reRender || meshRenderer->needsReRender();
    }

    if (reRender || continuousRendering) {
        sgl::Renderer->bindFBO(sceneFramebuffer);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, clearColor);

        sgl::Renderer->setProjectionMatrix(camera->getProjectionMatrix());
        sgl::Renderer->setViewMatrix(camera->getViewMatrix());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

        glEnable(GL_DEPTH_TEST);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
        glBlendEquation(GL_FUNC_ADD);

        if (inputData.get() != nullptr) {
            for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                meshRenderer->render();
            }
        }
        reRender = false;
    }

    // Render to screen
    sgl::Renderer->unbindFBO();
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    if (useLinearRGB) {
        sgl::Renderer->blitTexture(
                sceneTexture, sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)),
                gammaCorrectionShader);
    } else {
        sgl::Renderer->blitTexture(
                sceneTexture, sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)));
    }

    if (!uiOnScreenshot && screenshot) {
        printNow = true;
        saveScreenshot(
                saveDirectoryScreenshots + saveFilenameScreenshots
                + "_" + sgl::toString(screenshotNumber++) + ".png");
        printNow = false;
    }

    renderGUI();
}

void MainApp::renderGUI() {
    sgl::ImGuiWrapper::get()->renderStart();

    if (showSettingsWindow) {
        if (ImGui::Begin("Settings", &showSettingsWindow)) {
            // Draw an FPS counter
            static float displayFPS = 60.0f;
            static uint64_t fpsCounter = 0;
            if (sgl::Timer->getTicksMicroseconds() - fpsCounter > 1e6) {
                displayFPS = ImGui::GetIO().Framerate;
                fpsCounter = sgl::Timer->getTicksMicroseconds();
            }
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / fps, fps);
            ImGui::Separator();

            // Selection of displayed model
            renderFileSelectionSettingsGui();

            ImGui::Separator();

            static bool showSceneSettings = true;
            if (ImGui::CollapsingHeader("Scene Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen)) {
                renderSceneSettingsGUI();
            }
        }
        ImGui::End();
    }

    if (transferFunctionWindow.renderGUI()) {
        reRender = true;
        if (inputData) {
            inputData->onTransferFunctionMapRebuilt();
        }
        if (transferFunctionWindow.getTransferFunctionMapRebuilt()) {
            for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
                meshRenderer->onTransferFunctionMapRebuilt();
            }
        }
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
    MeshSourceDescription& sourceDescription = meshSourceDescriptions.at(selectedFileSourceIndex - 1);
    for (std::string& dataSetName : sourceDescription.data) {
        selectedMeshDataSetNames.push_back(dataSetName);
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

void MainApp::renderSceneSettingsGUI() {
    if (ImGui::ColorEdit3("Clear Color", (float*)&clearColorSelection, 0)) {
        clearColor = sgl::colorFromFloat(
                clearColorSelection.x, clearColorSelection.y, clearColorSelection.z, clearColorSelection.w);
        transferFunctionWindow.setClearColor(clearColor);
        reRender = true;
    }

    // Select light direction
    // Spherical coordinates: (r, θ, φ), i.e. with radial distance r, azimuthal angle θ (theta), and polar angle φ (phi)
    static float theta = sgl::PI/2;
    static float phi = 0.0f;
    bool angleChanged = false;
    angleChanged = ImGui::SliderAngle("Light Azimuth", &theta, 0.0f) || angleChanged;
    angleChanged = ImGui::SliderAngle("Light Polar Angle", &phi, 0.0f) || angleChanged;
    if (angleChanged) {
        // https://en.wikipedia.org/wiki/List_of_common_coordinate_transformations#To_cartesian_coordinates
        lightDirection = glm::vec3(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));
        reRender = true;
    }

    if (ImGui::Checkbox("Cull Back Face", &cullBackface)) {
        if (cullBackface) {
            glEnable(GL_CULL_FACE);
        } else {
            glDisable(GL_CULL_FACE);
        }
        reRender = true;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Continuous Rendering", &continuousRendering);
    ImGui::Checkbox("UI on Screenshot", &uiOnScreenshot);
    ImGui::SameLine();
    if (ImGui::Checkbox("Use Linear RGB", &useLinearRGB)) {
        updateColorSpaceMode();
        reRender = true;
    }
    ImGui::Checkbox("Show Transfer Function Window", &transferFunctionWindow.getShowTransferFunctionWindow());

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

    ImGui::SliderFloat("Move Speed", &MOVE_SPEED, 0.1f, 1.0f);

    ImGui::Separator();

    ImGui::InputText("##savescreenshotlabel", &saveFilenameScreenshots);
    if (ImGui::Button("Save screenshot")) {
        saveScreenshot(
                saveDirectoryScreenshots + saveFilenameScreenshots
                + "_" + sgl::toString(screenshotNumber++) + ".png");
    }
}

void MainApp::update(float dt) {
    sgl::AppLogic::update(dt);

    fpsArrayOffset = (fpsArrayOffset + 1) % fpsArray.size();
    fpsArray[fpsArrayOffset] = 1.0f/dt;

    transferFunctionWindow.update(dt);

    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) {
        // Ignore inputs below
        return;
    }

    // Rotate scene around camera origin
    if (sgl::Keyboard->isKeyDown(SDLK_x)) {
        glm::quat rot = glm::quat(glm::vec3(dt*ROT_SPEED, 0.0f, 0.0f));
        camera->rotate(rot);
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_y)) {
        glm::quat rot = glm::quat(glm::vec3(0.0f, dt*ROT_SPEED, 0.0f));
        camera->rotate(rot);
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_z)) {
        glm::quat rot = glm::quat(glm::vec3(0.0f, 0.0f, dt*ROT_SPEED));
        camera->rotate(rot);
        reRender = true;
    }

    if (sgl::Keyboard->isKeyDown(SDLK_u)) {
        showSettingsWindow = !showSettingsWindow;
        transferFunctionWindow.setShow(showSettingsWindow);
    }


    glm::mat4 rotationMatrix = camera->getRotationMatrix();//glm::mat4(camera->getOrientation());
    glm::mat4 invRotationMatrix = glm::inverse(rotationMatrix);
    if (sgl::Keyboard->isKeyDown(SDLK_PAGEDOWN)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, -dt*MOVE_SPEED, 0.0f)));
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_PAGEUP)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, dt*MOVE_SPEED, 0.0f)));
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_DOWN) || sgl::Keyboard->isKeyDown(SDLK_s)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, 0.0f, dt*MOVE_SPEED)));
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_UP) || sgl::Keyboard->isKeyDown(SDLK_w)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, 0.0f, -dt*MOVE_SPEED)));
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_LEFT) || sgl::Keyboard->isKeyDown(SDLK_a)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(-dt*MOVE_SPEED, 0.0f, 0.0f)));
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_RIGHT) || sgl::Keyboard->isKeyDown(SDLK_d)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(dt*MOVE_SPEED, 0.0f, 0.0f)));
        reRender = true;
    }

    if (io.WantCaptureMouse) {
        // Ignore inputs below
        return;
    }

    // Zoom in/out
    if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
        float moveAmount = sgl::Mouse->getScrollWheel()*dt*2.0;
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, 0.0f, -moveAmount*MOVE_SPEED)));
        reRender = true;

    }

    // Mouse rotation
    if (sgl::Mouse->isButtonDown(1) && sgl::Mouse->mouseMoved()) {
        sgl::Point2 pixelMovement = sgl::Mouse->mouseMovement();
        float yaw = dt*MOUSE_ROT_SPEED*pixelMovement.x;
        float pitch = -dt*MOUSE_ROT_SPEED*pixelMovement.y;

        glm::quat rotYaw = glm::quat(glm::vec3(0.0f, yaw, 0.0f));
        glm::quat rotPitch = glm::quat(pitch*glm::vec3(rotationMatrix[0][0], rotationMatrix[1][0],
                rotationMatrix[2][0]));
        camera->rotateYaw(yaw);
        camera->rotatePitch(pitch);
        reRender = true;
    }
}



// --- Visualization pipeline ---

void MainApp::loadHexahedralMesh(const std::string &fileName) {
    if (fileName.size() == 0) {
        inputData = HexMeshPtr();
        return;
    }

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

    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> cellIndices;
    bool loadingSuccessful = it->second->loadHexahedralMeshFromFile(fileName, vertices, cellIndices);
    if (loadingSuccessful) {
        inputData = HexMeshPtr(new HexMesh(transferFunctionWindow));
        inputData->setHexMeshData(vertices, cellIndices);
        inputData->setQualityMeasure(selectedQualityMeasure);
    }
}

void MainApp::prepareVisualizationPipeline() {
    if (inputData != nullptr) {
        bool isPreviousNodeDirty = inputData->isDirty();
        HexMeshPtr filteredMesh = getFilteredMesh(isPreviousNodeDirty);
        // Generate the visualization mapping for all renderers that have the dirty flag set (or if the filtered data
        // changed).
        for (HexahedralMeshRenderer* meshRenderer : meshRenderers) {
            if (meshRenderer->isDirty() || isPreviousNodeDirty) {
                meshRenderer->generateVisualizationMapping(filteredMesh);
            }
        }
    }
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
}
