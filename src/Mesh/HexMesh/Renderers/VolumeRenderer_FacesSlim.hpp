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

#ifndef HEXVOLUMERENDERER_VOLUMERENDERER_FACESSLIM_H
#define HEXVOLUMERENDERER_VOLUMERENDERER_FACESSLIM_H

#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/OpenGL/TimerGL.hpp>

#include "PPLL/PerPixelLinkedList.hpp"
#include "HexahedralMeshRenderer.hpp"
#include "EdgeDetection/EdgeDetectionRenderer.hpp"

/**
 * Renders all faces with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique per-pixel linked lists are used.
 * For more details see: Yang, J. C., Hensley, J., Grün, H. and Thibieroz, N., "Real-Time Concurrent
 * Linked List Construction on the GPU", Computer Graphics Forum, 29, 2010.
 *
 * For a comparison of different OIT algorithms see:
 * M. Kern, C. Neuhauser, T. Maack, M. Han, W. Usher and R. Westermann, "A Comparison of Rendering Techniques for 3D
 * Line Sets with Transparency," in IEEE Transactions on Visualization and Computer Graphics, 2020.
 * doi: 10.1109/TVCG.2020.2975795
 * URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9007507&isnumber=4359476
 */
class VolumeRenderer_FacesSlim : public HexahedralMeshRenderer, protected EdgeDetectionRenderer {
public:
    VolumeRenderer_FacesSlim(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~VolumeRenderer_FacesSlim();
    virtual bool getUsesSlimRepresentation() override { return true; }

    static const char* getWindowName() { return "Pseudo Volume Renderer"; }

    /**
     * Re-generates the visualization mapping.
     * @param meshIn The mesh to generate a visualization mapping for.
     * @param isNewMesh Whether a new mesh is loaded or just a new renderer is used.
     */
    virtual void uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh);

    /// Removes the old mesh.
    virtual void removeOldMesh() override {
        HexahedralMeshRenderer::removeOldMesh();
        removeOldMeshEdgeDetection();
    }

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState);
    virtual void setNewSettings(const SettingsMap& settings);

protected:
    void updateLargeMeshMode();
    void reallocateFragmentBuffer();
    void reloadGatherShader(bool copyShaderAttributes);
    void reloadResolveShader();
    void setSortingAlgorithmDefine();
    void setUniformData();
    void clear();
    void gather();
    void resolve();

    // Sorting algorithm for PPLL.
    SortingAlgorithmMode sortingAlgorithmMode = SORTING_ALGORITHM_MODE_PRIORITY_QUEUE;

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributes;

    // Multi-var information (optional mode if more than one vertex attribute available).
    bool isMultiVarData = false;
    bool useMultiVarData = false;
    int multiVarAttrIdx = 0;

    // Per-pixel linked list data.
    FragmentBufferMode fragmentBufferMode = FragmentBufferMode::BUFFER;
    size_t maxStorageBufferSize = 0;
    size_t maxDeviceMemoryBudget = 0;
    size_t fragmentBufferSize = 0;
    size_t numFragmentBuffers = 1;
    size_t cachedNumFragmentBuffers = 1;
    sgl::GeometryBufferPtr fragmentBuffer; //< if fragmentBufferMode == FragmentBufferMode::BUFFER
    std::vector<sgl::GeometryBufferPtr> fragmentBuffers; //< if fragmentBufferMode != FragmentBufferMode::BUFFER
    sgl::GeometryBufferPtr startOffsetBuffer;
    sgl::GeometryBufferPtr atomicCounterBuffer;

    // Per-pixel linked list settings.
    LargeMeshMode largeMeshMode = MESH_SIZE_MEDIUM;
    int expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[0][0];
    int expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[0][1];

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderProgramPtr resolveShader;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;

    // Data for performance measurements.
    int frameCounter = 0;
    std::string currentStateName;
    bool timerDataIsWritten = true;
    sgl::TimerGL* timer = nullptr;

    // Window data.
    int windowWidth = 0;
    int windowHeight = 0;

    // GUI data
    bool showRendererWindow = true;
};

#endif //HEXVOLUMERENDERER_VOLUMERENDERER_FACESSLIM_H
