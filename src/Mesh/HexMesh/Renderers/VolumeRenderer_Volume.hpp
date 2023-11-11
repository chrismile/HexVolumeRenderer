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

#ifndef HEXVOLUMERENDERER_VOLUMERENDERER_VOLUME_HPP
#define HEXVOLUMERENDERER_VOLUMERENDERER_VOLUME_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

/**
 * Renders all cells with transparency values determined by the transfer function set by the user.
 * The resulting opacity value is modulated using the distance traveled in each cell.
 * For this, the order-independent transparency (OIT) technique per-pixel linked lists are used.
 * For more details see: Yang, J. C., Hensley, J., Grün, H. and Thibieroz, N., "Real-Time Concurrent
 * Linked List Construction on the GPU", Computer Graphics Forum, 29, 2010.
 *
 * For a comparison of different OIT algorithms see:
 * M. Kern, C. Neuhauser, T. Maack, M. Han, W. Usher and R. Westermann, "A Comparison of Rendering Techniques for 3D
 * Line Sets with Transparency," in IEEE Transactions on Visualization and Computer Graphics, 2020.
 * doi: 10.1109/TVCG.2020.2975795
 * URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9007507&isnumber=4359476
 *
 * For rendering the cells as a volume, all back faces and front faces of the cells are rendered.
 * The distance traveled within a cell is determined as the sum of the differences in depth between consecutive pairs
 * of back faces and front faces.
 */
class VolumeRenderer_Volume : public HexahedralMeshRenderer {
public:
    VolumeRenderer_Volume(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~VolumeRenderer_Volume() {}

    static const char* getWindowName() { return "Volume Renderer"; }

    /**
     * Re-generates the visualization mapping.
     * @param meshIn The mesh to generate a visualization mapping for.
     * @param isNewMesh Whether a new mesh is loaded or just a new renderer is used.
     */
    virtual void uploadVisualizationMapping(HexMeshPtr meshIn, bool isNewMesh);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

protected:
    void setSortingAlgorithmDefine();
    void setUniformData();
    void clear();
    void gather();
    void resolve();

    // The rendering data for the volume object.
    sgl::ShaderAttributesPtr shaderAttributesVolumeFrontFaces;
    sgl::ShaderAttributesPtr shaderAttributesVolumeBackFaces;

    // Per-pixel linked list data.
    sgl::GeometryBufferPtr fragmentBufferVolumeFrontFaces;
    sgl::GeometryBufferPtr fragmentBufferVolumeBackFaces;
    sgl::GeometryBufferPtr fragmentBufferSurface;
    sgl::GeometryBufferPtr startOffsetVolumeFrontFaces;
    sgl::GeometryBufferPtr startOffsetVolumeBackFaces;
    sgl::GeometryBufferPtr startOffsetSurface;
    sgl::GeometryBufferPtr fragCounterVolumeFrontFaces;
    sgl::GeometryBufferPtr fragCounterVolumeBackFaces;
    sgl::GeometryBufferPtr fragCounterSurface;

    // The shaders for rendering.
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr gatherShaderVolumeFrontFaces;
    sgl::ShaderProgramPtr gatherShaderVolumeBackFaces;
    sgl::ShaderProgramPtr gatherShaderSurface;
    sgl::ShaderProgramPtr resolveShader;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;

    // GUI data
    bool showRendererWindow = true;
    bool useWeightedVertexAttributes = false;
};

#endif //HEXVOLUMERENDERER_VOLUMERENDERER_VOLUME_HPP
