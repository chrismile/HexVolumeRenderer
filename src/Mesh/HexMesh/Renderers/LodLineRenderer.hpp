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
#ifndef HEXVOLUMERENDERER_LODLINERENDERER_HPP
#define HEXVOLUMERENDERER_LODLINERENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"
#include "Mesh/HexMesh/Renderers/Intersection/Pickable.hpp"

/**
 * Renders the hexahedral mesh using lines.
 * The further line points are away from a focus center, the more coarse the line representation of the mesh is.
 * The LOD renderer uses the base-complex to create an LOD hierarchy.
 * This renderer (in contrast to LodLineRendererPerFragment) does not use per-fragment tests, but uses an octree
 * to decide what lines are filtered.
 */
class LodLineRenderer : public HexahedralMeshRenderer, protected Pickable {
public:
    LodLineRenderer(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow);
    virtual ~LodLineRenderer() {}

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
    // Updates the internal logic (called once per frame).
    virtual void update(float dt);

protected:
    void reloadSphereRenderData();
    sgl::ShaderProgramPtr shaderProgram;
    sgl::ShaderProgramPtr shaderProgramSurface;
    sgl::ShaderAttributesPtr shaderAttributes;
    sgl::ShaderAttributesPtr focusPointShaderAttributes;

    // GUI data
    bool showRendererWindow = true;
    float focusRadius = 0.05f;
    float lineWidth = 0.0015f;
};

#endif //HEXVOLUMERENDERER_LODLINERENDERER_HPP