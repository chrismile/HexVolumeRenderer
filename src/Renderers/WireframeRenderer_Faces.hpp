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

#ifndef HEXVOLUMERENDERER_WIREFRAMERENDERER_FACES_HPP
#define HEXVOLUMERENDERER_WIREFRAMERENDERER_FACES_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

/**
 * Renders the hexahedral mesh as a wireframe representation of its edges using its faces.
 *
 * For more details on the base idea see:
 * https://catlikecoding.com/unity/tutorials/advanced-rendering/flat-and-wireframe-shading/
 */
class WireframeRenderer_Faces : public HexahedralMeshRenderer {
public:
    WireframeRenderer_Faces(SceneData &sceneData, TransferFunctionWindow &transferFunctionWindow);
    virtual ~WireframeRenderer_Faces() {}

    // Re-generates the visualization mapping.
    virtual void generateVisualizationMapping(HexMeshPtr meshIn);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged() {}

    // Called when the transfer function was changed.
    virtual void onTransferFunctionMapRebuilt() {}

protected:
    sgl::ShaderProgramPtr shaderProgram;
    sgl::ShaderAttributesPtr shaderAttributes;
};

#endif //HEXVOLUMERENDERER_WIREFRAMERENDERER_FACES_HPP
