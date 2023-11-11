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

#ifndef HEXVOLUMERENDERER_SINGULARITYTYPECOUNTERRENDERER_HPP
#define HEXVOLUMERENDERER_SINGULARITYTYPECOUNTERRENDERER_HPP

#include <map>
#include <Graphics/Shader/ShaderAttributes.hpp>

#include "HexahedralMeshRenderer.hpp"

/**
 * Renders the hexahedral mesh as a wireframe representation of its edges using its faces.
 *
 * For more details on the base idea see:
 * https://catlikecoding.com/unity/tutorials/advanced-rendering/flat-and-wireframe-shading/
 */
class SingularityTypeCounterRenderer : public HexahedralMeshRenderer {
public:
    /**
     * @param sceneData A reference to the scene data object.
     * @param transferFunctionWindow The transfer function editor window.
     */
    SingularityTypeCounterRenderer(
            SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    virtual ~SingularityTypeCounterRenderer() {}

    static const char* getWindowName() { return "Wireframe Renderer (Faces)"; }

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

protected:
    // GUI data.
    bool showRendererWindow = true;
    unsigned int numCircleSEs = 0;
    unsigned int numOpenSEs = 0;
    std::map<unsigned int, unsigned int> irregularBoundaryEdgeMap; ///< Maps valence to number of occurences.
    std::map<unsigned int, unsigned int> irregularInnerEdgeMap; ///< Maps valence to number of occurences.
};

#endif //HEXVOLUMERENDERER_SINGULARITYTYPECOUNTERRENDERER_HPP
