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

#include "InternalState.hpp"

void getTestModesDepthComplexity(std::vector<InternalState> &states, InternalState state) {
    state.renderingMode = RENDERING_MODE_DEPTH_COMPLEXITY;
    state.name = "Depth Complexity";
    states.push_back(state);
}

void getTestModesClearViewUnified(std::vector<InternalState> &states, InternalState state) {
    state.renderingMode = RENDERING_MODE_CLEAR_VIEW_FACES_UNIFIED;
    state.name = "ClearView";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "perVertexAttributes", "true" },
            { "lineWidthBoostFactor", "2.0" },
            { "focusRadiusBoostFactor", "1.0" },
    })};
    states.push_back(state);
    //state.name = "ClearView Some Other Mode";
    //state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
    //        { "perVertexAttributes", "false" },
    //})};
    //states.push_back(state);
}

void getTestModesPaperForMesh(std::vector<InternalState> &states, InternalState state) {
    getTestModesDepthComplexity(states, state);
    getTestModesClearViewUnified(states, state);
}

std::vector<InternalState> getTestModesPaper()
{
    std::vector<InternalState> states;
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(1280, 720), glm::ivec2(1920, 1080), glm::ivec2(2560, 1440) };
    std::vector<glm::ivec2> windowResolutions = { glm::ivec2(1920, 1080) };
    std::vector<MeshDescriptor> meshDescriptors = {
            MeshDescriptor(
                    "2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation",
                    "anc101_a1", "mesh") };
    std::vector<std::string> transferFunctionNames = { };
    InternalState state;

    for (size_t i = 0; i < windowResolutions.size(); i++) {
        state.windowResolution = windowResolutions.at(i);
        for (size_t j = 0; j < meshDescriptors.size(); j++) {
            state.meshDescriptor = meshDescriptors.at(j);
            if (transferFunctionNames.size() > 0) {
                state.transferFunctionName = transferFunctionNames.at(j);
            }
            getTestModesPaperForMesh(states, state);
        }
    }

    // Append model name to state name if more than one model is loaded
    if (meshDescriptors.size() >= 1 || windowResolutions.size() > 1) {
        for (InternalState &state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x) + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.meshDescriptor.meshName + " " + state.name;
        }
    }

    return states;
}
