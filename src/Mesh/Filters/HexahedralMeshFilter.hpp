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

#ifndef FILTERS_HEXAHEDRALMESHFILTER_HPP
#define FILTERS_HEXAHEDRALMESHFILTER_HPP

#include "Mesh/HexMesh/HexMesh.hpp"

struct InternalState;
class SettingsMap;

class HexahedralMeshFilter {
public:
    virtual ~HexahedralMeshFilter() {}

    // Returns if the filter is active and should be applied on the input data.
    inline bool isEnabled() { return enabled; }
    // Returns if the visualization mapping needs to be re-generated.
    inline bool isDirty() { return dirty; }

    // Called when a new mesh is loaded from a file.
    virtual void onMeshLoaded(HexMeshPtr meshIn) {}

    // Gets a copy of mesh from parent if parent has multiple children, i.e., operations must use different meshes.
    virtual void filterMesh(HexMeshPtr meshIn)=0;
    inline HexMeshPtr getOutput() { return output; }
    inline void removeOldMesh() { output = HexMeshPtr(); }

    // Renders the GUI. The "dirty" flag might be set depending on the user's actions.
    virtual void renderGui()=0;

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState) {}
    virtual void setNewSettings(const SettingsMap& settings) {}

protected:
    HexMeshPtr output;
    bool enabled = true;
    bool dirty = true;
    bool showFilterWindow = true;
};

#endif // FILTERS_HEXAHEDRALMESHFILTER_HPP