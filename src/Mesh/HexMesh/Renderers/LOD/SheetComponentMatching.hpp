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

#ifndef HEXVOLUMERENDERER_SHEETCOMPONENTMATCHING_HPP
#define HEXVOLUMERENDERER_SHEETCOMPONENTMATCHING_HPP

#include <lemon/matching.h>
#include <lemon/smart_graph.h>
#include "HexahedralSheet.hpp"

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

/**
 * Matches a set of components (vertices of the graph) using the component connection data passed in the constructor
 * (representing the graph edges).
 */
class SheetComponentMatching {
public:
    /// Computes the component matching using the passed data. The functions below can be used to query the result.
    SheetComponentMatching(
            HexMeshPtr hexMesh, std::vector<SheetComponent>& components,
            std::vector<ComponentConnectionData>& connectionDataList);

    /// Returns whether a matching of any vertices could be performed (i.e., whether the mesh had any edges).
    inline bool getCouldMatchAny() { return couldMatchAny; }
    /// Returns a list of pairs of matched components by their index in the component array.
    inline std::vector<std::pair<size_t, size_t>>& getMatchedComponents() { return matchedComponents; }
    /// Returns a list of unmatched components by their index in the component array.
    inline std::vector<size_t>& getUnmatchedComponents() { return unmatchedComponents; }

private:
    bool couldMatchAny = false;
    std::vector<std::pair<size_t, size_t>> matchedComponents;
    std::vector<size_t> unmatchedComponents;
};


#endif //HEXVOLUMERENDERER_SHEETCOMPONENTMATCHING_HPP
