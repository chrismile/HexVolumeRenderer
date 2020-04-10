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

#ifndef HEXVOLUMERENDERER_PERFECTMATCHING_HPP
#define HEXVOLUMERENDERER_PERFECTMATCHING_HPP

#include <lemon/matching.h>
#include <lemon/smart_graph.h>
#include "HexahedralSheet.hpp"

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

class PerfectMatching {
public:
    PerfectMatching(
            HexMeshPtr hexMesh, std::vector<SheetComponent>& components,
            std::vector<ComponentConnectionData>& connectionDataList);
    inline bool getCouldMatchAny() { return couldMatchAny; }
    inline std::vector<std::pair<size_t, size_t>>& getMatchedComponents() { return matchedComponents; }
    inline std::vector<size_t>& getUnmatchedComponents() { return unmatchedComponents; }

private:
    bool couldMatchAny;
    std::vector<std::pair<size_t, size_t>> matchedComponents;
    std::vector<size_t> unmatchedComponents;
};


#endif //HEXVOLUMERENDERER_PERFECTMATCHING_HPP
