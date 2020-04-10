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

#ifndef HEXVOLUMERENDERER_LODSHEETGENERATION_HPP
#define HEXVOLUMERENDERER_LODSHEETGENERATION_HPP

#include <vector>
#include <memory>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

/**
 * Uses hexahedral mesh sheets to compute a level of detail structure of the grid lines.
 * The sheets are merged to so-called components.
 *
 * What is a component?
 * - A component is a union of one or multiple base complex sheets.
 *
 * How is an LOD structure created using components?
 * - Initially, every sheet belongs to its own component.
 * - Then, in each iteration until only one component is left, we create a perfect matching of pairs of neighboring
 *   components. We merge these pairs of components into larger components and mark all shared edges as not visible at
 *   the LOD level of the current iteration. (TODO: Exception maybe for singular edges?)
 *
 * What does 'neighboring' mean for two components?
 * - Component c_0 shares at least one boundary face with another component c_1.
 *
 * How can we derive the neighborhood relation from two components c_0, c_1 when merging them to a component c'?
 * - Neighbors(c') = (Neighbors(c_0) UNION Neighbors(c_1)) \ {c_0, c_1}
 *
 * What edges do we mark as not visible when merging two components c_0, c_1?
 * - Mark all edges E(c_0) INTERSECTION E(c_1) as not visible at the current LOD level (and all higher LOD levels of
 *   course, too).
 * (TODO: Exception maybe for singular edges?)
 *
 * @param hexMesh The hexahedral mesh.
 * @param lineVertices The list of line vertex positions.
 * @param lineColors The list of line vertex colors.
 * @param lineLodValues The list of line indices.
 */
void generateSheetLevelOfDetailLineStructure(
        const HexMeshPtr& hexMesh,
        std::vector<glm::vec3> &lineVertices,
        std::vector<glm::vec4> &lineColors,
        std::vector<float> &lineLodValues);

#endif //HEXVOLUMERENDERER_LODSHEETGENERATION_HPP
