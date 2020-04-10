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

#include "SheetComponentMatching.hpp"

SheetComponentMatching::SheetComponentMatching(
        HexMeshPtr hexMesh, std::vector<SheetComponent>& components,
        std::vector<ComponentConnectionData>& connectionDataList) {
    lemon::SmartGraph graph;
    lemon::SmartGraph::EdgeMap<float> weights(graph);

    // Add all graph vertices and edges.
    for (size_t i = 0; i < components.size(); i++) {
        graph.addNode();
    }
    for (size_t i = 0; i < connectionDataList.size(); i++) {
        ComponentConnectionData& connectionData = connectionDataList.at(i);
        lemon::SmartGraph::Node node0 = graph.nodeFromId(connectionData.firstIdx);
        lemon::SmartGraph::Node node1 = graph.nodeFromId(connectionData.secondIdx);
        lemon::SmartGraph::Edge edge0 = graph.addEdge(node0, node1);
        weights.set(edge0, connectionData.weight);
    }

    // http://lemon.cs.elte.hu/pub/doc/1.3.1/a00263.html
    lemon::MaxWeightedMatching<lemon::SmartGraph, lemon::SmartGraph::EdgeMap<float>> matching(graph, weights);
    matching.fractionalInit(); // init
    matching.start();

    // Find out which components (vertices) were matched and which were not matched.
    std::unordered_set<size_t> matchedComponentSet;

    for (lemon::SmartGraph::EdgeIt e(graph); e != lemon::INVALID; ++e) {
        if (matching.matching(e)) {
            couldMatchAny = true;
            size_t componentIdx0 = graph.id(graph.u(e));
            size_t componentIdx1 = graph.id(graph.v(e));
            matchedComponents.push_back(std::make_pair(componentIdx0, componentIdx1));
            matchedComponentSet.insert(componentIdx0);
            matchedComponentSet.insert(componentIdx1);
        }
    }

    for (size_t componentIdx = 0; componentIdx < components.size(); componentIdx++) {
        if (matchedComponentSet.find(componentIdx) == matchedComponentSet.end()) {
            unmatchedComponents.push_back(componentIdx);
        }
    }
}
