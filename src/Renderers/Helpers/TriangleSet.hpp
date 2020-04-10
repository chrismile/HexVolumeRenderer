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

#ifndef HEXVOLUMERENDERER_TRIANGLESET_HPP
#define HEXVOLUMERENDERER_TRIANGLESET_HPP

#include <algorithm>
#include <unordered_set>

struct TriangleKey {
    TriangleKey(uint32_t idx0, uint32_t idx1, uint32_t idx2) {
        vertexIndices[0] = idx0;
        vertexIndices[1] = idx1;
        vertexIndices[2] = idx2;
        sortedVertexIndices[0] = idx0;
        sortedVertexIndices[1] = idx1;
        sortedVertexIndices[2] = idx2;
        std::sort(sortedVertexIndices, sortedVertexIndices + 3);
    }

    bool operator==(const TriangleKey& rhs) const {
        for (int i = 0; i < 3; i++) {
            if (sortedVertexIndices[i] != rhs.sortedVertexIndices[i]) {
                return false;
            }
        }
        return true;
    }

    uint32_t vertexIndices[3];
    uint32_t sortedVertexIndices[3];
};

// Hash Function: H(i,j,k) = (i0*p_1 xor i1*jp_2 xor i2*jp_3) mod n
struct TriangleKeyHasher {
    std::size_t operator()(const TriangleKey& key) const {
        const size_t PRIME_NUMBERS[] = { 50331653, 12582917, 3145739 };
        return ((key.sortedVertexIndices[0] * PRIME_NUMBERS[0])
                ^ (key.sortedVertexIndices[1] * PRIME_NUMBERS[1]))
                ^ (key.sortedVertexIndices[2] * PRIME_NUMBERS[2]);
    }
};

typedef std::unordered_set<TriangleKey, TriangleKeyHasher> TriangleSet;

#endif //HEXVOLUMERENDERER_TRIANGLESET_HPP
