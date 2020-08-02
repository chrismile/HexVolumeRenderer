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

#ifndef HEXVOLUMERENDERER_EDGEKEY_HPP
#define HEXVOLUMERENDERER_EDGEKEY_HPP

#include <algorithm>
#include <unordered_set>

struct EdgeKey {
    EdgeKey(uint32_t id0, uint32_t id1) {
        edgeIds[0] = id0;
        edgeIds[1] = id1;
        std::sort(std::begin(edgeIds), std::end(edgeIds));
    }

    bool operator==(const EdgeKey& rhs) const {
        for (int i = 0; i < 2; i++) {
            if (edgeIds[i] != rhs.edgeIds[i]) {
                return false;
            }
        }
        return true;
    }

    uint32_t edgeIds[2];
};

// Hash Function: H(i,j) = (i0*p_1 xor i1*jp_2) mod n
struct EdgeKeyHasher {
    std::size_t operator()(const EdgeKey& key) const {
        const size_t PRIME_NUMBERS[] = { 12582917, 3145739 };
        return (key.edgeIds[0] * PRIME_NUMBERS[0]) ^ (key.edgeIds[1] * PRIME_NUMBERS[1]);
    }
};

typedef std::unordered_map<EdgeKey, uint32_t, EdgeKeyHasher> EdgeMap;

#endif //HEXVOLUMERENDERER_EDGEKEY_HPP
