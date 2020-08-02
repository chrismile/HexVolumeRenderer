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

#ifndef HEXVOLUMERENDERER_LINERENDERINGDEFINES_HPP
#define HEXVOLUMERENDERER_LINERENDERINGDEFINES_HPP

#include <glm/vec4.hpp>

// Minimum and maximum values in the UI.
const float MIN_LINE_WIDTH = 0.0001f;
const float MAX_LINE_WIDTH = 0.004f;

// The standard line size is computed depending on the cube root of the average cell volume times the factor below.
const float LINE_WIDTH_VOLUME_CBRT_FACTOR = 0.05f;
const float MIN_LINE_WIDTH_AUTO = 0.0003f;
const float MAX_LINE_WIDTH_AUTO = 0.004f;

// Ranges for ClearView.
const float MIN_FOCUS_RADIUS = 0.001f;
const float MAX_FOCUS_RADIUS = 0.4f;
const float FOCUS_RADIUS_VOLUME_CBRT_FACTOR = 2.0f;
const float MIN_FOCUS_RADIUS_AUTO = 0.001f;
const float MAX_FOCUS_RADIUS_AUTO = 0.4f;

// Focus sphere indicator size and color for ClearView renderers and LOD renderers.
const float MIN_FOCUS_SPHERE_RADIUS = 0.0005f;
const float MAX_FOCUS_SPHERE_RADIUS = 0.003f;
const float FOCUS_SPHERE_SIZE_FACTOR = 0.05f;
const glm::vec4 FOCUS_SPHERE_COLOR = glm::vec4(0.75f, 1.0f, 0.0f, 1.0f);//glm::vec4(0.2f, 0.0f, 0.0f, 1.0f);

#endif //HEXVOLUMERENDERER_LINERENDERINGDEFINES_HPP
