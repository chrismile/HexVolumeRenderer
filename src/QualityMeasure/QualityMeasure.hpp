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

#ifndef HEXVOLUMERENDERER_IMPORTANCECRITERIA_H
#define HEXVOLUMERENDERER_IMPORTANCECRITERIA_H

enum QualityMeasure {
    QUALITY_MEASURE_SCALED_JACOBIAN,
    QUALITY_MEASURE_EDGE_RATIO,
    QUALITY_MEASURE_DIAGONAL,
    QUALITY_MEASURE_DIMENSION,
    QUALITY_MEASURE_DISTORTION,
    QUALITY_MEASURE_JACOBIAN,
    QUALITY_MEASURE_MAX_EDGE_RATIO,
    QUALITY_MEASURE_MAX_ASPECT_FROBENIUS,
    QUALITY_MEASURE_MEAN_ASPECT_FROBENIUS,
    QUALITY_MEASURE_ODDY,
    QUALITY_MEASURE_RELATIVE_SIZE_SQUARED,
    QUALITY_MEASURE_SHAPE,
    QUALITY_MEASURE_SHAPE_AND_SIZE,
    QUALITY_MEASURE_SHEAR,
    QUALITY_MEASURE_SHEAR_AND_SIZE,
    QUALITY_MEASURE_SKEW,
    QUALITY_MEASURE_STRETCH,
    QUALITY_MEASURE_TAPER,
    QUALITY_MEASURE_VOLUME
};

const char *const QUALITY_MEASURE_NAMES[] = {
        "Scaled Jacobian",
        "Edge Ratio",
        "Diagonal",
        "Dimension",
        "Distortion",
        "Jacobian",
        "Max Edge Ratio",
        "Max Aspect Frobenius",
        "Mean Aspect Frobenius",
        "Oddy",
        "Relative Size Squared",
        "Shape",
        "Shape and Size",
        "Shear",
        "Shear and Size",
        "Skew",
        "Stretch",
        "Taper",
        "Volume"
};
const int NUM_IMPORTANCE_CRITERIA = ((int)(sizeof(QUALITY_MEASURE_NAMES) / sizeof(*QUALITY_MEASURE_NAMES)));

#endif //HEXVOLUMERENDERER_IMPORTANCECRITERIA_H
