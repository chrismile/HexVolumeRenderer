//
// Created by christoph on 23.01.20.
//

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
