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

#include <unordered_set>
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#ifdef USE_CORK
#include <cork.h>
#endif

#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>

#include "Mesh/BaseComplex/global_types.h"
#include "Mesh/HexMesh/Renderers/Helpers/PointToLineDistance.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/SearchStructures/KDTree.hpp"
#include "Mesh/HexMesh/HexMesh.hpp"
#include "Tubes.hpp"

void addHemisphereToMesh_Union(
        const glm::vec3& center, glm::vec3 tangent, glm::vec3 normal,
        float tubeRadius, int numLongitudeSubdivisions, int numLatitudeSubdivisions, bool isStartHemisphere,
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) {
    glm::vec3 binormal = glm::cross(normal, tangent);
    tangent = tubeRadius * tangent;
    normal = tubeRadius * normal;
    binormal = tubeRadius * binormal;

    float theta; // azimuth;
    float phi; // zenith;

    size_t vertexIndexOffset = vertexPositions.size() - numLongitudeSubdivisions;
    for (int lat = 1; lat <= numLatitudeSubdivisions; lat++) {
        phi = sgl::HALF_PI * (1.0f - float(lat) / numLatitudeSubdivisions);
        for (int lon = 0; lon < numLongitudeSubdivisions; lon++) {
            theta = -sgl::TWO_PI * float(lon) / numLongitudeSubdivisions;

            glm::vec3 pt(
                    std::cos(theta) * std::sin(phi),
                    std::sin(theta) * std::sin(phi),
                    std::cos(phi)
            );

            glm::vec3 trafoPt(
                    pt.x * normal.x + pt.y * binormal.x + pt.z * tangent.x + center.x,
                    pt.x * normal.y + pt.y * binormal.y + pt.z * tangent.y + center.y,
                    pt.x * normal.z + pt.y * binormal.z + pt.z * tangent.z + center.z
            );
            glm::vec3 normalMesh = glm::normalize(glm::vec3(
                    pt.x * normal.x + pt.y * binormal.x + pt.z * tangent.x,
                    pt.x * normal.y + pt.y * binormal.y + pt.z * tangent.y,
                    pt.x * normal.z + pt.y * binormal.z + pt.z * tangent.z
            ));

            vertexPositions.push_back(trafoPt);

            if (lat == numLatitudeSubdivisions) {
                break;
            }
        }
    }
    for (int lat = 0; lat < numLatitudeSubdivisions; lat++) {
        for (int lon = 0; lon < numLongitudeSubdivisions; lon++) {
            if (isStartHemisphere && lat == 0) {
                triangleIndices.push_back(
                                          (2*numLongitudeSubdivisions-lon)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(
                                          (2*numLongitudeSubdivisions-lon-1)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon)%numLongitudeSubdivisions
                                          + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(
                                          (2*numLongitudeSubdivisions-lon-1)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon+1)%numLongitudeSubdivisions
                                          + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon)%numLongitudeSubdivisions
                                          + (lat+1)*numLongitudeSubdivisions);
            } else if (lat < numLatitudeSubdivisions-1) {
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon+1)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon)%numLongitudeSubdivisions
                                          + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon+1)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon+1)%numLongitudeSubdivisions
                                          + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon)%numLongitudeSubdivisions
                                          + (lat+1)*numLongitudeSubdivisions);
            } else {
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + (lon+1)%numLongitudeSubdivisions
                                          + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(vertexIndexOffset
                                          + 0
                                          + (lat+1)*numLongitudeSubdivisions);
            }
        }
    }
}

void createCappedTubeCylinder(
        const glm::vec3& point0, const glm::vec3& point1,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions) {
    if (numCircleSubdivisions != int(globalCircleVertexPositions.size()) || tubeRadius != globalTubeRadius) {
        initGlobalCircleVertexPositions(numCircleSubdivisions, tubeRadius);
    }

    glm::vec3 normal0, normal1;

    glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
    std::vector<glm::vec3> lineNormals;
    glm::vec3 tangent = point1 - point0;
    float lineSegmentLength = glm::length(tangent);

    if (lineSegmentLength < 0.0001f) {
        // In case the two vertices are almost identical, just skip this path line segment
        return;
    }
    tangent = glm::normalize(tangent);

    insertOrientedCirclePoints(
            point0, tangent, lastLineNormal, vertexPositions);
    normal0 = glm::vec3(lastLineNormal.x, lastLineNormal.y, lastLineNormal.z);
    insertOrientedCirclePoints(
            point1, tangent, lastLineNormal, vertexPositions);
    normal1 = glm::vec3(lastLineNormal.x, lastLineNormal.y, lastLineNormal.z);

    for (int j = 0; j < numCircleSubdivisions; j++) {
        // Build two CCW triangles (one quad) for each side
        // Triangle 1
        triangleIndices.push_back(0*numCircleSubdivisions+j);
        triangleIndices.push_back(0*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
        triangleIndices.push_back(1*numCircleSubdivisions+(j+1)%numCircleSubdivisions);

        // Triangle 2
        triangleIndices.push_back(0*numCircleSubdivisions+j);
        triangleIndices.push_back(1*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
        triangleIndices.push_back(1*numCircleSubdivisions+j);
    }

    /*
     * Close the tube with two hemisphere caps at the ends.
     */
    int numLongitudeSubdivisions = numCircleSubdivisions; // azimuth
    int numLatitudeSubdivisions = std::ceil(numCircleSubdivisions/2); // zenith

    // Hemisphere at the start
    glm::vec3 center0 = point0;
    glm::vec3 tangent0 = point0 - point1;
    tangent0 = glm::normalize(tangent0);

    // Hemisphere at the end
    glm::vec3 center1 = point1;
    glm::vec3 tangent1 = point1 - point0;
    tangent1 = glm::normalize(tangent1);


    addHemisphereToMesh_Union(
            center1, tangent1, normal1, tubeRadius,
            numLongitudeSubdivisions, numLatitudeSubdivisions, false,
            triangleIndices, vertexPositions);
    addHemisphereToMesh_Union(
            center0, tangent0, normal0, tubeRadius,
            numLongitudeSubdivisions, numLatitudeSubdivisions, true,
            triangleIndices, vertexPositions);
}


void createCappedTriangleTubesUnionRenderDataCPU(
        HexMeshPtr hexMesh,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<glm::vec4>& vertexColors,
        bool useGlowColors) {
    /*
     * TODO:
     * - Build computation tree.
     * - Compute union of all tubes.
     *
     * When we have the merged mesh:
     * - For all vertices: Get closest line segment, and closest of two line segment points.
     * - Simplification: We know that the tube radius is the maximum search radius for any point.
     * => Rather:
     * - Get closest grid vertex.
     * - For all edges incident with grid vertex: See which one has the lowest point to line segment distance.
     * - Assign normal and color based on direction from grid vertex to triangle mesh vertex.
     * - Assign tangent based on line segment direction.
     *
     * In the end:
     * - Parallelize.
     */
    Mesh& mesh = hexMesh->getBaseComplexMesh();
    Singularity& si = hexMesh->getBaseComplexMeshSingularity();
    //int maxNumThreads = omp_get_max_threads();

    if (mesh.Es.size() == 0) {
        return;
    }

    // Reserve data outside of loop to make sure that memory reservations are kept to a minimum.
    std::vector<uint32_t>& unionMeshTriangleIndices = triangleIndices;
    std::vector<glm::vec3>& unionMeshVertexPositions = vertexPositions;
    std::vector<uint32_t> currentTubeTriangleIndices;
    std::vector<glm::vec3> currentTubeVertexPositions;
    std::vector<glm::vec3> edgeVertexPositions;
    edgeVertexPositions.reserve(2);

    // Successively compute the union with newly added edges sharing a hex mesh vertex with the previous union elements.
    std::queue<uint32_t> openEdgeQueue;
    std::unordered_set<uint32_t> closedEdgeSet;
    openEdgeQueue.push(mesh.Es.front().id);
    closedEdgeSet.insert(mesh.Es.front().id);
    while (!openEdgeQueue.empty()) {
        uint32_t& e_id = openEdgeQueue.front();
        Hybrid_E& e = mesh.Es.at(e_id);
        openEdgeQueue.pop();

        // Add all neighbors and get the vertex positions of the two points connected by the edge.
        for (uint32_t v_id : e.vs) {
            Hybrid_V& v = mesh.Vs.at(v_id);
            edgeVertexPositions.push_back(
                    glm::vec3(mesh.V(0, v_id), mesh.V(1, v_id), mesh.V(2, v_id)));
            for (uint32_t neighbor_e_id : v.neighbor_es) {
                if (closedEdgeSet.find(neighbor_e_id) == closedEdgeSet.end()) {
                    openEdgeQueue.push(neighbor_e_id);
                    closedEdgeSet.insert(neighbor_e_id);
                }
            }
        }

        // Get the mesh for the current edge.
        createCappedTubeCylinder(
                edgeVertexPositions.at(0), edgeVertexPositions.at(1), tubeRadius,
                numCircleSubdivisions, currentTubeTriangleIndices, currentTubeVertexPositions);

        // In first iteration: Just set the union mesh to the first tube.
        if (unionMeshTriangleIndices.size() == 0) {
            unionMeshTriangleIndices = currentTubeTriangleIndices;
            unionMeshVertexPositions = currentTubeVertexPositions;
        }

#ifdef USE_CORK
        CorkTriMesh inputUnionTriMesh, inputTubeMesh, outputUnionTriMesh;
        inputUnionTriMesh.n_triangles = unionMeshTriangleIndices.size() / 3;
        inputUnionTriMesh.n_vertices = unionMeshVertexPositions.size();
        inputUnionTriMesh.triangles = unionMeshTriangleIndices.data();
        inputUnionTriMesh.vertices = &unionMeshVertexPositions.front().x;
        inputTubeMesh.n_triangles = currentTubeTriangleIndices.size() / 3;
        inputTubeMesh.n_vertices = currentTubeVertexPositions.size();
        inputTubeMesh.triangles = currentTubeTriangleIndices.data();
        inputTubeMesh.vertices = &currentTubeVertexPositions.front().x;
        computeUnion(inputUnionTriMesh, inputTubeMesh, &outputUnionTriMesh);
        unionMeshTriangleIndices.resize(outputUnionTriMesh.n_triangles * 3);
        unionMeshVertexPositions.resize(outputUnionTriMesh.n_vertices);
        for (size_t i = 0; i < unionMeshTriangleIndices.size(); i++) {
            unionMeshTriangleIndices.at(i) = outputUnionTriMesh.triangles[i];
        }
        for (size_t i = 0; i < unionMeshVertexPositions.size(); i++) {
            unionMeshVertexPositions.at(i) = glm::vec3(
                    outputUnionTriMesh.vertices[i*3],
                    outputUnionTriMesh.vertices[i*3+1],
                    outputUnionTriMesh.vertices[i*3+2]);
        }
        freeCorkTriMesh(&outputUnionTriMesh);
#else
        // CSG not supported - just append the triangle data.
        sgl::Logfile::get()->write("createCappedTriangleTubesUnionRenderDataCPU: CSG not supported.");
        uint32_t indexOffset = unionMeshVertexPositions.size();
        for (uint32_t idx : currentTubeTriangleIndices) {
            unionMeshTriangleIndices.push_back(indexOffset + idx);
        }
        unionMeshVertexPositions.insert(
                unionMeshVertexPositions.end(), currentTubeVertexPositions.begin(), currentTubeVertexPositions.end());
#endif

        // Clean-up.
        edgeVertexPositions.clear();
        currentTubeTriangleIndices.clear();
        currentTubeVertexPositions.clear();
    }

    // Build a search structure on the hexahedral mesh vertices.
    KDTree kdTree;
    std::vector<IndexedPoint> indexedPoints;
    std::vector<IndexedPoint*> indexedPointsPointers;
    indexedPoints.resize(mesh.Vs.size());
    indexedPointsPointers.reserve(mesh.Vs.size());
    for (size_t i = 0; i < mesh.Vs.size(); i++) {
        Hybrid_V& v = mesh.Vs.at(i);
        IndexedPoint* point = &indexedPoints.at(i);
        point->index = v.id;
        point->position = glm::vec3(mesh.V(0, v.id), mesh.V(1, v.id), mesh.V(2, v.id));
        indexedPointsPointers.push_back(point);
    }
    kdTree.build(indexedPointsPointers);

    // Determine which edges are regular and which singular.
    const glm::vec4 regularColor = useGlowColors ? HexMesh::glowColorRegular : HexMesh::outlineColorRegular;
    const glm::vec4 singularColor = useGlowColors ? HexMesh::glowColorSingular : HexMesh::outlineColorSingular;
    std::unordered_set<uint32_t> singularEdgeIds;
    for (Singular_E& se : si.SEs) {
        for (uint32_t e_id : se.es_link) {
            singularEdgeIds.insert(e_id);
        }
    }

    // Now, for all triangle mesh vertices, find the closest point and take the attributes of the closest edge.
    const float EPSILON = 1e-5f;
    vertexNormals.reserve(unionMeshVertexPositions.size());
    vertexTangents.reserve(unionMeshVertexPositions.size());
    vertexColors.reserve(unionMeshVertexPositions.size());
    for (size_t i = 0; i < unionMeshVertexPositions.size(); i++) {
        // Find the closest vertex in the hexahedral mesh for the current triangle mesh vertex.
        glm::vec3& triangleMeshVertexPosition = unionMeshVertexPositions.at(i);
        std::vector<IndexedPoint*> pointsInRadius =
                kdTree.findPointsInSphere(triangleMeshVertexPosition, tubeRadius + EPSILON);
        assert(!pointsInRadius.empty());
        std::sort(pointsInRadius.begin(), pointsInRadius.end(),
                [triangleMeshVertexPosition](IndexedPoint* a, IndexedPoint* b) {
            return glm::length2(triangleMeshVertexPosition - a->position)
                    > glm::length2(triangleMeshVertexPosition - b->position);
        });
        IndexedPoint* closestPoint = pointsInRadius.front();
        Hybrid_V& v = mesh.Vs.at(closestPoint->index);

        // Now, find out which hexahedral mesh edge is closest to the triangle mesh vertex.
        float minimumEdgeDistance = -FLT_MAX;
        int minimumEdgeDistanceIndex = 0;
        int currentIndex = 0;
        for (uint32_t e_id : v.neighbor_es) {
            Hybrid_E& e = mesh.Es.at(e_id);
            for (uint32_t v_id : e.vs) {
                edgeVertexPositions.push_back(
                        glm::vec3(mesh.V(0, v_id), mesh.V(1, v_id), mesh.V(2, v_id)));
            }
            float edgeDistance = distanceToLineSegment(
                    unionMeshVertexPositions.at(i), edgeVertexPositions.at(0), edgeVertexPositions.at(1));
            if (edgeDistance < minimumEdgeDistance) {
                minimumEdgeDistance = edgeDistance;
                minimumEdgeDistanceIndex = currentIndex;
            }
            currentIndex++;
            edgeVertexPositions.clear();
        }

        // Get the positions of the vertices connected by the edge.
        Hybrid_E& minimum_dist_e = mesh.Es.at(v.neighbor_es.at(minimumEdgeDistanceIndex));
        for (uint32_t v_id : minimum_dist_e.vs) {
            edgeVertexPositions.push_back(
                    glm::vec3(mesh.V(0, v_id), mesh.V(1, v_id), mesh.V(2, v_id)));
        }

        // Get the attributes of the edge and associate them with the vertex.
        vertexNormals.push_back(glm::normalize(unionMeshVertexPositions.at(i) - closestPoint->position));
        vertexTangents.push_back(glm::normalize(edgeVertexPositions.at(1) - edgeVertexPositions.at(0)));
        bool isSingular = singularEdgeIds.find(minimum_dist_e.id) != singularEdgeIds.end();
        if (isSingular) {
            vertexColors.push_back(singularColor);
        } else {
            vertexColors.push_back(regularColor);
        }
        edgeVertexPositions.clear();
    }
}
