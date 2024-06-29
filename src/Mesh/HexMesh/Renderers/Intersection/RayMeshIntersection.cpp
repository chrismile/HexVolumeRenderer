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

#include "nanort.h"
#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Scene/Camera.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "RayMeshIntersection.hpp"

bool RayMeshIntersection::pickPointScreen(
        int x, int y, int w, int h, glm::vec3& firstHit, glm::vec3& lastHit) {
    int viewportWidth = w;
    int viewportHeight = h;

    glm::mat4 inverseViewMatrix = glm::inverse(camera->getViewMatrix());
    float scale = std::tan(camera->getFOVy() * 0.5f);
    glm::vec2 rayDirCameraSpace;
    rayDirCameraSpace.x = (2.0f * (float(x) + 0.5f) / float(viewportWidth) - 1.0f) * camera->getAspectRatio() * scale;
    rayDirCameraSpace.y = (2.0f * (float(viewportHeight - y - 1) + 0.5f) / float(viewportHeight) - 1.0f) * scale;
    glm::vec4 rayDirectionVec4 = inverseViewMatrix * glm::vec4(rayDirCameraSpace, -1.0, 0.0);
    glm::vec3 rayDirection = normalize(glm::vec3(rayDirectionVec4.x, rayDirectionVec4.y, rayDirectionVec4.z));

    return pickPointWorld(camera->getPosition(), rayDirection, firstHit, lastHit);
}

RayMeshIntersection_NanoRT::~RayMeshIntersection_NanoRT() {
    freeStorage();
}

void RayMeshIntersection_NanoRT::freeStorage() {
    if (accelerationStructure != nullptr) {
        delete accelerationStructure;
        accelerationStructure = nullptr;
        vertexPointer = nullptr;
        indexPointer = nullptr;
    }
}

void RayMeshIntersection_NanoRT::setMeshTriangleData(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& triangleIndices) {
    freeStorage();
    if (vertices.size() <= 0 || triangleIndices.size() <= 0) {
        return;
    }
    this->vertices = vertices;
    this->triangleIndices = triangleIndices;

    vertexPointer = &this->vertices.at(0).x;
    indexPointer = this->triangleIndices.data();

    const size_t numFaces = triangleIndices.size() / 3;
    nanort::BVHBuildOptions<float> buildOptions;
    nanort::TriangleMesh<float> triangleMesh(vertexPointer, indexPointer, sizeof(glm::vec3));
    nanort::TriangleSAHPred<float> trianglePred(vertexPointer, indexPointer, sizeof(glm::vec3));
    accelerationStructure = new nanort::BVHAccel<float>();
    bool returnValue = accelerationStructure->Build(numFaces, triangleMesh, trianglePred, buildOptions);
    if (!returnValue) {
        sgl::Logfile::get()->writeError(
                "ERROR in RayMeshIntersection::setMeshTriangleData: Couldn't build the acceleration structure.");
        delete accelerationStructure;
        accelerationStructure = nullptr;
        return;
    }

    nanort::BVHBuildStatistics stats = accelerationStructure->GetStatistics();
    float bmin[3], bmax[3];
    accelerationStructure->BoundingBox(bmin, bmax);
    sgl::Logfile::get()->write(std::string() + "BVH statistics:");
    sgl::Logfile::get()->write(std::string() + "  # of leaf   nodes:" + sgl::toString(stats.num_leaf_nodes));
    sgl::Logfile::get()->write(std::string() + "  # of branch nodes:" + sgl::toString(stats.num_branch_nodes));
    sgl::Logfile::get()->write(std::string() + "  May tree depth   :" + sgl::toString(stats.max_tree_depth));
    sgl::Logfile::get()->write(std::string() + "  May tree depth   :" + sgl::toString(stats.max_tree_depth));
    sgl::Logfile::get()->write(
            std::string() + "  Bounding box min : (" + sgl::toString(bmin[0]) + ", "
            + sgl::toString(bmin[1]) + ", " + sgl::toString(bmin[2]) + ")");
    sgl::Logfile::get()->write(
            std::string() + "  Bounding box max : (" + sgl::toString(bmax[0]) + ", "
            + sgl::toString(bmax[1]) + ", " + sgl::toString(bmax[2]) + ")");
}

bool RayMeshIntersection_NanoRT::pickPointWorld(
        const glm::vec3& cameraPosition, const glm::vec3& rayDirection, glm::vec3& firstHit, glm::vec3& lastHit) {
    if (accelerationStructure == nullptr) {
        return false;
    }

    glm::vec3 rayOrigin = cameraPosition;

    const float EPSILON_DEPTH = 1e-3f;
    const float INFINITY_DEPTH = 1e30f;

    nanort::BVHTraceOptions traceOptions;
    nanort::Ray<float> ray;
    ray.min_t = EPSILON_DEPTH;
    ray.max_t = INFINITY_DEPTH;
    ray.org[0] = rayOrigin.x;
    ray.org[1] = rayOrigin.y;
    ray.org[2] = rayOrigin.z;
    ray.dir[0] = rayDirection.x;
    ray.dir[1] = rayDirection.y;
    ray.dir[2] = rayDirection.z;

    const size_t MAX_ITERATIONS = 65535u;
    size_t iterationNum;
    for (iterationNum = 0; iterationNum < MAX_ITERATIONS; iterationNum++) {
        nanort::TriangleIntersector<> triangleIntersector(vertexPointer, indexPointer, sizeof(glm::vec3));
        nanort::TriangleIntersection<> intersection;
        bool hit = accelerationStructure->Traverse(ray, triangleIntersector, &intersection, traceOptions);
        if (!hit) {
            break;
        }

        glm::vec3 intersectionPosition = rayOrigin + intersection.t * rayDirection;
        rayOrigin = intersectionPosition;

        if (iterationNum == 0) {
            firstHit = intersectionPosition;
        }
    }

    if (iterationNum == 0) {
        return false;
    }
    lastHit = rayOrigin;

    return true;
}
