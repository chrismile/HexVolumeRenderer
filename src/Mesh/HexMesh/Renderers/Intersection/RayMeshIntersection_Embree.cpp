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

#if defined(USE_EMBREE3)
#include <embree3/rtcore.h>
#elif defined(USE_EMBREE4)
#include <embree4/rtcore.h>
#endif

#include <glm/vec4.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>

#include "RayMeshIntersection_Embree.hpp"

RayMeshIntersection_Embree::RayMeshIntersection_Embree(const sgl::CameraPtr& camera) : RayMeshIntersection(camera) {
    device = rtcNewDevice(nullptr);
    scene = rtcNewScene(device);
    mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
}

RayMeshIntersection_Embree::~RayMeshIntersection_Embree() {
    freeStorage();

    rtcReleaseGeometry(mesh);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
}

void RayMeshIntersection_Embree::freeStorage() {
    if (loaded) {
        rtcDetachGeometry(scene, geomID);
        rtcCommitScene(scene);
        loaded = false;
    }
}

void RayMeshIntersection_Embree::setMeshTriangleData(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& triangleIndices) {
    freeStorage();
    if (vertices.empty() || triangleIndices.empty()) {
        return;
    }

    const size_t numVertices = vertices.size();
    const size_t numIndices = triangleIndices.size();
    const size_t numTriangles = numIndices / 3;

    auto* vertexPointer = (glm::vec4*)rtcSetNewGeometryBuffer(
            mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(glm::vec4), numVertices);
    auto* indexPointer = (uint32_t*)rtcSetNewGeometryBuffer(
            mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
            sizeof(uint32_t) * 3, numTriangles);
    for (size_t i = 0; i < numVertices; i++) {
        const glm::vec3& vertex = vertices.at(i);
        vertexPointer[i] = glm::vec4(vertex.x, vertex.y, vertex.z, 1.0f);
    }
    for (size_t i = 0; i < numIndices; i++) {
        indexPointer[i] = triangleIndices[i];
    }

    rtcCommitGeometry(mesh);
    unsigned int geomID = rtcAttachGeometry(scene, mesh);

    rtcCommitScene(scene);
    loaded = true;
}

bool RayMeshIntersection_Embree::pickPointWorld(
        const glm::vec3& cameraPosition, const glm::vec3& rayDirection, glm::vec3& firstHit, glm::vec3& lastHit) {
    if (!loaded) {
        return false;
    }

    glm::vec3 rayOrigin = cameraPosition;

    const float EPSILON_DEPTH = 1e-3f;
    const float INFINITY_DEPTH = 1e30f;

#ifdef USE_EMBREE3
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
#endif

    const size_t MAX_ITERATIONS = 65535u;
    size_t iterationNum;
    RTCRayHit query{};
#ifdef USE_EMBREE4
    query.ray.mask = -1;
#endif
    for (iterationNum = 0; iterationNum < MAX_ITERATIONS; iterationNum++) {
        query.ray.org_x = rayOrigin.x;
        query.ray.org_y = rayOrigin.y;
        query.ray.org_z = rayOrigin.z;
        query.ray.dir_x = rayDirection.x;
        query.ray.dir_y = rayDirection.y;
        query.ray.dir_z = rayDirection.z;
        query.ray.tnear = EPSILON_DEPTH;
        query.ray.tfar = INFINITY_DEPTH;
        query.ray.time = 0.0f;
        query.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        query.hit.primID = RTC_INVALID_GEOMETRY_ID;

#ifdef USE_EMBREE3
        rtcIntersect1(scene, &context, &query);
#elif defined(USE_EMBREE4)
        rtcIntersect1(scene, &query, nullptr);
#endif
        if (query.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
            break;
        }

        glm::vec3 intersectionPosition = rayOrigin + query.ray.tfar * rayDirection;
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
