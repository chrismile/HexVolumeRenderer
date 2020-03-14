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
#ifndef HEXVOLUMERENDERER_RAYMESHINTERSECTION_EMBREE_HPP
#define HEXVOLUMERENDERER_RAYMESHINTERSECTION_EMBREE_HPP

#include "RayMeshIntersection.hpp"

// Forward declarations.
struct RTCDeviceTy;
typedef struct RTCDeviceTy* RTCDevice;
class RTCSceneTy;
typedef struct RTCSceneTy* RTCScene;
class RTCGeometryTy;
typedef struct RTCGeometryTy* RTCGeometry;

/**
 * This class provides ray-mesh intersection capabilities for picking points on a mesh.
 * The functionality of this specialization of RayMeshIntersection uses Embree.
 */
class RayMeshIntersection_Embree : public RayMeshIntersection {
public:
    RayMeshIntersection_Embree(const sgl::CameraPtr& camera);
    ~RayMeshIntersection_Embree();

    /**
     * Sets the triangle mesh data.
     * @param vertices The vertex points.
     * @param triangleIndices The triangle indices.
     */
    virtual void setMeshTriangleData(
            const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& triangleIndices);

    /**
     * Picks a point on the mesh using a ray in world coordinates.
     * @param cameraPosition The origin of the ray (usually the camera position, but can be somewhere different, too).
     * @param rayDirection The direction of the ray (assumed to be normalized).
     * @param firstHit The first hit point on the mesh (closest to the camera) is stored in this variable.
     * @param lastHit The last hit point on the mesh (furthest away from the camera) is stored in this variable.
     * @return True if a point on the mesh was hit.
     */
    virtual bool pickPointWorld(
            const glm::vec3& cameraPosition, const glm::vec3& rayDirection,
            glm::vec3& firstHit, glm::vec3& lastHit);

private:
    void freeStorage();

    RTCDevice device;
    RTCScene scene;
    RTCGeometry mesh;
    unsigned int geomID = 0;
    bool loaded = false;
};

#endif //HEXVOLUMERENDERER_RAYMESHINTERSECTION_EMBREE_HPP
