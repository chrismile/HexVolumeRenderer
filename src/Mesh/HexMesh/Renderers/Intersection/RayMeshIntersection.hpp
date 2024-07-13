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
#ifndef HEXVOLUMERENDERER_RAYMESHINTERSECTION_HPP
#define HEXVOLUMERENDERER_RAYMESHINTERSECTION_HPP

#include <vector>
#include <glm/vec3.hpp>

namespace sgl {
class Camera;
typedef std::shared_ptr<Camera> CameraPtr;
}

/**
 * This interface provides ray-mesh intersection capabilities for picking points on a mesh.
 */
class RayMeshIntersection {
public:
    RayMeshIntersection(const sgl::CameraPtr& camera) : camera(camera) {}
    virtual ~RayMeshIntersection() {}

    /**
     * Sets the triangle mesh data.
     * @param vertices The vertex points.
     * @param triangleIndices The triangle indices.
     */
    virtual void setMeshTriangleData(
            const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& triangleIndices) = 0;

    /**
     * Picks a point on the mesh using screen coordinates (assuming origin at upper left corner of viewport).
     * @param x The x position on the screen (usually the mouse position).
     * @param y The y position on the screen (usually the mouse position).
     * @param w The viewport width.
     * @param h The viewport height.
     * @param firstHit The first hit point on the mesh (closest to the camera) is stored in this variable.
     * @param lastHit The last hit point on the mesh (furthest away from the camera) is stored in this variable.
     * @return True if a point on the mesh was hit.
     */
    bool pickPointScreen(int x, int y, int w, int h, glm::vec3& firstHit, glm::vec3& lastHit);

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
            glm::vec3& firstHit, glm::vec3& lastHit) = 0;

protected:
    sgl::CameraPtr camera;
};

namespace nanort {
template<class T>
class BVHAccel;
}

/**
 * This class provides ray-mesh intersection capabilities for picking points on a mesh.
 * The functionality of this specialization of RayMeshIntersection uses NanoRT.
 * It is the fallback when Embree (which is faster than NanoRT) is not available.
 */
class RayMeshIntersection_NanoRT : public RayMeshIntersection {
public:
    RayMeshIntersection_NanoRT(const sgl::CameraPtr& camera) : RayMeshIntersection(camera) {}
    ~RayMeshIntersection_NanoRT();

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

    nanort::BVHAccel<float>* accelerationStructure = nullptr;
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> triangleIndices;
    const float* vertexPointer;
    const uint32_t* indexPointer;
};

#endif //HEXVOLUMERENDERER_RAYMESHINTERSECTION_HPP
