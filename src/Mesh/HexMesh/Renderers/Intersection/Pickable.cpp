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

#include <glm/glm.hpp>

#include <Math/Math.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Pickable.hpp"
#include "Mesh/HexMesh/Renderers/Helpers/LineRenderingDefines.hpp"

glm::vec3 Pickable::focusPoint = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec4 Pickable::focusPointColor = FOCUS_SPHERE_COLOR;

void Pickable::updatePickable(float dt, bool& reRender, SceneData& sceneData) {
#ifdef SGL_INPUT_API_V2
    if (sgl::Keyboard->getModifier(ImGuiKey_ModCtrl)) {
#else
    if (sgl::Keyboard->getModifier() & KMOD_CTRL) {
#endif
        if (sgl::Mouse->buttonPressed(1) || (sgl::Mouse->isButtonDown(1) && sgl::Mouse->mouseMoved())) {
            int mouseX = sgl::Mouse->getX();
            int mouseY = sgl::Mouse->getY();
            if (sgl::ImGuiWrapper::get()->getUseDockSpaceMode()) {
                mouseX -= sceneData.pickingOffsetX;
                mouseY -= sceneData.pickingOffsetY;
            }
            bool rayHasHitMesh = sceneData.rayMeshIntersection.pickPointScreen(
                    mouseX, mouseY, (*sceneData.sceneTexture)->getW(), (*sceneData.sceneTexture)->getH(),
                    firstHit, lastHit);
            if (rayHasHitMesh) {
                focusPoint = firstHit;
                hitLookingDirection = glm::normalize(firstHit - sceneData.camera->getPosition());
                hasHitInformation = true;
                reRender = true;
            }
        }

        if (sgl::Mouse->getScrollWheel() > 0.1f || sgl::Mouse->getScrollWheel() < -0.1f) {
            if (!hasHitInformation) {
                glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
                glm::vec3 lookingDirection = glm::vec3(-inverseViewMatrix[2].x, -inverseViewMatrix[2].y, -inverseViewMatrix[2].z);

                float moveAmount = sgl::Mouse->getScrollWheel() * dt * 0.5f;
                glm::vec3 moveDirection = focusPoint - sceneData.camera->getPosition();
                moveDirection *= float(sgl::sign(glm::dot(lookingDirection, moveDirection)));
                if (glm::length(moveDirection) < 1e-4) {
                    moveDirection = lookingDirection;
                }
                moveDirection = glm::normalize(moveDirection);
                focusPoint = focusPoint + moveAmount * moveDirection;
            } else {
                float moveAmount = sgl::Mouse->getScrollWheel() * dt;
                glm::vec3 newFocusPoint = focusPoint + moveAmount * hitLookingDirection;
                float t = glm::dot(newFocusPoint - firstHit, hitLookingDirection);
                t = glm::clamp(t, 0.0f, glm::length(lastHit - firstHit));
                focusPoint = firstHit + t * hitLookingDirection;
            }
            reRender = true;
        }
    }
}
