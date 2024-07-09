/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020-2021, Christoph Neuhauser
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

#include <map>
#include <vector>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/CheckpointWindow.hpp>
#include <ImGui/imgui.h>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include <Python.h>

#include "ReplayWidget.hpp"

static ReplayState currentReplayStateGlobal;
static std::vector<ReplayState> replayStatesGlobal;
static bool useCameraFlightGlobal = false;
static bool isFirstState = true;

static PyObject* py_set_duration(PyObject* self, PyObject* args) {
    float duration = 0.0f;
    if (!PyArg_ParseTuple(args, "f", &duration)) {
        return NULL;
    }

    if (!isFirstState) {
        replayStatesGlobal.push_back(currentReplayStateGlobal);
        currentReplayStateGlobal = ReplayState();
    } else {
        isFirstState = false;
    }

    currentReplayStateGlobal.duration = duration;
    currentReplayStateGlobal.startTime = replayStatesGlobal.empty() ? 0.0f : replayStatesGlobal.back().stopTime;
    currentReplayStateGlobal.stopTime = currentReplayStateGlobal.startTime + duration;
    Py_RETURN_NONE;
}

static PyObject* py_set_mesh(PyObject* self, PyObject* args) {
    const char* paperName = nullptr;
    const char* meshName = nullptr;
    const char* meshFileEnding = nullptr;
    if (!PyArg_ParseTuple(args, "sss", &paperName, &meshName, &meshFileEnding)) {
        return NULL;
    }
    currentReplayStateGlobal.meshDescriptor = MeshDescriptor(
            paperName, meshName, meshFileEnding);
    Py_RETURN_NONE;
}

static PyObject* py_set_transfer_function(PyObject* self, PyObject* args) {
    const char* transferFunctionName = nullptr;
    if (!PyArg_ParseTuple(args, "s", &transferFunctionName)) {
        return NULL;
    }
    currentReplayStateGlobal.transferFunctionName = transferFunctionName;
    Py_RETURN_NONE;
}

static PyObject* py_set_renderer(PyObject* self, PyObject* args) {
    const char* rendererName = nullptr;
    if (!PyArg_ParseTuple(args, "s", &rendererName)) {
        return NULL;
    }
    currentReplayStateGlobal.rendererName = rendererName;
    Py_RETURN_NONE;
}

static PyObject* py_set_rendering_algorithm_settings(PyObject* self, PyObject* args) {
    PyObject* dict = nullptr;
    if (!PyArg_ParseTuple(args, "O", &dict)) {
        return NULL;
    }

    if (!PyDict_Check(dict)) {
        return NULL;
    }

    PyObject* key, *value;
    Py_ssize_t pos = 0;
    while (PyDict_Next(dict, &pos, &key, &value)) {
        std::string keyString = (const char*)PyUnicode_AsUTF8(key);
        ReplayDataType replayDataType;
        if (PyLong_CheckExact(value)) {
            replayDataType = REPLAY_DATA_TYPE_INTEGER;
        } else if (PyFloat_CheckExact(value)) {
            replayDataType = REPLAY_DATA_TYPE_REAL;
        } else if (PyByteArray_CheckExact(value)) {
            replayDataType = REPLAY_DATA_TYPE_STRING;
        } else if (PyUnicode_CheckExact(value)) {
            replayDataType = REPLAY_DATA_TYPE_STRING;
        } else if (PyBool_Check(value)) {
            replayDataType = REPLAY_DATA_TYPE_BOOLEAN;
        } else if (PyTuple_CheckExact(value)) {
            Py_ssize_t tupleSize = PyTuple_Size(value);
            if (tupleSize == 2) {
                replayDataType = REPLAY_DATA_TYPE_VEC2;
            } else if (tupleSize == 3) {
                replayDataType = REPLAY_DATA_TYPE_VEC3;
            } else if (tupleSize == 4) {
                replayDataType = REPLAY_DATA_TYPE_VEC4;
            } else {
                sgl::Logfile::get()->writeInfo("INFO in py_set_rendering_algorithm_settings: Invalid tuple size.");
                Py_RETURN_NONE;
            }
        } else {
            sgl::Logfile::get()->writeInfo("INFO in py_set_rendering_algorithm_settings: Unknown data type.");
            replayDataType = REPLAY_DATA_TYPE_STRING;
        }

        if (replayDataType == REPLAY_DATA_TYPE_BOOLEAN) {
            currentReplayStateGlobal.rendererSettings.insert(keyString, bool(PyObject_IsTrue(value)));
        } else if (replayDataType == REPLAY_DATA_TYPE_VEC2 || replayDataType == REPLAY_DATA_TYPE_VEC3
                || replayDataType == REPLAY_DATA_TYPE_VEC4) {
            Py_ssize_t tupleSize = PyTuple_Size(value);
            PyObject* tupleValues[4] = { nullptr, nullptr, nullptr, nullptr };
            float values[4];
            for (Py_ssize_t i = 0; i < tupleSize; i++) {
                tupleValues[i] = PyTuple_GetItem(value, i);
                if (tupleValues[i] == nullptr) {
                    sgl::Logfile::get()->writeError(
                            "ERROR in py_set_rendering_algorithm_settings: Invalid tuple.");
                    Py_RETURN_NONE;
                }
                bool isFloat = PyFloat_Check(tupleValues[i]);
                bool isLong = PyLong_Check(tupleValues[i]);
                if (isFloat) {
                    values[i] = PyFloat_AsDouble(tupleValues[i]);
                } else if (isLong) {
                    values[i] = PyLong_AsDouble(tupleValues[i]);
                } else {
                    sgl::Logfile::get()->writeError(
                            "ERROR in py_set_rendering_algorithm_settings: Tuple must contain float or long values.");
                    Py_RETURN_NONE;
                }
            }

            if (tupleSize == 2) {
                currentReplayStateGlobal.rendererSettings.insert(
                        keyString,
                        vec2ToString(glm::vec2(values[0], values[1])),
                        replayDataType);
            } else if (tupleSize == 3) {
                currentReplayStateGlobal.rendererSettings.insert(
                        keyString,
                        vec3ToString(glm::vec3(values[0], values[1], values[2])),
                        replayDataType);
            } else if (tupleSize == 4) {
                currentReplayStateGlobal.rendererSettings.insert(
                        keyString,
                        vec4ToString(glm::vec4(values[0], values[1], values[2], values[3])),
                        replayDataType);
            }
        } else {
            PyObject* valueRepresentation = PyObject_Repr(value);
            std::string valueString = PyUnicode_AsUTF8(valueRepresentation);
            currentReplayStateGlobal.rendererSettings.insert(keyString, valueString, replayDataType);
        }
    }

    Py_RETURN_NONE;
}

static PyObject* py_set_camera_position(PyObject* self, PyObject* args) {
    glm::vec3 cameraPosition = glm::vec3(0.0f);
    Py_ssize_t tupleSize = PyTuple_Size(args);
    if (tupleSize == 1) {
        PyObject* positionTuple = nullptr;
        if (!PyArg_ParseTuple(args, "O", &positionTuple)) {
            return NULL;
        }
        if (!PyArg_ParseTuple(positionTuple, "fff", &cameraPosition.x, &cameraPosition.y, &cameraPosition.z)) {
            return NULL;
        }
    } else if (tupleSize == 3) {
        if (!PyArg_ParseTuple(args, "fff", &cameraPosition.x, &cameraPosition.y, &cameraPosition.z)) {
            return NULL;
        }
    } else {
        sgl::Logfile::get()->writeError(
                "ERROR in py_set_camera_position: Tuple must contain three float values or one tuple.");
        return NULL;
    }

    currentReplayStateGlobal.cameraPositionSet = true;
    currentReplayStateGlobal.cameraPosition = cameraPosition;
    Py_RETURN_NONE;
}

static PyObject* py_set_camera_yaw_pitch_rad(PyObject* self, PyObject* args) {
    float yaw = 0.0f, pitch = 0.0f;
    Py_ssize_t tupleSize = PyTuple_Size(args);
    if (tupleSize == 1) {
        PyObject* yawPitchTuple = nullptr;
        if (!PyArg_ParseTuple(args, "O", &yawPitchTuple)) {
            return NULL;
        }
        if (!PyArg_ParseTuple(yawPitchTuple, "ff", &yaw, &pitch)) {
            return NULL;
        }
    } else if (tupleSize == 2) {
        if (!PyArg_ParseTuple(args, "ff", &yaw, &pitch)) {
            return NULL;
        }
    } else {
        sgl::Logfile::get()->writeError(
                "ERROR in py_set_camera_yaw_pitch_rad: Tuple must contain two float values or one tuple.");
        return NULL;
    }

    // We don't want a flip-over at the poles of the unit sphere.
    /*const float EPSILON = 0.001f;
    pitch = sgl::clamp(pitch, -sgl::HALF_PI + EPSILON, sgl::HALF_PI - EPSILON);

    glm::vec3 cameraFront, cameraRight, cameraUp;

    glm::vec3 globalUp = glm::vec3(0.0f, 1.0f, 0.0f);
    cameraFront.x = cos(yaw) * cos(pitch);
    cameraFront.y = sin(pitch);
    cameraFront.z = sin(yaw) * cos(pitch);

    cameraFront = glm::normalize(cameraFront);
    cameraRight = glm::normalize(glm::cross(cameraFront, globalUp));
    cameraUp    = glm::normalize(glm::cross(cameraRight, cameraFront));*/

    currentReplayStateGlobal.cameraOrientationSet = true;
    //currentReplayStateGlobal.cameraOrientation = glm::quat_cast(glm::lookAt(
    //        glm::vec3(0.0f), cameraFront, cameraUp));
    currentReplayStateGlobal.cameraOrientation =
               glm::angleAxis(-pitch, glm::vec3(1, 0, 0))
               * glm::angleAxis(yaw + sgl::PI / 2.0f, glm::vec3(0, 1, 0));
    Py_RETURN_NONE;
}

static PyObject* py_set_camera_checkpoint(PyObject* self, PyObject* args) {
    const char* cameraStateName = nullptr;
    if (!PyArg_ParseTuple(args, "s", &cameraStateName)) {
        return NULL;
    }
    currentReplayStateGlobal.cameraCheckpointName = cameraStateName;
    Py_RETURN_NONE;
}

static PyObject* py_set_use_camera_flight(PyObject* self, PyObject* args) {
    int useCameraFlight = false;
    if (!PyArg_ParseTuple(args, "p", &useCameraFlight)) {
        return NULL;
    }

    useCameraFlightGlobal = useCameraFlight;
    Py_RETURN_NONE;
}

static PyMethodDef REPLAY_METHODS[] = {
        {"set_duration", py_set_duration, METH_VARARGS,
                "Sets the duration of the state. Needs to be called as the first function in each state!"},
        {"set_mesh", py_set_mesh, METH_VARARGS,
                "Sets the name of mesh to load. Expects paper name, mesh name, mesh file extension."},
        {"set_transfer_function", py_set_transfer_function, METH_VARARGS,
                "Sets the name of transfer function to load."},
        {"set_renderer", py_set_renderer, METH_VARARGS,
                "Sets the name of the renderer to use."},
        {"set_rendering_algorithm_settings", py_set_rendering_algorithm_settings, METH_VARARGS,
                "Sets a dict of rendering algorithm settings."},
        {"set_camera_position", py_set_camera_position, METH_VARARGS,
                "Sets the camera world position."},
        {"set_camera_yaw_pitch_rad", py_set_camera_yaw_pitch_rad, METH_VARARGS,
                "Sets the camera orientation using a yaw and pitch value (in radians)."},
        {"set_camera_checkpoint", py_set_camera_checkpoint, METH_VARARGS,
                "Sets the camera checkpoint corresponding to the passed string."},
        {"set_use_camera_flight", py_set_use_camera_flight, METH_VARARGS,
                "Whether to use the pre-defined camera flight for camera positions and orientations."},
        {NULL, NULL, 0, NULL}
};

static PyModuleDef EmbModule = {
        PyModuleDef_HEAD_INIT, "g", NULL, -1, REPLAY_METHODS,
        NULL, NULL, NULL, NULL
};

ReplayWidget::ReplayWidget(
        SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow,
        sgl::CheckpointWindow& checkpointWindow)
        : sceneData(sceneData), transferFunctionWindow(transferFunctionWindow), checkpointWindow(checkpointWindow) {
    scriptDirectory = sgl::AppSettings::get()->getDataDirectory() + "ReplayScripts/";
    directoryContentWatch.setPath(scriptDirectory, true);
    directoryContentWatch.initialize();
    updateAvailableReplayScripts();

    PyObject* sys = PyImport_ImportModule("sys");
    PyObject* sysPath = PyObject_GetAttrString(sys, "path");
    PyObject* folderPath = PyUnicode_FromString(scriptDirectory.c_str());
    PyList_Append(sysPath, folderPath);
    //PyRun_SimpleString("import sys\nsys.path.append(\"" + scriptDirectory + "\")\n");
    Py_DECREF(sys);
    Py_DECREF(sysPath);
    Py_DECREF(folderPath);

    const char* moduleName = "g";
    PyImport_AddModule("g");
    PyObject* moduleGlobal = PyModule_Create(&EmbModule);

    PyObject* sysModules = PyImport_GetModuleDict();
    PyDict_SetItemString(sysModules, moduleName, moduleGlobal);
    Py_DECREF(moduleGlobal);
}

ReplayWidget::~ReplayWidget() {
    ;
}

bool ReplayWidget::runScript(const std::string& filename) {
    replayStates.clear();
    currentStateIndex = 0;
    replayStatesGlobal.clear();
    useCameraFlightGlobal = false;
    isFirstState = true;
    currentReplayStateGlobal = ReplayState();

    std::string scriptModuleName = sgl::FileUtils::get()->filenameWithoutExtension(filename);
    PyObject* pythonFileName = PyUnicode_DecodeFSDefault(scriptModuleName.c_str());
    PyObject* moduleOrig = PyImport_Import(pythonFileName);
    Py_DECREF(pythonFileName);
    PyObject* module = PyImport_ReloadModule(moduleOrig);
    Py_DECREF(moduleOrig);

    if (!module) {
        if (PyErr_Occurred()) {
            PyErr_Print();
        }
        sgl::Logfile::get()->writeError(
                std::string() + "ERROR in ReplayWidget::runScript: Couldn't execute script \""
                + scriptModuleName + "\"!");
        return false;
    }

    PyObject* replayFunc = PyObject_GetAttrString(module, "replay");
    if (replayFunc && PyCallable_Check(replayFunc)) {
        PyObject* pValue = PyObject_CallObject(replayFunc, nullptr);
        if (pValue != NULL) {
            Py_DECREF(pValue);
        } else {
            Py_DECREF(replayFunc);
            Py_DECREF(module);
            PyErr_Print();
            sgl::Logfile::get()->writeError(
                    std::string() + "ERROR in ReplayWidget::runScript: Failed to call 'replay' in script \""
                    + scriptModuleName + "\"!");
            return false;
        }
    } else {
        if (PyErr_Occurred()) {
            PyErr_Print();
        }
        sgl::Logfile::get()->writeError(
                std::string() + "ERROR in ReplayWidget::runScript: Couldn't find function 'replay' in script \""
                + scriptModuleName + "\"!");
    }

    replayStatesGlobal.push_back(currentReplayStateGlobal);
    replayStates = replayStatesGlobal;
    if (!replayStates.empty()) {
        firstTimeState = true;
    }
    replayStatesGlobal.clear();
    currentReplayStateGlobal = ReplayState();
    useCameraFlight = useCameraFlightGlobal;

    cameraPositionLast = sceneData.camera->getPosition();
    cameraOrientationLast =
            glm::angleAxis(-sceneData.camera->getPitch(), glm::vec3(1, 0, 0))
            * glm::angleAxis(sceneData.camera->getYaw() + sgl::PI / 2.0f, glm::vec3(0, 1, 0));
    cameraFovyLast = sceneData.camera->getFOVy();
    currentRendererSettings = std::map<std::string, std::string>();
    replaySettingsLast = ReplaySettingsMap();

    Py_DECREF(replayFunc);
    Py_DECREF(module);

    return true;
}

bool ReplayWidget::update(float currentTime, bool& stopRecording, bool& stopCameraFlight) {
    directoryContentWatch.update([this] { this->updateAvailableReplayScripts(); });

    if (currentStateIndex >= int(replayStates.size())) {
        if (recording) {
            recording = false;
            stopRecording = true;
        }
        if (useCameraFlight) {
            useCameraFlight = false;
            stopCameraFlight = true;
        }
        return false;
    }

    currentRendererSettings.clear();
    do {
        ReplayState& replayState = replayStates.at(currentStateIndex);

        // Set static attributes that need to be set when the state is entered for the first time.
        if (firstTimeState) {
            if (!replayState.meshDescriptor.meshName.empty()) {
                loadMeshCallback(replayState.meshDescriptor);
            }
            if (!replayState.rendererName.empty()) {
                loadRendererCallback(replayState.rendererName);
            }
            if (!replayState.transferFunctionName.empty()) {
                transferFunctionWindow.loadFunctionFromFile(
                        transferFunctionWindow.getSaveDirectory() + replayState.transferFunctionName);
            }
            if (!replayState.cameraCheckpointName.empty()) {
                sgl::Checkpoint checkpoint;
                if (checkpointWindow.getCheckpoint(replayState.cameraCheckpointName, checkpoint)) {
                    replayState.cameraPositionSet = replayState.cameraOrientationSet = replayState.cameraFovySet = true;
                    replayState.cameraPosition = checkpoint.position;
                    replayState.cameraOrientation = checkpoint.orientation;
                    replayState.cameraFovy = checkpoint.fovy;
                }
            }

            replayState.rendererSettings.setStaticSettings(currentRendererSettings);

            firstTimeState = false;
        }

        if (currentTime >= replayState.stopTime) {
            if (currentStateIndex == int(replayStates.size()) - 1) {
                currentStateIndex++;
                currentTime = replayState.stopTime;
            } else {
                currentStateIndex++;
                if (replayState.cameraPositionSet) {
                    cameraPositionLast = replayState.cameraPosition;
                }
                if (replayState.cameraOrientationSet) {
                    cameraOrientationLast = replayState.cameraOrientation;
                }
                if (replayState.cameraFovySet) {
                    cameraFovyLast = replayState.cameraFovy;
                }

                currentCameraMatrix =
                        glm::toMat4(cameraOrientationLast) * sgl::matrixTranslation(-cameraPositionLast);
                currentFovy = cameraFovyLast;
                replayState.rendererSettings.setDynamicSettings(currentRendererSettings);

                replaySettingsLast.updateReplaySettings(replayState.rendererSettings);
                firstTimeState = true;
                continue;
            }
        }

        // Set per-frame attributes.
        if (replayState.duration >= std::numeric_limits<float>::epsilon()) {
            float t = (currentTime - replayState.startTime) / (replayState.stopTime - replayState.startTime);
            glm::vec3 position = cameraPositionLast;
            glm::quat orientation = cameraOrientationLast;
            float fovy = cameraFovyLast;
            if (replayState.cameraPositionSet) {
                position = glm::mix(cameraPositionLast, replayState.cameraPosition, t);
            }
            if (replayState.cameraOrientationSet) {
                orientation = glm::slerp(cameraOrientationLast, replayState.cameraOrientation, t);
            }
            if (replayState.cameraFovySet) {
                fovy = glm::mix(cameraFovyLast, replayState.cameraFovy, t);
            }
            currentCameraMatrix = glm::toMat4(orientation) * sgl::matrixTranslation(-position);
            currentFovy = fovy;
            replaySettingsLast.setInterpolatedDynamicSettings(
                    replayState.rendererSettings, currentRendererSettings, t);
        }
    } while (currentStateIndex < int(replayStates.size()) && currentTime >= replayStates.at(currentStateIndex).stopTime);

    return true;
}

void ReplayWidget::setLoadMeshCallback(std::function<void(const MeshDescriptor& meshDescriptor)> loadMeshCallback) {
    this->loadMeshCallback = loadMeshCallback;
}

void ReplayWidget::setLoadRendererCallback(std::function<void(const std::string& rendererName)> loadRendererCallback) {
    this->loadRendererCallback = loadRendererCallback;
}

void ReplayWidget::updateAvailableReplayScripts() {
    sgl::FileUtils::get()->ensureDirectoryExists(scriptDirectory);
    std::vector<std::string> availableFilesAll = sgl::FileUtils::get()->getFilesInDirectoryVector(scriptDirectory);
    availableScriptFiles.clear();
    availableScriptFiles.reserve(availableFilesAll.size());

    for (const std::string& filename : availableFilesAll) {
        if (sgl::FileUtils::get()->hasExtension(filename.c_str(), ".py")) {
            availableScriptFiles.push_back(filename);
        }
    }
    sgl::FileUtils::get()->sortPathStrings(availableScriptFiles);

    // Update currently selected filename
    for (size_t i = 0; i < availableScriptFiles.size(); i++) {
        availableScriptFiles.at(i) = sgl::FileUtils::get()->getPureFilename(availableScriptFiles.at(i));
        if (availableScriptFiles.at(i) == scriptFileName) {
            selectedFileIndex = (int)i;
        }
    }
}

ReplayWidget::ReplayWidgetUpdateType ReplayWidget::renderFileDialog() {
    ReplayWidget::ReplayWidgetUpdateType updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_NONE;

    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3184, 1786, 646, 294);
    if (ImGui::Begin("Replay Widget", &showWindow)) {
        // Load file data
        if (ImGui::ListBox(
                "##availablefiles", &selectedFileIndex,
                [this](void *data, int idx, const char **out_text) -> bool {
                    *out_text = availableScriptFiles.at(idx).c_str();
                    return true;
                }, NULL, availableScriptFiles.size(), 4)) {
            scriptFileName = availableScriptFiles.at(selectedFileIndex);
        }
        ImVec2 cursorPosEnd = ImGui::GetCursorPos();
        ImGui::SameLine();

        ImVec2 cursorPos = ImGui::GetCursorPos();
        ImGui::Text("Available files");
        ImGui::SetCursorPos(cursorPos + ImVec2(0.0f, 42.0f));
        ImGui::SetCursorPos(cursorPosEnd);

        if (!recording && currentStateIndex >= int(replayStates.size())) {
            ImGui::InputText("##savefilelabel", &scriptFileName);
            ImGui::SameLine();
            if (ImGui::Button("Run") && !scriptFileName.empty()) {
                bool loadingSuccessful = runScript(scriptFileName);
                if (loadingSuccessful) {
                    updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_LOAD;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Record") && !scriptFileName.empty()) {
                bool loadingSuccessful = runScript(scriptFileName);
                if (loadingSuccessful) {
                    recording = true;
                    updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_START_RECORDING;
                }
            }
        } else {
            if (ImGui::Button("Stop")) {
                currentStateIndex = int(replayStates.size());
                if (recording) {
                    recording = false;
                    updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING;
                } else {
                    updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_STOP;
                }
            }
        }
    }
    ImGui::End();

    return updateType;
}

ReplayWidget::ReplayWidgetUpdateType ReplayWidget::renderGui() {
    return renderFileDialog();
}

ReplayWidget::ReplayWidgetUpdateType ReplayWidget::loadReplicabilityStampState() {
    ReplayWidget::ReplayWidgetUpdateType updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_NONE;
    bool loadingSuccessful = runScript("fig19.py");
    if (loadingSuccessful) {
        updateType = ReplayWidget::REPLAY_WIDGET_UPDATE_LOAD;
    }
    return updateType;
}
