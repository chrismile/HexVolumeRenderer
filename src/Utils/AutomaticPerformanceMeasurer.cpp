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

#include <GL/glew.h>

#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Texture/Bitmap.hpp>
#include <Graphics/OpenGL/SystemGL.hpp>

#include "AutomaticPerformanceMeasurer.hpp"

AutomaticPerformanceMeasurer::AutomaticPerformanceMeasurer(std::vector<InternalState> _states,
                                   const std::string& _csvFilename, const std::string& _depthComplexityFilename,
                                   std::function<void(const InternalState&)> _newStateCallback)
        : states(_states), currentStateIndex(0), newStateCallback(_newStateCallback), file(_csvFilename),
          depthComplexityFile(_depthComplexityFilename), perfFile("performance_list.csv") {
    sgl::FileUtils::get()->ensureDirectoryExists("images/");

    // Write header
    file.writeRow({"Name", "Average Time (ms)", "Memory (GiB)", "Buffer Size (GiB)", "Time Stamp (s), Frame Time (ns)"});
    depthComplexityFile.writeRow({"Current State", "Frame Number", "Min Depth Complexity", "Max Depth Complexity",
                                  "Avg Depth Complexity Used", "Avg Depth Complexity All", "Total Number of Fragments"});
    perfFile.writeRow({"Name", "Time per frame (ms)"});

    // Set initial state
    setNextState(true);
}

AutomaticPerformanceMeasurer::~AutomaticPerformanceMeasurer() {
    writeCurrentModeData();
    file.close();
    depthComplexityFile.close();
    perfFile.close();
}


float nextModeCounter = 0.0f;
const float TIME_PER_MODE = 32.5f; // in seconds
bool AutomaticPerformanceMeasurer::update(float currentTime) {
    nextModeCounter = currentTime;
    if (nextModeCounter >= TIME_PER_MODE) {
        nextModeCounter = 0.0f;
        if (currentStateIndex == states.size()-1) {
            return false; // Terminate program
        }
        setNextState();
    }
    return true;
}


void AutomaticPerformanceMeasurer::writeCurrentModeData() {
    // Write row with performance metrics of this mode
    timerGL.stopMeasuring();
    double timeMS = timerGL.getTimeMS(currentState.name);
    file.writeCell(currentState.name);
    perfFile.writeCell(currentState.name);
    file.writeCell(sgl::toString(timeMS));

    // Write current memory consumption in gigabytes
    file.writeCell(sgl::toString(getUsedVideoMemorySizeGiB()));
    file.writeCell(sgl::toString(currentAlgorithmsBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));


    auto performanceProfile = timerGL.getCurrentFrameTimeList();
    for (auto &perfPair : performanceProfile) {
        float timeStamp = perfPair.first;
        uint64_t frameTimeNS = perfPair.second;
        float frameTimeMS = float(frameTimeNS) / float(1.0E6);
        file.writeCell(sgl::toString(frameTimeMS));
        perfFile.writeCell(sgl::toString(frameTimeMS));
    }

    file.newRow();
    perfFile.newRow();
}

void AutomaticPerformanceMeasurer::setNextState(bool first) {
    if (!first) {
        writeCurrentModeData();
        currentStateIndex++;
    }

    depthComplexityFrameNumber = 0;
    currentAlgorithmsBufferSizeBytes = 0;
    currentState = states.at(currentStateIndex);
    sgl::Logfile::get()->writeInfo(std::string() + "New state: " + currentState.name);
    newStateCallback(currentState);
}

void AutomaticPerformanceMeasurer::startMeasure(float timeStamp) {
    //if (currentState.oitAlgorithm == RENDER_MODE_RAYTRACING) {
        // CPU rendering algorithm, thus use a CPU timer and not a GPU timer.
        //timerGL.startCPU(currentState.name, timeStamp);
    //} else {
    timerGL.startGPU(currentState.name, timeStamp);
    //}
}

void AutomaticPerformanceMeasurer::endMeasure() {
    timerGL.end();
}

void AutomaticPerformanceMeasurer::pushDepthComplexityFrame(
        uint64_t minComplexity, uint64_t maxComplexity, float avgUsed, float avgAll, uint64_t totalNumFragments) {
    depthComplexityFile.writeCell(currentState.name);
    depthComplexityFile.writeCell(sgl::toString((int)depthComplexityFrameNumber));
    depthComplexityFile.writeCell(sgl::toString((int)minComplexity));
    depthComplexityFile.writeCell(sgl::toString((int)maxComplexity));
    depthComplexityFile.writeCell(sgl::toString(avgUsed));
    depthComplexityFile.writeCell(sgl::toString(avgAll));
    depthComplexityFile.writeCell(sgl::toString((int)totalNumFragments));
    depthComplexityFile.newRow();
    depthComplexityFrameNumber++;
}

void AutomaticPerformanceMeasurer::setCurrentAlgorithmBufferSizeBytes(size_t numBytes) {
    currentAlgorithmsBufferSizeBytes = numBytes;
}

/*#ifndef GL_QUERY_RESOURCE_TYPE_VIDMEM_ALLOC_NV
#include <SDL2/SDL.h>
#endif*/

void AutomaticPerformanceMeasurer::setInitialFreeMemKilobytes(int initialFreeMemKilobytes) {
    this->initialFreeMemKilobytes = initialFreeMemKilobytes;
}

float AutomaticPerformanceMeasurer::getUsedVideoMemorySizeGiB() {
    // https://www.khronos.org/registry/OpenGL/extensions/NVX/NVX_gpu_memory_info.txt
    if (sgl::SystemGL::get()->isGLExtensionAvailable("GL_NVX_gpu_memory_info")) {
        GLint freeMemKilobytes = 0;
        glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &freeMemKilobytes);
        float usedGiB = (initialFreeMemKilobytes - freeMemKilobytes) * 1000.0 / 1024.0 / 1024.0 / 1024.0;
        return usedGiB;
    }
    // https://www.khronos.org/registry/OpenGL/extensions/NV/NV_query_resource.txt
    /*if (sgl::SystemGL::get()->isGLExtensionAvailable("GL_NV_query_resource")) {
        // Doesn't work for whatever reason :(
        /*GLint buffer[4096];
#ifndef GL_QUERY_RESOURCE_TYPE_VIDMEM_ALLOC_NV
#define GL_QUERY_RESOURCE_TYPE_VIDMEM_ALLOC_NV 0x9540
        typedef GLint (*PFNGLQUERYRESOURCENVPROC) (GLenum queryType, GLint tagId, GLuint bufSize, GLint *buffer);
        PFNGLQUERYRESOURCENVPROC glQueryResourceNV
                = (PFNGLQUERYRESOURCENVPROC)SDL_GL_GetProcAddress("glQueryResourceNV");
        glQueryResourceNV(GL_QUERY_RESOURCE_TYPE_VIDMEM_ALLOC_NV, -1, 4096, buffer);
#else
        glQueryResourceNV(GL_QUERY_RESOURCE_TYPE_VIDMEM_ALLOC_NV, 0, 6, buffer);
#endif
        // Used video memory stored at int at index 5 (in kilobytes).
        size_t usedKB = buffer[5];
        float usedGiB = (usedKB * 1000) / 1024.0 / 1024.0 / 1024.0;
        return usedGiB;
    }*/

    // Fallback
    return 0.0f;
}
