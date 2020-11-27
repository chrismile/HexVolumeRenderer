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

#ifndef HEXVOLUMERENDERER_AUTOMATICPERFORMANCEMEASURER_HPP
#define HEXVOLUMERENDERER_AUTOMATICPERFORMANCEMEASURER_HPP

#include <string>
#include <functional>
#include <Utils/File/CsvWriter.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/Texture/Bitmap.hpp>
#include <Graphics/OpenGL/TimerGL.hpp>

#include "InternalState.hpp"

const float TIME_PERFORMANCE_MEASUREMENT = 128.0f;

class AutomaticPerformanceMeasurer {
public:
    AutomaticPerformanceMeasurer(std::vector<InternalState> _states,
    const std::string& _csvFilename, const std::string& _depthComplexityFilename,
            std::function<void(const InternalState&)> _newStateCallback);
    ~AutomaticPerformanceMeasurer();

    // To be called by the application
    void setInitialFreeMemKilobytes(int initialFreeMemKilobytes);
    void startMeasure(float timeStamp);
    void endMeasure();

    /// Returns false if all modes were tested and the app should terminate.
    bool update(float currentTime);

    // Called by OIT_DepthComplexity
    void pushDepthComplexityFrame(
            uint64_t minComplexity, uint64_t maxComplexity, float avgUsed, float avgAll, uint64_t totalNumFragments);

    // Called by OIT algorithms.
    void setCurrentAlgorithmBufferSizeBytes(size_t numBytes);
    inline void setClearViewTimer(sgl::TimerGL* clearViewTimer) { this->clearViewTimer = clearViewTimer; }

private:
    /// Write out the performance data of "currentState" to "file".
    void writeCurrentModeData();
    /// Switch to the next state in "states".
    void setNextState(bool first = false);

    /// Returns amount of used video memory size in GiB.
    float getUsedVideoMemorySizeGiB();

    std::vector<InternalState> states;
    size_t currentStateIndex;
    InternalState currentState;
    std::function<void(const InternalState&)> newStateCallback; // Application callback

    float nextModeCounter = 0.0f;

    sgl::TimerGL timerGL;
    int initialFreeMemKilobytes;
    sgl::CsvWriter file;
    sgl::CsvWriter depthComplexityFile;
    sgl::CsvWriter perfFile;
    size_t depthComplexityFrameNumber = 0;
    size_t currentAlgorithmsBufferSizeBytes = 0;

    // For depth complexity renderer.
    bool newDepthComplexityMode = true;
    size_t maxPPLLNumFragments = 0;

    // For ClearView unified renderer.
    sgl::CsvWriter clearViewFile;
    sgl::TimerGL* clearViewTimer = nullptr;
};


#endif //HEXVOLUMERENDERER_AUTOMATICPERFORMANCEMEASURER_HPP
