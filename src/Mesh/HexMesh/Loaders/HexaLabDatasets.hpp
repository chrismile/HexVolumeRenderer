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

#ifndef HEXVOLUMERENDERER_HEXALABDATASETS_HPP
#define HEXVOLUMERENDERER_HEXALABDATASETS_HPP

#include <string>
#include <vector>
#include <functional>
#include <thread>

/**
 * This file can help download data sets from HexaLab. The program assumes the user has the rights to download the data.
 *
 * HexaLab.net: an online viewer for hexahedral meshes
 * Matteo Braccix, Marco Tarini1,2,x, Nico Pietroni1,4, Marco Livesu3, Paolo Cignoni1
 * Computer-Aided Design, Volume 110, May 2019
 * DOI:10.1016/j.cad.2018.12.003
 * (preprint available on arxiv)
 * Copyright 2018 Visual Computing Lab ISTI - CNR
 */

struct MeshPaperDescription {
    std::string title, venue, authors, year, PDF, web, DOI;
};

struct MeshSourceDescription {
    MeshPaperDescription paper;
    std::string path, label;
    std::vector<std::string> data;
    std::vector<std::vector<std::string>> dataAdditionalFiles;
};

class LoaderThread {
public:
    LoaderThread() : loaderThread() {}
    ~LoaderThread() {
        if (ownsThread && loaderThread.joinable()) {
            loaderThread.join();
        }
    }
    void setThread(std::thread& thread) {
        ownsThread = true;
        loaderThread = std::move(thread);
    }
    void join() {
        if (loaderThread.joinable()) {
            loaderThread.join();
        }
    }

private:
    std::thread loaderThread;
    bool ownsThread = false;
};

/// The passed callback is called when the data was loaded successfully.
void downloadHexaLabDataSets(std::function<void()> callback, LoaderThread& loaderThread);
bool getIsDataSetDownloadRunning();
std::vector<MeshSourceDescription> parseSourceDescriptions();

#endif //HEXVOLUMERENDERER_HEXALABDATASETS_HPP
