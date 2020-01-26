//
// Created by christoph on 23.01.20.
//

#ifndef HEXVOLUMERENDERER_HEXALABDATASETS_HPP
#define HEXVOLUMERENDERER_HEXALABDATASETS_HPP

#include <string>
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

const std::string meshDirectory = "Data/Meshes/";

struct MeshPaperDescription {
    std::string title, venue, authors, year, PDF, web, DOI;
};

struct MeshSourceDescription {
    MeshPaperDescription paper;
    std::string path, label;
    std::vector<std::string> data;
};

class LoaderThread {
public:
    LoaderThread() : loaderThread() {}
    ~LoaderThread() {
        if (ownsThread) {
            loaderThread.join();
        }
    }
    void setThread(std::thread& thread) {
        ownsThread = true;
        loaderThread = std::move(thread);
    }

private:
    std::thread loaderThread;
    bool ownsThread = false;
};

/// The passed callback is called when the data was loaded successfully.
void downloadHexaLabDataSets(std::function<void()> callback, LoaderThread& loaderThread);
std::vector<MeshSourceDescription> parseSourceDescriptions();

#endif //HEXVOLUMERENDERER_HEXALABDATASETS_HPP
