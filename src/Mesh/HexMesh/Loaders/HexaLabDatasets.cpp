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

#include <iostream>
#include <thread>

#include <curl/curl.h>
#include <json/json.h>

#include <Utils/AppSettings.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/Logfile.hpp>

#include "HexaLabDatasets.hpp"

const std::string hexaLabBaseUrl = "https://www.hexalab.net/datasets/";

static size_t writeDataCallbackCurl(void *pointer, size_t size, size_t numMembers, void *stream) {
    size_t written = fwrite(pointer, size, numMembers, (FILE*)stream);
    return written;
}

void downloadFile(const std::string &url, const std::string &localFileName) {
    CURL *curlHandle = curl_easy_init();

    char *compressedUrl = curl_easy_escape(curlHandle, url.c_str(), url.size());
    std::string fixedUrl = compressedUrl;
    sgl::stringReplaceAll(fixedUrl, "%3A", ":");
    sgl::stringReplaceAll(fixedUrl, "%2F", "/");
    std::cout << "Starting to download \"" << fixedUrl << "\"..." << std::endl;


    curl_easy_setopt(curlHandle, CURLOPT_URL, fixedUrl.c_str());
    //curl_easy_setopt(curlHandle, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(curlHandle, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, writeDataCallbackCurl);
    FILE *pagefile = fopen(localFileName.c_str(), "wb");
    if (pagefile) {
        curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, pagefile);
        curl_easy_perform(curlHandle);
        fclose(pagefile);
    }
    curl_free(compressedUrl);

    curl_easy_cleanup(curlHandle);
}

static volatile bool downloadThreadIsRunning = false;

void downloadHexaLabDataSetThreadFunction(std::function<void()> callback) {
    // Load the configuration file.
    curl_global_init(CURL_GLOBAL_ALL);
    const std::string meshDirectory = sgl::AppSettings::get()->getDataDirectory() + "Meshes/";
    const std::string indexFileName = meshDirectory + "index.json";
    downloadFile(hexaLabBaseUrl + "index.json", indexFileName);

    // Now, parse the index.json file.
    std::ifstream jsonFileStream(indexFileName);
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errorString;
    Json::Value root;
    if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
        sgl::Logfile::get()->writeError(errorString);
        curl_global_cleanup();
        return;
    }
    jsonFileStream.close();

    // Download all indexed mesh files.
    Json::Value sources = root["sources"];
    for (Json::Value::const_iterator sourceIt = sources.begin(); sourceIt != sources.end(); ++sourceIt) {
        Json::Value data = (*sourceIt)["data"];
        for (Json::Value::const_iterator dataIt = data.begin(); dataIt != data.end(); ++dataIt) {
            std::string relativeFileName = (*sourceIt)["path"].asString() + "/" + dataIt->asString();
            std::string fileUrl = hexaLabBaseUrl + relativeFileName;
            std::string filePath = meshDirectory + relativeFileName;
            sgl::FileUtils::get()->ensureDirectoryExists(sgl::FileUtils::get()->getPathToFile(filePath));
            if (!sgl::FileUtils::get()->exists(filePath)) {
                downloadFile(fileUrl, filePath);
            }
        }
    }
    curl_global_cleanup();
    callback();
    downloadThreadIsRunning = false;
}

void downloadHexaLabDataSets(std::function<void()> callback, LoaderThread& loaderThread) {
    const std::string meshDirectory = sgl::AppSettings::get()->getDataDirectory() + "Meshes/";
    if (sgl::FileUtils::get()->exists(meshDirectory)) {
        // Already downloaded
        return;
    }
    sgl::FileUtils::get()->ensureDirectoryExists(meshDirectory);

    // Let the program stay responsible by detaching the download thread.
    if (!downloadThreadIsRunning) {
        downloadThreadIsRunning = true;
        std::thread downloadThread(downloadHexaLabDataSetThreadFunction, callback);
        loaderThread.setThread(downloadThread);
    }
}

bool getIsDataSetDownloadRunning() {
    return downloadThreadIsRunning;
}

std::vector<MeshSourceDescription> parseSourceDescriptions() {
    std::vector<MeshSourceDescription> meshSourceDescriptions;

    // Parse the index.json file.
    const std::string meshDirectory = sgl::AppSettings::get()->getDataDirectory() + "Meshes/";
    const std::string indexFileName = meshDirectory + "index.json";
    std::ifstream jsonFileStream(indexFileName);
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errorString;
    Json::Value root;
    if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
        sgl::Logfile::get()->writeError(errorString);
        return meshSourceDescriptions;
    }
    jsonFileStream.close();

    Json::Value sources = root["sources"];
    for (Json::Value::const_iterator sourceIt = sources.begin(); sourceIt != sources.end(); ++sourceIt) {
        Json::Value paper = root["paper"];
        MeshSourceDescription source;
        source.path = (*sourceIt)["path"].asString();
        source.label = (*sourceIt)["label"].asString();
        source.paper.title = paper["title"].asString();
        source.paper.venue = paper["venue"].asString();
        source.paper.authors = paper["authors"].asString();
        source.paper.year = paper["year"].asString();
        source.paper.PDF = paper["PDF"].asString();
        source.paper.web = paper["web"].asString();
        source.paper.DOI = paper["DOI"].asString();
        Json::Value data = (*sourceIt)["data"];
        for (Json::Value::const_iterator dataIt = data.begin(); dataIt != data.end(); ++dataIt) {
            std::vector<std::string> additionalFiles;
            if (dataIt->isString()) {
                source.data.push_back(dataIt->asString());
            } else if (dataIt->isArray()) {
                int filenameIdx = 0;
                for (Json::Value::const_iterator filenameIt = dataIt->begin(); filenameIt != dataIt->end(); ++filenameIt) {
                    if (filenameIdx == 0) {
                        source.data.push_back(filenameIt->asString());
                    } else {
                        additionalFiles.push_back(filenameIt->asString());
                    }
                    filenameIdx++;
                }
            } else {
                sgl::Logfile::get()->writeError("ERROR in parseSourceDescriptions: 'data' is no string or array.");
            }
            source.dataAdditionalFiles.push_back(additionalFiles);
        }
        meshSourceDescriptions.push_back(source);
    }
    return meshSourceDescriptions;
}