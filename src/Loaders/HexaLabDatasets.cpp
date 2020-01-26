//
// Created by christoph on 23.01.20.
//

#include <iostream>
#include <thread>

#include <boost/algorithm/string/replace.hpp>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>

#include <Utils/File/FileUtils.hpp>
#include <Utils/File/Logfile.hpp>

#include "HexaLabDatasets.hpp"

const std::string indexFileName = meshDirectory + "index.json";
const std::string hexaLabBaseUrl = "https://www.hexalab.net/datasets/";

static size_t writeDataCallbackCurl(void *pointer, size_t size, size_t numMembers, void *stream) {
    size_t written = fwrite(pointer, size, numMembers, (FILE*)stream);
    return written;
}

void downloadFile(const std::string &url, const std::string &localFileName) {
    CURL *curlHandle = curl_easy_init();

    char *compressedUrl = curl_easy_escape(curlHandle, url.c_str(), url.size());
    std::string fixedUrl = compressedUrl;
    boost::replace_all(fixedUrl, "%3A", ":");
    boost::replace_all(fixedUrl, "%2F", "/");
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

std::vector<MeshSourceDescription> parseSourceDescriptions() {
    std::vector<MeshSourceDescription> meshSourceDescriptions;

    // Parse the index.json file.
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
            source.data.push_back(dataIt->asString());
        }
        meshSourceDescriptions.push_back(source);
    }
    return meshSourceDescriptions;
}