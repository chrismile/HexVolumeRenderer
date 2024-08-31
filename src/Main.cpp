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

#ifdef USE_PYTHON
#ifdef PYTHONHOME_PATH
#include <cstdlib>
#endif
#include <Python.h>
#endif

#include <Utils/File/FileUtils.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/AppLogic.hpp>
#include <Graphics/Window.hpp>

#ifdef SGL_INPUT_API_V2
#include <Graphics/Utils/HiDPI.hpp>
#else
#include <SDL/HiDPI.hpp>
#endif

#include "MainApp.hpp"

int main(int argc, char *argv[]) {
    // Initialize the filesystem utilities
    sgl::FileUtils::get()->initialize("HexVolumeRenderer", argc, argv);

    // Load the file containing the app settings
    std::string settingsFile = sgl::FileUtils::get()->getConfigDirectory() + "settings.txt";
    sgl::AppSettings::get()->loadSettings(settingsFile.c_str());
    sgl::AppSettings::get()->getSettings().addKeyValue("window-multisamples", 0);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-debugContext", true);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-vSync", true);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-resizable", true);
#ifdef DATA_PATH
    if (!sgl::FileUtils::get()->directoryExists("Data") && !sgl::FileUtils::get()->directoryExists("../Data")) {
        sgl::AppSettings::get()->setDataDirectory(DATA_PATH);
    }
#endif
    sgl::AppSettings::get()->initializeDataDirectory();

    std::string iconPath = sgl::AppSettings::get()->getDataDirectory() + "Fonts/icon_256.png";
    sgl::AppSettings::get()->setApplicationDescription("Visualization tool for hex meshes using transparent volumes");
    sgl::AppSettings::get()->loadApplicationIconFromFile(iconPath);

    ImVector<ImWchar> fontRanges;
    ImFontGlyphRangesBuilder builder;
    builder.AddChar(L'\u03BB'); // lambda
    builder.AddChar(L'\u2113'); // ell / SCRIPT SMALL L
    builder.BuildRanges(&fontRanges);
    bool useMultiViewport = false;
    if (sgl::AppSettings::get()->getSettings().getValueOpt("useDockSpaceMode", useMultiViewport)) {
        useMultiViewport = !useMultiViewport;
    }
#ifdef USE_STEAMWORKS
    sgl::overwriteHighDPIScaleFactor(1.2f);
    sgl::AppSettings::get()->setLoadGUI(fontRanges.Data, true, useMultiViewport, 1.0f);
#else
    sgl::AppSettings::get()->setLoadGUI(fontRanges.Data, true, useMultiViewport);
#endif
    sgl::Window* window = sgl::AppSettings::get()->createWindow();
    sgl::AppSettings::get()->initializeSubsystems();

#ifdef USE_PYTHON
#ifdef PYTHONHOME
    const char* pythonhomeEnvVar = getenv("PYTHONHOME");
    if (!pythonhomeEnvVar || strlen(pythonhomeEnvVar) == 0) {
        Py_SetPythonHome(PYTHONHOME);
        // As of 2022-01-25, "lib-dynload" is not automatically found when using MSYS2 together with MinGW.
#if defined(__MINGW32__) && defined(PYTHONPATH)
        Py_SetPath(PYTHONPATH ";" PYTHONPATH "/site-packages;" PYTHONPATH "/lib-dynload");
#endif
    }
#endif
    // PyMem_Malloc causes segmentation fault on Fedora 39.
    //wchar_t** argvWidestr = (wchar_t**)PyMem_Malloc(sizeof(wchar_t*) * argc);
    wchar_t** argvWidestr = (wchar_t**)malloc(sizeof(wchar_t*) * argc);
    for (int i = 0; i < argc; i++) {
        wchar_t* argWidestr = Py_DecodeLocale(argv[i], nullptr);
        argvWidestr[i] = argWidestr;
    }
    Py_Initialize();
    PySys_SetArgv(argc, argvWidestr);
#endif

    MainApp* app = new MainApp();

    // Code path for replicability stamp.
    if (argc == 2 && strcmp(argv[1], "--replicability") == 0) {
        app->loadReplicabilityStampState();
    }

    app->run();
    delete app;

    sgl::AppSettings::get()->release();

#ifdef USE_PYTHON
    Py_Finalize();
#endif

    return 0;
}
