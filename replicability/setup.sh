#!/bin/bash

# BSD 2-Clause License
#
# Copyright (c) 2021, Christoph Neuhauser
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# A script for reproducing results from our TVCG paper for the replicability stamp
# (http://www.replicabilitystamp.org/). It replicates figure 19 from the paper.

# Reset the current working directory.
START_DIR="${PWD}"

if [[ "${PWD##*/}" == "replicability" ]]; then
    # Go to parent directory.
    cd ..
fi

# We assume we are in the repository directory now.
REPO_DIR="${PWD}"

# Install all necessary packages using the package manager (super user rights required on Ubuntu for this).
if [ ! -d "dependencies" ]; then
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [ -x "$(command -v apt)" ]; then
            echo "Installing necessary packages..."
            sudo apt-get install git cmake g++ libglm-dev libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev \
            libsdl2-ttf-dev  libpng-dev libboost-filesystem-dev libtinyxml2-dev libarchive-dev libglew-dev \
            libjsoncpp-dev libeigen3-dev python3-dev libcurl4-openssl-dev
        elif [ -x "$(command -v pacman)" ]; then
            echo "Installing necessary packages..."
            sudo pacman -S git cmake glew boost libarchive glm tinyxml2 sdl2 sdl2_image python3 eigen curl jsoncpp
        fi
    elif [[ "$OSTYPE" == "msys"* ]]; then
        echo "Installing necessary packages..."
        echo "y" | pacman -S make git wget mingw64/mingw-w64-x86_64-gcc mingw64/mingw-w64-x86_64-gdb
        echo "y" | pacman -S mingw64/mingw-w64-x86_64-glm mingw64/mingw-w64-x86_64-libpng mingw64/mingw-w64-x86_64-SDL2 \
        mingw64/mingw-w64-x86_64-SDL2_image mingw64/mingw-w64-x86_64-SDL2_mixer mingw64/mingw-w64-x86_64-SDL2_ttf \
        mingw64/mingw-w64-x86_64-tinyxml2 mingw64/mingw-w64-x86_64-boost mingw64/mingw-w64-x86_64-glew \
        mingw64/mingw-w64-x86_64-cmake mingw64/mingw-w64-x86_64-libarchive \
        mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-curl mingw64/mingw-w64-x86_64-eigen3 \
        mingw64/mingw-w64-x86_64-python mingw64/mingw-w64-x86_64-embree
    else
        echo "Unknown operating system $OSTYPE." >&2
        exit 1
    fi
fi

# Download, compile and install sgl.
if [ ! -d "dependencies/sgl" ]; then
    echo "Compiling and installing sgl..."
    mkdir dependencies
    mkdir dependencies/sgl
    mkdir dependencies/sgl-repo
    git clone https://github.com/chrismile/sgl.git dependencies/sgl-repo
    git checkout tvcg-hexmesh
    cd dependencies/sgl-repo
    mkdir build
    cd build
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${REPO_DIR}/dependencies/sgl" ..
    elif [[ "$OSTYPE" == "msys"* ]]; then
        cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${REPO_DIR}/dependencies/sgl" ..
    fi
    make -j
    make install
    cd "${REPO_DIR}"
fi

# Add the path to the shared library compiled for sgl.
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if [[ ! "${LD_LIBRARY_PATH}" == *"${REPO_DIR}/dependencies/sgl/lib"* ]]; then
        export LD_LIBRARY_PATH=LD_LIBRARY_PATH:"${REPO_DIR}/dependencies/sgl/lib"
    fi
elif [[ "$OSTYPE" == "msys"* ]]; then
    if [[ ! "${PATH}" == *"${REPO_DIR}/dependencies/sgl/bin"* ]]; then
        export PATH=$PATH:"${REPO_DIR}/dependencies/sgl/bin"
    fi
fi

# Compile HexVolumeRenderer.
if [ ! -d "build" ]; then
    mkdir build
    cd build
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        cmake -DCMAKE_BUILD_TYPE=Release -Dsgl_DIR="${REPO_DIR}/dependencies/sgl/lib/cmake/sgl" ..
    elif [[ "$OSTYPE" == "msys"* ]]; then
        cmake -G "MSYS Makefiles" -DPython3_FIND_REGISTRY=NEVER -DCMAKE_BUILD_TYPE=Release -Dsgl_DIR="${REPO_DIR}/dependencies/sgl/lib/cmake/sgl" ..
    fi
    make -j
    cd ..
fi

# Download the data sets.
if [ ! -d "Data/Meshes/2019 - Symmetric Moving Frames" ]; then
    # This tool can help download data sets from HexaLab.
    # The program assumes the user has the rights to download the data.
    #
    # HexaLab.net: an online viewer for hexahedral meshes
    # Matteo Bracci, Marco Tarini, Nico Pietroni, Marco Livesu, Paolo Cignoni
    # Computer-Aided Design, Volume 110, May 2019
    # DOI:10.1016/j.cad.2018.12.003
    # (preprint available on arxiv)
    # Copyright 2018 Visual Computing Lab ISTI - CNR
    echo "Downloading HexaLab data sets..."
    build/HexaLabDatasetsDownloader
fi

# Adapt the Python home path if msys2 is used.
if [[ "$OSTYPE" == "msys"* ]]; then
    if [[ ! "${PYTHONHOME}" == *"/mingw64"* ]]; then
        export PYTHONHOME="/mingw64"
    fi
fi

# Run the program.
build/HexVolumeRenderer --replicability

# Reset the working directory.
cd "${START_DIR}"
