#!/usr/bin/env bash

# BSD 2-Clause License
#
# Copyright (c) 2021-2023, Christoph Neuhauser
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

# Conda crashes with "set -euo pipefail".
set -eo pipefail

scriptpath="$( cd "$(dirname "$0")" ; pwd -P )"
projectpath="$scriptpath"
pushd $scriptpath > /dev/null

if [[ "$(uname -s)" =~ ^MSYS_NT.* ]] || [[ "$(uname -s)" =~ ^MINGW.* ]]; then
    use_msys=true
else
    use_msys=false
fi
if [[ "$(uname -s)" =~ ^Darwin.* ]]; then
    use_macos=true
else
    use_macos=false
fi
os_arch="$(uname -m)"

run_program=true
debug=false
glibcxx_debug=false
clean=false
build_dir_debug=".build_debug"
build_dir_release=".build_release"
use_vcpkg=false
use_conda=false
conda_env_name="hexvolumerenderer"
link_dynamic=false
use_custom_vcpkg_triplet=false
if [ $use_msys = false ] && command -v pacman &> /dev/null; then
    is_embree_installed=true
else
    is_embree_installed=false
fi
# Replicability Stamp (https://www.replicabilitystamp.org/) mode for replicating a figure from the corresponding paper.
replicability=false

# Check if a conda environment is already active.
if $use_conda; then
    if [ ! -z "${CONDA_DEFAULT_ENV+x}" ]; then
        conda_env_name="$CONDA_DEFAULT_ENV"
    fi
fi

# Process command line arguments.
for ((i=1;i<=$#;i++));
do
    if [ ${!i} = "--do-not-run" ]; then
        run_program=false
    elif [ ${!i} = "--debug" ] || [ ${!i} = "debug" ]; then
        debug=true
    elif [ ${!i} = "--glibcxx-debug" ]; then
        glibcxx_debug=true
    elif [ ${!i} = "--clean" ] || [ ${!i} = "clean" ]; then
        clean=true
    elif [ ${!i} = "--vcpkg" ] || [ ${!i} = "--use-vcpkg" ]; then
        use_vcpkg=true
    elif [ ${!i} = "--conda" ] || [ ${!i} = "--use-conda" ]; then
        use_conda=true
    elif [ ${!i} = "--conda-env-name" ]; then
        ((i++))
        conda_env_name=${!i}
    elif [ ${!i} = "--link-static" ]; then
        link_dynamic=false
    elif [ ${!i} = "--link-dynamic" ]; then
        link_dynamic=true
    elif [ ${!i} = "--vcpkg-triplet" ]; then
        ((i++))
        vcpkg_triplet=${!i}
        use_custom_vcpkg_triplet=true
    elif [ ${!i} = "--replicability" ]; then
        replicability=true
    fi
done

if [ $clean = true ]; then
    echo "------------------------"
    echo " cleaning up old files  "
    echo "------------------------"
    rm -rf third_party/vcpkg/ .build_release/ .build_debug/ Shipping/
    if grep -wq "sgl" .gitmodules; then
        rm -rf third_party/sgl/install/ third_party/sgl/.build_release/ third_party/sgl/.build_debug/
    else
        rm -rf third_party/sgl/
    fi
    git submodule update --init --recursive
fi

if [ $debug = true ]; then
    cmake_config="Debug"
    build_dir=$build_dir_debug
else
    cmake_config="Release"
    build_dir=$build_dir_release
fi
destination_dir="Shipping"
if $use_macos; then
    binaries_dest_dir="$destination_dir/HexVolumeRenderer.app/Contents/MacOS"
    if ! command -v brew &> /dev/null; then
        if [ ! -d "/opt/homebrew/bin" ]; then
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        fi
        if [ -d "/opt/homebrew/bin" ]; then
            #echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/$USER/.zprofile
            eval "$(/opt/homebrew/bin/brew shellenv)"
        fi
    fi
fi

params_link=()
params_vcpkg=()
build_sgl_release_only=false
if [ $use_custom_vcpkg_triplet = true ]; then
    params_link+=(-DVCPKG_TARGET_TRIPLET=$vcpkg_triplet)
    if [ -f "$projectpath/third_party/vcpkg/triplets/$vcpkg_triplet.cmake" ]; then
        triplet_file_path="$projectpath/third_party/vcpkg/triplets/$vcpkg_triplet.cmake"
    elif [ -f "$projectpath/third_party/vcpkg/triplets/community/$vcpkg_triplet.cmake" ]; then
        triplet_file_path="$projectpath/third_party/vcpkg/triplets/community/$vcpkg_triplet.cmake"
    else
        echo "Custom vcpkg triplet set, but file not found."
        exit 1
    fi
    if grep -q "VCPKG_BUILD_TYPE release" "$triplet_file_path"; then
        build_sgl_release_only=true
    fi
elif [ $use_vcpkg = true ] && [ $use_macos = false ] && [ $link_dynamic = true ]; then
    params_link+=(-DVCPKG_TARGET_TRIPLET=x64-linux-dynamic)
fi
if [ $use_vcpkg = true ] && [ $use_macos = false ]; then
    params_vcpkg+=(-DUSE_STATIC_STD_LIBRARIES=On)
fi
if [ $use_vcpkg = true ]; then
    params_vcpkg+=(-DCMAKE_TOOLCHAIN_FILE="$projectpath/third_party/vcpkg/scripts/buildsystems/vcpkg.cmake")
fi

is_installed_apt() {
    local pkg_name="$1"
    if [ "$(dpkg -l | awk '/'"$pkg_name"'/ {print }'|wc -l)" -ge 1 ]; then
        return 0
    else
        return 1
    fi
}

is_installed_pacman() {
    local pkg_name="$1"
    if pacman -Qs $pkg_name > /dev/null; then
        return 0
    else
        return 1
    fi
}

is_installed_yay() {
    local pkg_name="$1"
    if yay -Ss $pkg_name > /dev/null | grep -q 'instal'; then
        return 1
    else
        return 0
    fi
}

is_installed_yum() {
    local pkg_name="$1"
    if yum list installed "$pkg_name" > /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

is_installed_rpm() {
    local pkg_name="$1"
    if rpm -q "$pkg_name" > /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

is_installed_brew() {
    local pkg_name="$1"
    if brew list $pkg_name > /dev/null; then
        return 0
    else
        return 1
    fi
}

# https://stackoverflow.com/questions/8063228/check-if-a-variable-exists-in-a-list-in-bash
list_contains() {
    if [[ "$1" =~ (^|[[:space:]])"$2"($|[[:space:]]) ]]; then
        return 0
    else
        return 1
    fi
}

if $use_msys && command -v pacman &> /dev/null && [ ! -d $build_dir_debug ] && [ ! -d $build_dir_release ]; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v rsync &> /dev/null \
            || ! command -v curl &> /dev/null || ! command -v wget &> /dev/null || ! command -v unzip &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v ntldd &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        pacman --noconfirm -S --needed make git rsync curl wget unzip mingw64/mingw-w64-x86_64-cmake \
        mingw64/mingw-w64-x86_64-gcc mingw64/mingw-w64-x86_64-gdb mingw-w64-x86_64-ntldd
    fi

    # Dependencies of sgl and the application.
    if ! is_installed_pacman "mingw-w64-x86_64-boost" || ! is_installed_pacman "mingw-w64-x86_64-icu" \
            || ! is_installed_pacman "mingw-w64-x86_64-glm" || ! is_installed_pacman "mingw-w64-x86_64-libarchive" \
            || ! is_installed_pacman "mingw-w64-x86_64-tinyxml2" || ! is_installed_pacman "mingw-w64-x86_64-libpng" \
            || ! is_installed_pacman "mingw-w64-x86_64-SDL2" || ! is_installed_pacman "mingw-w64-x86_64-SDL2_image" \
            || ! is_installed_pacman "mingw-w64-x86_64-glew" || ! is_installed_pacman "mingw-w64-x86_64-jsoncpp" \
            || ! is_installed_pacman "mingw-w64-x86_64-eigen3" || ! is_installed_pacman "mingw-w64-x86_64-python" \
            || ! is_installed_pacman "mingw-w64-x86_64-embree"; then
        echo "------------------------"
        echo "installing dependencies "
        echo "------------------------"
        pacman --noconfirm -S --needed mingw64/mingw-w64-x86_64-boost mingw64/mingw-w64-x86_64-icu \
        mingw64/mingw-w64-x86_64-glm mingw64/mingw-w64-x86_64-libarchive mingw64/mingw-w64-x86_64-tinyxml2 \
        mingw64/mingw-w64-x86_64-libpng mingw64/mingw-w64-x86_64-SDL2 mingw64/mingw-w64-x86_64-SDL2_image \
        mingw64/mingw-w64-x86_64-glew mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-eigen3 \
        mingw64/mingw-w64-x86_64-python mingw64/mingw-w64-x86_64-embree
    fi
    if ! (is_installed_pacman "mingw-w64-x86_64-curl" || is_installed_pacman "mingw-w64-x86_64-curl-gnutls" \
            || is_installed_pacman "mingw-w64-x86_64-curl-winssl"); then
        pacman --noconfirm -S --needed mingw64/mingw-w64-x86_64-curl
    fi
elif $use_msys && command -v pacman &> /dev/null; then
    :
elif [ ! -z "${BUILD_USE_NIX+x}" ]; then
    echo "------------------------"
    echo "   building using Nix"
    echo "------------------------"
elif $use_macos && command -v brew &> /dev/null && [ ! -d $build_dir_debug ] && [ ! -d $build_dir_release ]; then
    if ! is_installed_brew "git"; then
        brew install git
    fi
    if ! is_installed_brew "cmake"; then
        brew install cmake
    fi
    if ! is_installed_brew "curl"; then
        brew install curl
    fi
    if ! is_installed_brew "pkg-config"; then
        brew install pkg-config
    fi
    if ! is_installed_brew "llvm"; then
        brew install llvm
    fi
    if ! is_installed_brew "libomp"; then
        brew install libomp
    fi
    if ! is_installed_brew "autoconf"; then
        brew install autoconf
    fi
    if ! is_installed_brew "automake"; then
        brew install automake
    fi
    if ! is_installed_brew "autoconf-archive"; then
        brew install autoconf-archive
    fi

    # Homebrew MoltenVK does not contain script for setting environment variables, unfortunately.
    #if ! is_installed_brew "molten-vk"; then
    #    brew install molten-vk
    #fi

    # Dependencies of sgl and the application.
    if [ $use_vcpkg = false ]; then
        if ! is_installed_brew "boost"; then
            brew install boost
        fi
        if ! is_installed_brew "icu4c"; then
            brew install icu4c
        fi
        if ! is_installed_brew "glm"; then
            brew install glm
        fi
        if ! is_installed_brew "libarchive"; then
            brew install libarchive
        fi
        if ! is_installed_brew "tinyxml2"; then
            brew install tinyxml2
        fi
        if ! is_installed_brew "zlib"; then
            brew install zlib
        fi
        if ! is_installed_brew "libpng"; then
            brew install libpng
        fi
        if ! is_installed_brew "sdl2"; then
            brew install sdl2
        fi
        if ! is_installed_brew "sdl2_image"; then
            brew install sdl2_image
        fi
        if ! is_installed_brew "glew"; then
            brew install glew
        fi
        if ! is_installed_brew "jsoncpp"; then
            brew install jsoncpp
        fi
        if ! is_installed_brew "eigen"; then
            brew install eigen
        fi
        if ! is_installed_brew "python@3.12"; then
            brew install python@3.12
        fi
        if ! is_installed_brew "curl"; then
            brew install curl
        fi
    fi
elif $use_macos && command -v brew &> /dev/null; then
    :
elif command -v apt &> /dev/null && ! $use_conda; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v patchelf &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        sudo apt install -y cmake git curl pkg-config build-essential patchelf
    fi

    # Dependencies of sgl and the application.
    if $use_vcpkg; then
        if ! is_installed_apt "libgl-dev" || ! is_installed_apt "libxmu-dev" || ! is_installed_apt "libxi-dev" \
                || ! is_installed_apt "libx11-dev" || ! is_installed_apt "libxft-dev" \
                || ! is_installed_apt "libxext-dev" || ! is_installed_apt "libxrandr-dev" \
                || ! is_installed_apt "libwayland-dev" || ! is_installed_apt "libxkbcommon-dev" \
                || ! is_installed_apt "libegl1-mesa-dev" || ! is_installed_apt "libibus-1.0-dev" \
                || ! is_installed_apt "autoconf" || ! is_installed_apt "automake" \
                || ! is_installed_apt "autoconf-archive"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo apt install -y libgl-dev libxmu-dev libxi-dev libx11-dev libxft-dev libxext-dev libxrandr-dev \
            libwayland-dev libxkbcommon-dev libegl1-mesa-dev libibus-1.0-dev autoconf automake autoconf-archive
        fi
    else
        if ! is_installed_apt "libboost-filesystem-dev" || ! is_installed_apt "libicu-dev" \
                || ! is_installed_apt "libglm-dev" || ! is_installed_apt "libarchive-dev" \
                || ! is_installed_apt "libtinyxml2-dev" || ! is_installed_apt "libpng-dev" \
                || ! is_installed_apt "libsdl2-dev" || ! is_installed_apt "libsdl2-image-dev" \
                || ! is_installed_apt "libglew-dev" || ! is_installed_apt "libjsoncpp-dev" \
                || ! is_installed_apt "libeigen3-dev" || ! is_installed_apt "python3-dev"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo apt install -y libboost-filesystem-dev libicu-dev libglm-dev libarchive-dev libtinyxml2-dev libpng-dev \
            libsdl2-dev libsdl2-image-dev libglew-dev libjsoncpp-dev libeigen3-dev python3-dev
        fi
        if ! (is_installed_apt "libcurl4-openssl-dev" || is_installed_apt "libcurl4-gnutls-dev" \
                || is_installed_apt "libcurl4-nss-dev"); then
            sudo apt install -y libcurl4-openssl-dev
        fi
    fi
elif command -v pacman &> /dev/null && ! $use_conda; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v patchelf &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        sudo pacman -S cmake git curl pkgconf base-devel patchelf
    fi

    # Dependencies of sgl and the application.
    if $use_vcpkg; then
        if ! is_installed_pacman "libgl" || ! is_installed_pacman "vulkan-devel" || ! is_installed_pacman "shaderc" \
                || ! is_installed_pacman "openssl" || ! is_installed_pacman "autoconf" \
                || ! is_installed_pacman "automake" || ! is_installed_pacman "autoconf-archive"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo pacman -S libgl vulkan-devel shaderc openssl autoconf automake autoconf-archive
        fi
    else
        if ! is_installed_pacman "boost" || ! is_installed_pacman "icu" || ! is_installed_pacman "glm" \
                || ! is_installed_pacman "libarchive" || ! is_installed_pacman "tinyxml2" \
                || ! is_installed_pacman "libpng" || ! is_installed_pacman "sdl2" || ! is_installed_pacman "sdl2_image" \
                || ! is_installed_pacman "glew" || ! is_installed_pacman "jsoncpp" || ! is_installed_pacman "eigen" \
                || ! is_installed_pacman "python3" || ! is_installed_pacman "curl" \
                || ! is_installed_pacman "embree"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo pacman -S boost icu glm libarchive tinyxml2 libpng sdl2 sdl2_image glew jsoncpp eigen python3 curl \
            embree
        fi
    fi
elif command -v yum &> /dev/null && ! $use_conda; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v patchelf &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        sudo yum install -y cmake git curl pkgconf gcc gcc-c++ patchelf
    fi

    # Dependencies of sgl and the application.
    if $use_vcpkg; then
        if ! is_installed_rpm "perl" || ! is_installed_rpm "libstdc++-devel" || ! is_installed_rpm "libstdc++-static" \
                || ! is_installed_rpm "autoconf" || ! is_installed_rpm "automake" \
                || ! is_installed_rpm "autoconf-archive" || ! is_installed_rpm "glew-devel" \
                || ! is_installed_rpm "libXext-devel" || ! is_installed_rpm "vulkan-headers" \
                || ! is_installed_rpm "vulkan-loader" || ! is_installed_rpm "vulkan-tools" \
                || ! is_installed_rpm "vulkan-validation-layers" || ! is_installed_rpm "libshaderc-devel"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo yum install -y perl libstdc++-devel libstdc++-static autoconf automake autoconf-archive glew-devel \
            libXext-devel vulkan-headers vulkan-loader vulkan-tools vulkan-validation-layers libshaderc-devel
        fi
    else
        if ! is_installed_rpm "boost-devel" || ! is_installed_rpm "libicu-devel" || ! is_installed_rpm "glm-devel" \
                || ! is_installed_rpm "libarchive-devel" || ! is_installed_rpm "tinyxml2-devel" \
                || ! is_installed_rpm "libpng-devel" || ! is_installed_rpm "SDL2-devel" \
                || ! is_installed_rpm "SDL2_image-devel" || ! is_installed_rpm "glew-devel" \
                || ! is_installed_rpm "jsoncpp-devel" || ! is_installed_rpm "eigen3-devel" \
                || ! is_installed_rpm "python3-devel" || ! is_installed_rpm "libcurl-devel"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo yum install -y boost-devel libicu-devel glm-devel libarchive-devel tinyxml2-devel libpng-devel \
            SDL2-devel SDL2_image-devel glew-devel jsoncpp-devel eigen3-devel python3-devel libcurl-devel
        fi
    fi
elif $use_conda && ! $use_macos; then
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/miniconda3/etc/profile.d/conda.sh" shell.bash hook
    elif [ -f "/opt/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/opt/anaconda3/etc/profile.d/conda.sh" shell.bash hook
    elif [ ! -z "${CONDA_PREFIX+x}" ]; then
        . "$CONDA_PREFIX/etc/profile.d/conda.sh" shell.bash hook
    fi

    if ! command -v conda &> /dev/null; then
        echo "------------------------"
        echo "  installing Miniconda  "
        echo "------------------------"
        if [ "$os_arch" = "x86_64" ]; then
            wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
            chmod +x Miniconda3-latest-Linux-x86_64.sh
            bash ./Miniconda3-latest-Linux-x86_64.sh
            . "$HOME/miniconda3/etc/profile.d/conda.sh" shell.bash hook
            rm ./Miniconda3-latest-Linux-x86_64.sh
        else
            wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
            chmod +x Miniconda3-latest-Linux-aarch64.sh
            bash ./Miniconda3-latest-Linux-aarch64.sh
            . "$HOME/miniconda3/etc/profile.d/conda.sh" shell.bash hook
            rm ./Miniconda3-latest-Linux-aarch64.sh
        fi
    fi

    if ! conda env list | grep ".*${conda_env_name}.*" >/dev/null 2>&1; then
        echo "------------------------"
        echo "creating conda environment"
        echo "------------------------"
        conda create -n "${conda_env_name}" -y
        conda init bash
        conda activate "${conda_env_name}"
    elif [ "${var+CONDA_DEFAULT_ENV}" != "${conda_env_name}" ]; then
        conda activate "${conda_env_name}"
    fi

    conda_pkg_list="$(conda list)"
    if ! list_contains "$conda_pkg_list" "boost" || ! list_contains "$conda_pkg_list" "conda-forge::icu" \
            || ! list_contains "$conda_pkg_list" "glm" || ! list_contains "$conda_pkg_list" "libarchive" \
            || ! list_contains "$conda_pkg_list" "tinyxml2" || ! list_contains "$conda_pkg_list" "libpng" \
            || ! list_contains "$conda_pkg_list" "sdl2" || ! list_contains "$conda_pkg_list" "sdl2" \
            || ! list_contains "$conda_pkg_list" "glew" || ! list_contains "$conda_pkg_list" "cxx-compiler" \
            || ! list_contains "$conda_pkg_list" "make" || ! list_contains "$conda_pkg_list" "cmake" \
            || ! list_contains "$conda_pkg_list" "pkg-config" || ! list_contains "$conda_pkg_list" "gdb" \
            || ! list_contains "$conda_pkg_list" "git" \
            || ! list_contains "$conda_pkg_list" "mesa-libgl-devel-cos7-x86_64" \
            || ! list_contains "$conda_pkg_list" "libglvnd-glx-cos7-x86_64" \
            || ! list_contains "$conda_pkg_list" "mesa-dri-drivers-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxau-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libselinux-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxdamage-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxxf86vm-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxext-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "xorg-libxfixes" || ! list_contains "$conda_pkg_list" "xorg-libxau" \
            || ! list_contains "$conda_pkg_list" "xorg-libxrandr" || ! list_contains "$conda_pkg_list" "patchelf" \
            || ! list_contains "$conda_pkg_list" "jsoncpp" || ! list_contains "$conda_pkg_list" "eigen" \
            || ! list_contains "$conda_pkg_list" "libcurl"; then
        echo "------------------------"
        echo "installing dependencies "
        echo "------------------------"
        conda install -y -c conda-forge boost conda-forge::icu glm libarchive tinyxml2 libpng sdl2 sdl2 glew \
        cxx-compiler make cmake pkg-config gdb git mesa-libgl-devel-cos7-x86_64 libglvnd-glx-cos7-x86_64 \
        mesa-dri-drivers-cos7-aarch64 libxau-devel-cos7-aarch64 libselinux-devel-cos7-aarch64 \
        libxdamage-devel-cos7-aarch64 libxxf86vm-devel-cos7-aarch64 libxext-devel-cos7-aarch64 xorg-libxfixes \
        xorg-libxau xorg-libxrandr patchelf jsoncpp eigen libcurl
    fi
else
    echo "Warning: Unsupported system package manager detected." >&2
fi

if ! command -v cmake &> /dev/null; then
    echo "CMake was not found, but is required to build the program."
    exit 1
fi
if ! command -v git &> /dev/null; then
    echo "git was not found, but is required to build the program."
    exit 1
fi
if ! command -v curl &> /dev/null; then
    echo "curl was not found, but is required to build the program."
    exit 1
fi
if [ $use_macos = false ] && ! command -v pkg-config &> /dev/null; then
    echo "pkg-config was not found, but is required to build the program."
    exit 1
fi


[ -d "./third_party/" ] || mkdir "./third_party/"
pushd third_party > /dev/null


params_sgl=()
params=()
params_run=()
params_gen=()

if [ $use_msys = true ]; then
    params_gen+=(-G "MSYS Makefiles")
    params_sgl+=(-G "MSYS Makefiles")
    params+=(-G "MSYS Makefiles")
fi

if [ $use_vcpkg = false ] && [ $use_macos = true ]; then
    params_gen+=(-DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=False)
    params_gen+=(-DCMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH=False)
    params_gen+=(-DCMAKE_FIND_FRAMEWORK=LAST)
    params_gen+=(-DCMAKE_FIND_APPBUNDLE=NEVER)
    params_gen+=(-DCMAKE_PREFIX_PATH="$(brew --prefix)")
    params_sgl+=(-DCMAKE_INSTALL_PREFIX="../install")
    params_sgl+=(-DZLIB_ROOT="$(brew --prefix)/opt/zlib")
    params+=(-DZLIB_ROOT="$(brew --prefix)/opt/zlib")
fi

if $glibcxx_debug; then
    params_sgl+=(-DUSE_GLIBCXX_DEBUG=On)
    params+=(-DUSE_GLIBCXX_DEBUG=On)
fi

use_vulkan=false
vulkan_sdk_env_set=true

if [ $use_vcpkg = true ] && [ ! -d "./vcpkg" ]; then
    echo "------------------------"
    echo "    fetching vcpkg      "
    echo "------------------------"
    if $use_vulkan && [ $vulkan_sdk_env_set = false ]; then
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process."
        exit 1
    fi
    git clone --depth 1 https://github.com/microsoft/vcpkg.git
    vcpkg/bootstrap-vcpkg.sh -disableMetrics
fi

if [ $use_vcpkg = true ] && [ $use_macos = false ] && [ $link_dynamic = false ]; then
    params_sgl+=(-DBUILD_STATIC_LIBRARY=On)
fi

if [ ! -d "./sgl" ]; then
    echo "------------------------"
    echo "     fetching sgl       "
    echo "------------------------"
    git clone --depth 1 https://github.com/chrismile/sgl.git
fi

if [ -f "./sgl/$build_dir/CMakeCache.txt" ]; then
    remove_build_cache_sgl=false
    if grep -q vcpkg_installed "./sgl/$build_dir/CMakeCache.txt"; then
        cache_uses_vcpkg=true
    else
        cache_uses_vcpkg=false
    fi
    if ([ $use_vcpkg = true ] && [ $cache_uses_vcpkg = false ]) || ([ $use_vcpkg = false ] && [ $cache_uses_vcpkg = true ]); then
        remove_build_cache_sgl=true
    fi
    if [ $remove_build_cache_sgl = true ]; then
        echo "Removing old sgl build cache..."
        if [ -d "./sgl/$build_dir_debug" ]; then
            rm -rf "./sgl/$build_dir_debug"
        fi
        if [ -d "./sgl/$build_dir_release" ]; then
            rm -rf "./sgl/$build_dir_release"
        fi
        if [ -d "./sgl/install" ]; then
            rm -rf "./sgl/install"
        fi
    fi
fi

if [ ! -d "./sgl/install" ]; then
    echo "------------------------"
    echo "     building sgl       "
    echo "------------------------"

    pushd "./sgl" >/dev/null
    if ! $build_sgl_release_only; then
        mkdir -p $build_dir_debug
    fi
    mkdir -p $build_dir_release

    if ! $build_sgl_release_only; then
        pushd "$build_dir_debug" >/dev/null
        cmake .. \
             -DCMAKE_BUILD_TYPE=Debug \
             -DCMAKE_INSTALL_PREFIX="../install" \
             ${params_gen[@]+"${params_gen[@]}"} ${params_link[@]+"${params_link[@]}"} \
             ${params_vcpkg[@]+"${params_vcpkg[@]}"} ${params_sgl[@]+"${params_sgl[@]}"}
        if [ $use_vcpkg = false ] && [ $use_macos = false ]; then
            make -j $(nproc)
            make install
        fi
        popd >/dev/null
    fi

    pushd $build_dir_release >/dev/null
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="../install" \
         ${params_gen[@]+"${params_gen[@]}"} ${params_link[@]+"${params_link[@]}"} \
         ${params_vcpkg[@]+"${params_vcpkg[@]}"} ${params_sgl[@]+"${params_sgl[@]}"}
    if [ $use_vcpkg = false ] && [ $use_macos = false ]; then
        make -j $(nproc)
        make install
    fi
    popd >/dev/null

    if [ $use_macos = true ]; then
        if ! $build_sgl_release_only; then
            cmake --build $build_dir_debug --parallel $(sysctl -n hw.ncpu)
            cmake --build $build_dir_debug --target install
        fi

        cmake --build $build_dir_release --parallel $(sysctl -n hw.ncpu)
        cmake --build $build_dir_release --target install
    elif [ $use_vcpkg = true ]; then
        if ! $build_sgl_release_only; then
            cmake --build $build_dir_debug --parallel $(nproc)
            cmake --build $build_dir_debug --target install
            if [ $link_dynamic = true ]; then
                cp $build_dir_debug/libsgld.so install/lib/libsgld.so
            fi
        fi

        cmake --build $build_dir_release --parallel $(nproc)
        cmake --build $build_dir_release --target install
        if [ $link_dynamic = true ]; then
            cp $build_dir_release/libsgl.so install/lib/libsgl.so
        fi
    fi

    popd >/dev/null
fi

embree_version="4.3.2"

if [ $use_macos = false ] && [ $use_msys = false ]; then
    if ! $is_embree_installed && [ $os_arch = "x86_64" ]; then
        if [ ! -d "./embree-${embree_version}.x86_64.linux" ]; then
            echo "------------------------"
            echo "   downloading Embree   "
            echo "------------------------"
            wget "https://github.com/embree/embree/releases/download/v${embree_version}/embree-${embree_version}.x86_64.linux.tar.gz"
            if [ $( tar --exclude='*/*' -tvf "embree-${embree_version}.x86_64.linux.tar.gz" | wc -l) -gt 1 ]; then
                mkdir -p "embree-${embree_version}.x86_64.linux"
                tar -xvzf "embree-${embree_version}.x86_64.linux.tar.gz" -C "embree-${embree_version}.x86_64.linux"
            else
                # For older Embree versions.
                tar -xvzf "embree-${embree_version}.x86_64.linux.tar.gz"
            fi
        fi
        if [[ $embree_version == 3* ]]; then
            embree_lib_name="embree3"
        elif [[ $embree_version == 4* ]]; then
            embree_lib_name="embree4"
        else
            echo "Unsupported Embree version $embree_version detected."
            exit 1
        fi
        params+=(-Dembree_DIR="${projectpath}/third_party/embree-${embree_version}.x86_64.linux/lib/cmake/embree-${embree_version}")
    fi
#elif [ $use_macos = true ]; then
#    if [ ! -d "./embree/embree/lib/cmake" ]; then
#        # Make sure we have no leftovers from a failed build attempt.
#        if [ -d "./embree-repo" ]; then
#            rm -rf "./embree-repo"
#        fi
#        if [ -d "./embree-build" ]; then
#            rm -rf "./embree-build"
#        fi
#        if [ -d "./embree" ]; then
#            rm -rf "./embree"
#        fi
#
#        params_embree=()
#        if [[ $(uname -m) == 'arm64' ]]; then
#            params_embree+=(-DBUILD_TBB_FROM_SOURCE=On)
#        fi
#
#        # Build Embree and its dependencies.
#        git clone https://github.com/embree/embree.git embree-repo
#        mkdir embree-build
#        pushd "./embree-build" >/dev/null
#        cmake ../embree-repo/scripts/superbuild -DCMAKE_INSTALL_PREFIX="$projectpath/third_party/embree" \
#        -DBUILD_JOBS=$(sysctl -n hw.ncpu) ${params_embree[@]+"${params_embree[@]}"}
#        cmake --build . --parallel $(sysctl -n hw.ncpu)
#        cmake --build . --parallel $(sysctl -n hw.ncpu)
#        popd >/dev/null
#    fi
#
#    params+=(-Dembree_DIR="${projectpath}/third_party/embree/embree/lib/cmake/$(ls "${projectpath}/third_party/embree/embree/lib/cmake")")
fi
if [ $use_vcpkg = true ]; then
    params+=(-DPYTHONHOME="./python3")
fi

if $use_msys; then
    Python3_VERSION="$(find "$MSYSTEM_PREFIX/lib/" -maxdepth 1 -type d -name 'python*' -printf "%f" -quit)"
    params+=(-DPython3_FIND_REGISTRY=NEVER -DPYTHONHOME="./python3" -DPYTHONPATH="./python3/lib/$Python3_VERSION")
fi

popd >/dev/null # back to project root

if [ $debug = true ]; then
    echo "------------------------"
    echo "  building in debug     "
    echo "------------------------"
else
    echo "------------------------"
    echo "  building in release   "
    echo "------------------------"
fi

if [ -f "./$build_dir/CMakeCache.txt" ]; then
    remove_build_cache=false
    if grep -q vcpkg_installed "./$build_dir/CMakeCache.txt"; then
        cache_uses_vcpkg=true
    else
        cache_uses_vcpkg=false
    fi
    if ([ $use_vcpkg = true ] && [ $cache_uses_vcpkg = false ]) || ([ $use_vcpkg = false ] && [ $cache_uses_vcpkg = true ]); then
        remove_build_cache=true
    fi
    if [ remove_build_cache = true ]; then
        echo "Removing old application build cache..."
        if [ -d "./$build_dir_debug" ]; then
            rm -rf "./$build_dir_debug"
        fi
        if [ -d "./$build_dir_release" ]; then
            rm -rf "./$build_dir_release"
        fi
        if [ -d "./$destination_dir" ]; then
            rm -rf "./$destination_dir"
        fi
    fi
fi

mkdir -p $build_dir

echo "------------------------"
echo "      generating        "
echo "------------------------"
pushd $build_dir >/dev/null
cmake .. \
    -DCMAKE_BUILD_TYPE=$cmake_config \
    -Dsgl_DIR="$projectpath/third_party/sgl/install/lib/cmake/sgl/" \
    ${params_gen[@]+"${params_gen[@]}"} ${params_link[@]+"${params_link[@]}"} \
    ${params_vcpkg[@]+"${params_vcpkg[@]}"} ${params[@]+"${params[@]}"}
if [ $use_vcpkg = true ] || [ $use_msys = true ] || [ $use_macos = true ]; then
    Python3_VERSION=$(cat pythonversion.txt)
fi
popd >/dev/null

echo "------------------------"
echo "      compiling         "
echo "------------------------"
if [ $use_macos = true ]; then
    cmake --build $build_dir --parallel $(sysctl -n hw.ncpu)
elif [ $use_vcpkg = true ]; then
    cmake --build $build_dir --parallel $(nproc)
else
    pushd "$build_dir" >/dev/null
    make -j $(nproc)
    popd >/dev/null
fi

echo "------------------------"
echo "   copying new files    "
echo "------------------------"

# https://stackoverflow.com/questions/2829613/how-do-you-tell-if-a-string-contains-another-string-in-posix-sh
contains() {
    string="$1"
    substring="$2"
    if test "${string#*$substring}" != "$string"
    then
        return 0
    else
        return 1
    fi
}
startswith() {
    string="$1"
    prefix="$2"
    if test "${string#$prefix}" != "$string"
    then
        return 0
    else
        return 1
    fi
}

if $use_msys; then
    if [[ -z "${PATH+x}" ]]; then
        export PATH="${projectpath}/third_party/sgl/install/bin"
    elif [[ ! "${PATH}" == *"${projectpath}/third_party/sgl/install/bin"* ]]; then
        export PATH="${projectpath}/third_party/sgl/install/bin:$PATH"
    fi
elif $use_macos; then
    if [ -z "${DYLD_LIBRARY_PATH+x}" ]; then
        export DYLD_LIBRARY_PATH="${projectpath}/third_party/sgl/install/lib"
    elif contains "${DYLD_LIBRARY_PATH}" "${projectpath}/third_party/sgl/install/lib"; then
        export DYLD_LIBRARY_PATH="DYLD_LIBRARY_PATH:${projectpath}/third_party/sgl/install/lib"
    fi
else
  if [[ -z "${LD_LIBRARY_PATH+x}" ]]; then
      export LD_LIBRARY_PATH="${projectpath}/third_party/sgl/install/lib"
  elif [[ ! "${LD_LIBRARY_PATH}" == *"${projectpath}/third_party/sgl/install/lib"* ]]; then
      export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${projectpath}/third_party/sgl/install/lib"
  fi
fi
if [ $use_macos = false ] && [ $use_msys = false ]; then
    if ! $is_embree_installed && [ $os_arch = "x86_64" ]; then
        export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${projectpath}/third_party/embree-${embree_version}.x86_64.linux/lib"
    fi
fi
if [ $use_macos = true ] && [ $use_vcpkg = true ]; then
    export PYTHONHOME="$PYTHONHOME_global"
elif [ $use_macos = true ] && [ $use_vcpkg = false ]; then
    export PYTHONHOME="../$destination_dir/python3"
elif $use_msys; then
    export PYTHONHOME="/mingw64"
else
    if [ $use_vcpkg = true ]; then
        export PYTHONHOME="../Shipping/bin/python3"
    fi
fi

if $use_msys; then
    mkdir -p $destination_dir/bin

    # Copy sgl to the destination directory.
    if [ $debug = true ] ; then
        cp "./third_party/sgl/install/bin/libsgld.dll" "$destination_dir/bin"
    else
        cp "./third_party/sgl/install/bin/libsgl.dll" "$destination_dir/bin"
    fi

    # Copy the application to the destination directory.
    cp "$build_dir/HexVolumeRenderer.exe" "$destination_dir/bin"

    # Copy all dependencies of the application to the destination directory.
    ldd_output="$(ntldd -R $build_dir/HexVolumeRenderer.exe)"
    for library_abs in $ldd_output
    do
        if [[ $library_abs == "not found"* ]] || [[ $library_abs == "ext-ms-win"* ]] || [[ $library_abs == "=>" ]] \
                || [[ $library_abs == "(0x"* ]] || [[ $library_abs == "C:\\WINDOWS"* ]] \
                || [[ $library_abs == "not" ]] || [[ $library_abs == "found"* ]]; then
            continue
        fi
        library="$(cygpath "$library_abs")"
        if [[ $library == "$MSYSTEM_PREFIX"* ]] || [[ $library == "$projectpath"* ]];
        then
            cp "$library" "$destination_dir/bin"
        fi
        if [[ $library == libpython* ]];
        then
            tmp=${library#*lib}
            Python3_VERSION=${tmp%.dll}
        fi
    done
elif [ $use_macos = true ] && [ $use_vcpkg = true ]; then
    [ -d $destination_dir ] || mkdir $destination_dir
    rsync -a "$build_dir/HexVolumeRenderer.app/Contents/MacOS/HexVolumeRenderer" $destination_dir
elif [ $use_macos = true ] && [ $use_vcpkg = false ]; then
    brew_prefix="$(brew --prefix)"
    mkdir -p $destination_dir

    if [ -d "$destination_dir/HexVolumeRenderer.app" ]; then
        rm -rf "$destination_dir/HexVolumeRenderer.app"
    fi

    # Copy the application to the destination directory.
    cp -a "$build_dir/HexVolumeRenderer.app" "$destination_dir"

    # Copy sgl to the destination directory.
    if [ $debug = true ] ; then
        cp "./third_party/sgl/install/lib/libsgld.dylib" "$binaries_dest_dir"
    else
        cp "./third_party/sgl/install/lib/libsgl.dylib" "$binaries_dest_dir"
    fi

    # Copy all dependencies of the application and sgl to the destination directory.
    rsync -a "$VULKAN_SDK/lib/libMoltenVK.dylib" "$binaries_dest_dir"
    copy_dependencies_recursive() {
        local binary_path="$1"
        local binary_source_folder=$(dirname "$binary_path")
        local binary_name=$(basename "$binary_path")
        local binary_target_path="$binaries_dest_dir/$binary_name"
        if contains "$(file "$binary_target_path")" "dynamically linked shared library"; then
            install_name_tool -id "@executable_path/$binary_name" "$binary_target_path" &> /dev/null
        fi
        local otool_output="$(otool -L "$binary_path")"
        local otool_output=${otool_output#*$'\n'}
        while read -r line
        do
            local stringarray=($line)
            local library=${stringarray[0]}
            local library_name=$(basename "$library")
            local library_target_path="$binaries_dest_dir/$library_name"
            if ! startswith "$library" "@rpath/" \
                && ! startswith "$library" "@loader_path/" \
                && ! startswith "$library" "/System/Library/Frameworks/" \
                && ! startswith "$library" "/usr/lib/"
            then
                install_name_tool -change "$library" "@executable_path/$library_name" "$binary_target_path" &> /dev/null

                if [ ! -f "$library_target_path" ]; then
                    cp "$library" "$binaries_dest_dir"
                    copy_dependencies_recursive "$library"
                fi
            elif startswith "$library" "@rpath/"; then
                install_name_tool -change "$library" "@executable_path/$library_name" "$binary_target_path" &> /dev/null

                local rpath_grep_string="$(otool -l "$binary_target_path" | grep RPATH -A2)"
                local counter=0
                while read -r grep_rpath_line
                do
                    if [ $(( counter % 4 )) -eq 2 ]; then
                        local stringarray_grep_rpath_line=($grep_rpath_line)
                        local rpath=${stringarray_grep_rpath_line[1]}
                        if startswith "$rpath" "@loader_path"; then
                            rpath="${rpath/@loader_path/$binary_source_folder}"
                        fi
                        local library_rpath="${rpath}${library#"@rpath"}"

                        if [ -f "$library_rpath" ]; then
                            if [ ! -f "$library_target_path" ]; then
                                cp "$library_rpath" "$binaries_dest_dir"
                                copy_dependencies_recursive "$library_rpath"
                            fi
                            break
                        fi
                    fi
                    counter=$((counter + 1))
                done < <(echo "$rpath_grep_string")
            fi
        done < <(echo "$otool_output")
    }
    copy_dependencies_recursive "$build_dir/HexVolumeRenderer.app/Contents/MacOS/HexVolumeRenderer"
    if [ $debug = true ]; then
        copy_dependencies_recursive "./third_party/sgl/install/lib/libsgld.dylib"
    else
        copy_dependencies_recursive "./third_party/sgl/install/lib/libsgl.dylib"
    fi

    # Fix code signing for arm64.
    for filename in $binaries_dest_dir/*
    do
        if contains "$(file "$filename")" "arm64"; then
            codesign --force -s - "$filename" &> /dev/null
        fi
    done
else
    mkdir -p $destination_dir/bin

    # Copy the application to the destination directory.
    rsync -a "$build_dir/HexVolumeRenderer" "$destination_dir/bin"

    # Copy all dependencies of the application to the destination directory.
    ldd_output="$(ldd $build_dir/HexVolumeRenderer)"

    if ! $is_embree_installed && [ $os_arch = "x86_64" ]; then
        libembree3_so="$(readlink -f "${projectpath}/third_party/embree-${embree_version}.x86_64.linux/lib/lib${embree_lib_name}.so")"
        ldd_output="$ldd_output $libembree3_so"
    fi
    library_blacklist=(
        "libOpenGL" "libGLdispatch" "libGL.so" "libGLX.so"
        "libwayland" "libffi." "libX" "libxcb" "libxkbcommon"
        "ld-linux" "libdl." "libutil." "libm." "libc." "libpthread." "libbsd." "librt."
    )
    if [ $use_vcpkg = true ]; then
        # We build with libstdc++.so and libgcc_s.so statically. If we were to ship them, libraries opened with dlopen will
        # use our, potentially older, versions. Then, we will get errors like "version `GLIBCXX_3.4.29' not found" when
        # the Vulkan loader attempts to load a Vulkan driver that was built with a never version of libstdc++.so.
        # I tried to solve this by using "patchelf --replace-needed" to directly link to the patch version of libstdc++.so,
        # but that made no difference whatsoever for dlopen.
        library_blacklist+=("libstdc++.so")
        library_blacklist+=("libgcc_s.so")
    fi
    for library in $ldd_output
    do
        if [[ $library != "/"* ]]; then
            continue
        fi
        is_blacklisted=false
        for blacklisted_library in ${library_blacklist[@]+"${library_blacklist[@]}"}; do
            if [[ "$library" == *"$blacklisted_library"* ]]; then
                is_blacklisted=true
                break
            fi
        done
        if [ $is_blacklisted = true ]; then
            continue
        fi
        # TODO: Add blacklist entries for pulseaudio and dependencies when not using vcpkg.
        #cp "$library" "$destination_dir/bin"
        #patchelf --set-rpath '$ORIGIN' "$destination_dir/bin/$(basename "$library")"
        if [ $use_vcpkg = true ]; then
            cp "$library" "$destination_dir/bin"
            patchelf --set-rpath '$ORIGIN' "$destination_dir/bin/$(basename "$library")"
        fi
    done
    patchelf --set-rpath '$ORIGIN' "$destination_dir/bin/HexVolumeRenderer"
    if ! $is_embree_installed; then
        ln -sf "./$(basename "$libembree3_so")" "$destination_dir/bin/lib${embree_lib_name}.so"
    fi
fi

# 2023-11-11: It seems like for LineVis, vcpkg_installed is in the root directory, but for HexVolumeRenderer
# it is in the build folder.
if [ $use_vcpkg = true ]; then
    if [ -d "vcpkg_installed" ]; then
        vcpkg_installed_dir="vcpkg_installed"
    elif [ -d "$build_dir/vcpkg_installed" ]; then
        vcpkg_installed_dir="$build_dir/vcpkg_installed"
    fi
    if [ $use_macos = true ]; then
        vcpkg_installed_dir_triplet="vcpkg_installed/$(ls $vcpkg_installed_dir | grep -Ewv 'vcpkg')"
    else
        if [ $use_custom_vcpkg_triplet = true ]; then
            vcpkg_installed_dir_triplet="$vcpkg_installed_dir/$vcpkg_triplet"
        else
            vcpkg_installed_dir_triplet="$vcpkg_installed_dir/$(ls --ignore=vcpkg $vcpkg_installed_dir)"
        fi
    fi
fi
# Copy python3 to the destination directory.
if $use_msys; then
    if [ ! -d "$destination_dir/bin/python3" ]; then
        mkdir -p "$destination_dir/bin/python3/lib"
        #cp -r "$MSYSTEM_PREFIX/lib/$Python3_VERSION" "$destination_dir/bin/python3/lib"
        rsync -qav "$MSYSTEM_PREFIX/lib/$Python3_VERSION" "$destination_dir/bin/python3/lib" --exclude site-packages --exclude dist-packages
    fi
elif [ $use_macos = true ] && [ $use_vcpkg = true ]; then
    [ -d $destination_dir/python3 ]     || mkdir $destination_dir/python3
    [ -d $destination_dir/python3/lib ] || mkdir $destination_dir/python3/lib
    rsync -a "$vcpkg_installed_dir_triplet/lib/$Python3_VERSION" $destination_dir/python3/lib
    #rsync -a "$(eval echo "$vcpkg_installed_dir_triplet/lib/python*")" $destination_dir/python3/lib
elif [ $use_macos = true ] && [ $use_vcpkg = false ]; then
    python_version=${Python3_VERSION#python}
    python_subdir="$brew_prefix/Cellar/python@${Python3_VERSION#python}"
    PYTHONHOME_global="$python_subdir/$(ls "$python_subdir")/Frameworks/Python.framework/Versions/$python_version"
    if [ ! -d "$binaries_dest_dir/python3" ]; then
        mkdir -p "$binaries_dest_dir/python3/lib"
        rsync -a "$PYTHONHOME_global/lib/$Python3_VERSION" "$binaries_dest_dir/python3/lib"
        rsync -a "$brew_prefix/lib/$Python3_VERSION" "$binaries_dest_dir/python3/lib"
        #rsync -a "$(eval echo "$brew_prefix/lib/python*")" $binaries_dest_dir/python3/lib
    fi
elif [ $use_vcpkg = true ]; then
    [ -d $destination_dir/bin/python3 ]     || mkdir $destination_dir/bin/python3
    [ -d $destination_dir/bin/python3/lib ] || mkdir $destination_dir/bin/python3/lib
    python_lib_dir="$vcpkg_installed_dir_triplet/lib/$Python3_VERSION"
    rsync -a "$python_lib_dir" $destination_dir/bin/python3/lib
    #rsync -a "$(eval echo "$vcpkg_installed_dir_triplet/lib/python*")" $destination_dir/python3/lib
fi

# Copy the docs to the destination directory.
cp "README.md" "$destination_dir"
if [ ! -d "$destination_dir/LICENSE" ]; then
    mkdir -p "$destination_dir/LICENSE"
    cp -r "docs/license-libraries/." "$destination_dir/LICENSE/"
    cp -r "LICENSE" "$destination_dir/LICENSE/LICENSE-hexvolumerenderer.txt"
fi
if [ ! -d "$destination_dir/docs" ]; then
    cp -r "docs" "$destination_dir"
fi

# Create a run script.
if $use_msys; then
    printf "@echo off\npushd %%~dp0\npushd bin\nstart \"\" HexVolumeRenderer.exe\n" > "$destination_dir/run.bat"
elif $use_macos; then
    printf "#!/bin/sh\npushd \"\$(dirname \"\$0\")\" >/dev/null\n./HexVolumeRenderer.app/Contents/MacOS/HexVolumeRenderer\npopd\n" > "$destination_dir/run.sh"
    chmod +x "$destination_dir/run.sh"
else
    printf "#!/bin/bash\npushd \"\$(dirname \"\$0\")/bin\" >/dev/null\n./HexVolumeRenderer\npopd\n" > "$destination_dir/run.sh"
    chmod +x "$destination_dir/run.sh"
fi

# Replicability Stamp mode.
if [ $replicability = true ]; then
    params_run+=(--replicability)
fi
# Download the data sets.
if [ $replicability = true ] && [ ! -d "Data/Meshes/2019 - Symmetric Moving Frames" ]; then
    # This tool can help download data sets from HexaLab.
    # The program assumes the user has the rights to download the data.
    #
    # HexaLab.net: an online viewer for hexahedral meshes
    # Matteo Bracci, Marco Tarini, Nico Pietroni, Marco Livesu, Paolo Cignoni
    # Computer-Aided Design, Volume 110, May 2019
    # DOI:10.1016/j.cad.2018.12.003
    # (preprint available on arxiv)
    # Copyright 2018 Visual Computing Lab ISTI - CNR
    echo "------------------------"
    echo "Downloading HexaLab data sets..."
    echo "------------------------"
    build/HexaLabDatasetsDownloader
fi


# Run the program as the last step.
echo ""
echo "All done!"
pushd $build_dir >/dev/null

if [ $run_program = true ] && [ $use_macos = false ]; then
    ./HexVolumeRenderer ${params_run[@]+"${params_run[@]}"}
elif [ $run_program = true ] && [ $use_macos = true ]; then
    #open ./HexVolumeRenderer.app
    #open ./HexVolumeRenderer.app --args --perf
    ./HexVolumeRenderer.app/Contents/MacOS/HexVolumeRenderer ${params_run[@]+"${params_run[@]}"}
fi
