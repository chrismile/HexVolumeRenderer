# HexVolumeRenderer
A program for rendering hexahedral meshes in the form of transparent volumes.

This repository was created for the paper

Interactive Focus+Context Rendering for Volumetric Mesh Inspection. \
Christoph Neuhauser, Junpeng Wang, Rüdiger Westermann. \
to appear in IEEE Transactions on Visualization and Computer Graphics 2021 (TVCG 2021).

DOI: 10.1109/TVCG.2021.3074607

The rendering mode discussed in the paper can be used when selecting the renderer "ClearView (Unified)" in the program.
The LOD structure discussed can be viewed with the renderer "LOD Lines (Preview, Sheets)".

## Building and running the programm on Linux

A dependency of the program is the library sgl (https://github.com/chrismile/sgl).
It needs to be installed somewhere on the system. If it is not found by CMake, use `-Dsgl_DIR=<path-to-cmake-config>`.
On Ubuntu 20.04 for example, you can install all other necessary dependencies with this command (additionally to the
prerequisites required by sgl):

```
sudo apt-get install libjsoncpp-dev libeigen3-dev python3-dev libcurl4-openssl-dev
OR:
sudo apt-get install libjsoncpp-dev libeigen3-dev python3-dev libcurl4-gnutls-dev
```

On Ubuntu 18.04:

```
sudo apt-get install libjsoncpp-dev libcurl-dev libeigen3-dev python3-dev liblemon-dev
```

Lemon is an optional dependency. After installing sgl (see above), execute in the repository directory:

```
mkdir build
cd build
cmake ..
make -j
```

To run the program, execute:

```
export LD_LIBRARY_PATH=/usr/local/lib
./HexVolumeRenderer
```

The program is also able to use Embree instead of NanoRT (which is shipped together with this application) for ray-mesh
intersection tests. Embree is considerably faster. To enable Embree support, please use the following command.

```
cmake -DUSE_EMBREE=ON -Dembree_DIR=<path-to-embree> ..
```

The user needs to manually download Embree from its website and make sure that all its dependencies (like TBB and ISPC)
are also installed. If TBB and ISPC are not found and installed manually, please use the following command.

```
cmake -DUSE_EMBREE=ON -DISPC_EXECUTABLE=<path-to-ispc-binary> -DTBB_ROOT=<path-to-tbb> -Dembree_DIR=<path-to-embree> ..
```

## Building and running the program on Windows

The build process was also tested on Windows 10 64-bit using MSYS2 and Mingw-w64 (http://www.msys2.org/).
Using MSYS2 and Pacman, the following packages need to be installed additionally to the prerequisites required by sgl.

```
pacman -S mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-curl mingw64/mingw-w64-x86_64-eigen3 mingw64/mingw-w64-x86_64-python mingw64/mingw-w64-x86_64-embree
```

Furthermore, the graph library LEMON (http://lemon.cs.elte.hu/trac/lemon), which is an optional dependency, needs to be
built manually if needed, as no msys2 package is available for it at the time of writing this README file.

On Windows, using MSYS2 and Mingw-w64 (http://www.msys2.org/), it is best to use the following CMake command to
configure CMake:

```
cmake -G "MSYS Makefiles" -DPython3_FIND_REGISTRY=NEVER ..
```

To run the program, execute:

```
export PYTHONHOME="/mingw64"
./HexVolumeRenderer
```

### How to run with CLion on Windows

In order to run the program with CLion on Windows in conjunction with msys2, adding a MinGW toolchain is necessary.
For this, select the following options when creating a new toolchain in the settings (assuming msys2 is installed in
the standard directory C:\msys64).
- Environment: C:\msys64\mingw64
- CMake: C:\msys64\mingw64\bin\cmake.exe
- Make: C:\msys64\usr\bin\make.exe
- C Compiler: C:\msys64\mingw64\bin\gcc.exe
- C++ Compiler: C:\msys64\mingw64\bin\g++.exe
- Debugger: C:\msys64\mingw64\bin\gdb.exe

Then, add the following CMake options to the used build profiles.

```
-G "MSYS Makefiles" -DPython3_FIND_REGISTRY=NEVER -Dsgl_DIR=/usr/local/lib/cmake/sgl
```

Finally, in order for Windows to find the .dll files at runtime, add the following environment variables to the run
configurations.

```
PYTHONHOME=C:/msys64/mingw64;PATH=C:/msys64/mingw64/bin\;C:/msys64/usr/local/bin
```


## How to add new data sets

This program optionally supports downloading data sets from HexaLab. The user is responsible himself or herself that he or she has the rights to do this. For more details on HexaLab see:

[HexaLab.net](http://www.hexalab.net): an online viewer for hexahedral meshes

[Matteo Bracci](https://github.com/c4stan)<sup>x</sup>, [Marco Tarini](http://vcg.isti.cnr.it/~tarini/)<sup>1,2,x</sup>, [Nico Pietroni](http://vcg.isti.cnr.it/~pietroni)<sup>1,4</sup>, [Marco Livesu](http://pers.ge.imati.cnr.it/livesu/)<sup>3</sup>, [Paolo Cignoni](http://vcg.isti.cnr.it/~cignoni)<sup>1</sup>

[Computer-Aided Design, Volume 110, May 2019](https://doi.org/10.1016/j.cad.2018.12.003)

[DOI:10.1016/j.cad.2018.12.003](https://doi.org/10.1016/j.cad.2018.12.003)

(_[preprint](https://arxiv.org/pdf/1806.06639) available on [arxiv](https://arxiv.org/abs/1806.06639)_)

Copyright 2018
[Visual Computing Lab](http://vcg.isti.cnr.it)
[ISTI](http://www.isti.cnr.it) - [CNR](http://www.cnr.it)

Live view on [www.hexalab.net](http://www.hexalab.net)
- <sup>x</sup> Joint first authors
- <sup>1</sup> [ISTI](http://www.isti.cnr.it) - [CNR](http://www.cnr.it)
- <sup>2</sup> [Universit? degli Studi di Milano ("La Statale")](http://www.unimi.it)
- <sup>3</sup> [IMATI](http://www.imati.cnr.it/) - [CNR](http://www.cnr.it)
- <sup>4</sup> [University of Technology, Sidney](https://www.uts.edu.au/)

Furthermore, the user can edit the index.json file in the directory Data/Meshes/ to add meshes manually.
Please note that the file is overwritten when using the downloading functionality.

## External Code, Assets and Data Sets

The program uses code excerpts from HexaLab (https://github.com/chrismile/HexaLab, see above) and code from Robust
Hexahedral Re-Meshing (https://github.com/gaoxifeng/Robust-Hexahedral-Re-Meshing).

"Robust Structure Simplification for Hex Re-meshing",
Xifeng Gao, Daniele Panozzo, Wenping Wang, Zhigang Deng, Guoning Chen,
In ACM Transactions on Graphics (Proceedings of SIGGRAPH ASIA 2017)

The code from HexaLab is covered by the MIT License (see LICENSE-HexaLab).
The code from Gao et al. is covered by the MPL 2.0 License (see License-BaseComplex).
The code of the application itself is covered by the BSD 2-Clause License (see LICENSE).
The font Droid Sans in the directory Data/Fonts/ is covered by the Apache Version 2.0 License (see DROID-SANS-LICENSE.txt).

Please note that different licenses apply to the data sets, dependencies and other assets downloaded.
License information of used libraries can be found in the directory 'LICENSE-Libraries'. These libraries are used
without modifications to the upstream version of the code of the respective libraries.
Thus, the code is not provided in the repository.


## Replicability

For more details on the TVCG Replicability Stamp, please refer to `replicability/README.md`.

