# HexVolumeRenderer
A program for rendering hexahedral meshes in the form of transparent volumes.

This repository was created for the paper "Interactive Focus+Context Rendering for Volumetric Mesh Inspection", Christoph Neuhauser, Junpeng Wang, Rüdiger Westermann (to be published).

The rendering mode discussed in the paper can be used when selecting the renderer "ClearView (Unified)" in the program.
The LOD structure discussed can be viewed with the renderer "LOD Lines (Preview, Sheets)".

## Building and running the programm

The program requires the library sgl (https://github.com/chrismile/sgl).
On Ubuntu 20.04 for example, you can install all other necessary dependencies with this command (additionally to the prerequisites required by sgl):

```
sudo apt-get install libjsoncpp-dev libcurl-dev libeigen3-dev liblemon-dev libgegl-dev
```

Lemon and GEGL are optional dependencies. After installing sgl (see above) execute in the repository directory:

```
mkdir build
cd build
cmake ..
make -j
ln -s ../Data .
```
(Alternatively, use 'cp -R ../Data .' to copy the Data directory instead of creating a soft link to it).

The build process was also tested on Windows 10 64-bit using MSYS2 and Mingw-w64 (http://www.msys2.org/). Using MSYS2 and Pacman, the following packages need to be installed additionally to the prerequisites required by sgl.

```
pacman -S mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-curl mingw64/mingw-w64-x86_64-eigen3 mingw64/mingw-w64-x86_64-embree mingw64/mingw-w64-x86_64-gegl
```

Furthermore, the graph library LEMON (http://lemon.cs.elte.hu/trac/lemon) needs to be built manually, as no msys2 package is available for it at the time of writing this README file.

On Windows, using MSYS2 and Mingw-w64 (http://www.msys2.org/), it is best to use the following CMake command to configure CMake:
```
cmake .. -G"MSYS Makefiles"
```

To run the program, execute:
```
export LD_LIBRARY_PATH=/usr/local/lib
./HexVolumeRenderer
```

The program is also able to use Embree instead of NanoRT (which is shipped together with this application) for ray-mesh intersection tests.
Embree is considerably faster. To enable Embree support, please use the following command.

```
cmake -DUSE_EMBREE=ON -Dembree_DIR=<path-to-embree> ..
```

The user needs to manually download Embree from its website and make sure that all its dependencies (like TBB and ISPC) are also installed.

If TBB and ISPC are not found and installed manually, please use the following command.

```
cmake -DUSE_EMBREE=ON -DISPC_EXECUTABLE=<path-to-ispc-binary> -DTBB_ROOT=<path-to-tbb> -Dembree_DIR=<path-to-embree> ..
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


## Optional CSG Code

The program can make use of CSG libraries to compute the union of tube meshes for optimal line rendering.
Please note, however, that this requires a very slow preprocess step. At the moment, only the CSG library Cork
(https://github.com/gilbo/cork) is supported. It is licensed under the LGPL license. For more details on the license,
please refer to 'LICENSE-Libraries/LICENSE-Cork'. To enable Cork support, run

```
cmake .. -DUSE_CORK=ON -DCORK_LIBRARIES=/home/christoph/Programming/HiwiCG/cork/lib/libcork.so -DCORK_INCLUDE_DIR=/home/christoph/Programming/HiwiCG/cork/include
```

Please note that compiling Cork requires `libgmp-dev` and that you need to dynamically link to it to fulfill the
licensing terms of the LGPL. By default, Cork builds a static library.
Make sure to edit the Makefile and add the `-fPIC -shared` flags for compiling and linking, respectively.
