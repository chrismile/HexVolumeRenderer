# HexVolumeRenderer

A program for rendering hexahedral meshes in the form of transparent volumes.

This repository was created for the paper

Interactive Focus+Context Rendering for Hexahedral Mesh Inspection. \
Christoph Neuhauser, Junpeng Wang, Rüdiger Westermann. \
to appear in IEEE Transactions on Visualization and Computer Graphics 2021 (TVCG 2021). \
DOI: 10.1109/TVCG.2021.3074607

The rendering mode discussed in the paper can be used when selecting the renderer "ClearView (Unified)" in the program.
The LOD structure discussed can be viewed with the renderer "LOD Lines (Preview, Sheets)".


## Building and running the programm

Currently, there are three main ways to compile the program:
- Linux: Using the system package manager to install all dependencies (tested: apt on Ubuntu, pacman on Arch Linux).
- Linux & Windows: Installing all dependencies using [vcpkg](https://github.com/microsoft/vcpkg).
- Windows: Using MSYS2 to install all dependencies.

A build script `build.sh` is available in the project root directory that builds the application using the system
package manager on Linux and MSYS2 on Windows. Please download and install MSYS2 from https://www.msys2.org/ if you wish
to use this build script and run the script from an MSYS2 shell.
Alternatively, we recommend to use vcpkg if the users wants to compile the application with Microsoft Visual Studio.
A build script using MSVC and vcpkg, `build-msvc.bat`, is available in the project root directory.
Guides for manual compilation for the different build types can be found in the directory `docs/compilation`.


## How to add new data sets

This program optionally supports downloading data sets from HexaLab. The user is responsible himself or herself that he or she has the rights to do this. For more details on HexaLab see:

[HexaLab.net](http://www.hexalab.net): an online viewer for hexahedral meshes \
[Matteo Bracci](https://github.com/c4stan)<sup>x</sup>, [Marco Tarini](http://vcg.isti.cnr.it/~tarini/)<sup>1,2,x</sup>, [Nico Pietroni](http://vcg.isti.cnr.it/~pietroni)<sup>1,4</sup>, [Marco Livesu](http://pers.ge.imati.cnr.it/livesu/)<sup>3</sup>, [Paolo Cignoni](http://vcg.isti.cnr.it/~cignoni)<sup>1</sup> \
[Computer-Aided Design, Volume 110, May 2019](https://doi.org/10.1016/j.cad.2018.12.003) \
[DOI:10.1016/j.cad.2018.12.003](https://doi.org/10.1016/j.cad.2018.12.003) \
(_[preprint](https://arxiv.org/pdf/1806.06639) available on [arxiv](https://arxiv.org/abs/1806.06639)_) \
Copyright 2018
[Visual Computing Lab](http://vcg.isti.cnr.it)
[ISTI](http://www.isti.cnr.it) - [CNR](http://www.cnr.it) \
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

The code from HexaLab is covered by the MIT License (see LICENSE file).
The code from Gao et al. is covered by the MPL 2.0 License (see LICENSE file).
The code of the application itself is covered by the BSD 2-Clause License (see LICENSE file).
The font Droid Sans in the directory Data/Fonts/ is covered by the Apache Version 2.0 License (see LICENSE file).

Please note that different licenses apply to the data sets, dependencies and other assets downloaded.
License information of used libraries can be found in the directory 'docs/license-libraries'. These libraries are used
without modifications to the upstream version of the code of the respective libraries.
Thus, the code is not provided in the repository.


## Replicability

For more details on the TVCG Replicability Stamp, please refer to `replicability/README.md`.

