# HexVolumeRenderer
A program for rendering hexahedral meshes in the form of transparent volumes.

## Building and running the programm

On Ubuntu 18.04 for example, you can install all necessary packages with this command (additionally to the prerequisites required by sgl):

```
sudo apt-get install libjsoncpp-dev libcurl-dev libeigen3-dev
```

After installing sgl (see above) execute in the repository directory:

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
pacman -S mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-libcurl
```

On Windows, using MSYS2 and Mingw-w64 (http://www.msys2.org/), it is best to use the following CMake command:
```
cmake .. -G"MSYS Makefiles"
```

To run the program, execute:
```
export LD_LIBRARY_PATH=/usr/local/lib
./HexVolumeRenderer
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
