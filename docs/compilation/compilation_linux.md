## Compilation on Linux

### Ubuntu

The build process was tested on Ubuntu 16.04 and Ubuntu 20.04.

As the first step, the library [sgl](https://github.com/chrismile/sgl) (https://github.com/chrismile/sgl) needs to be
installed somewhere on the system. In the following steps, it is assumed you have set up sgl and installed all of its
dependencies. We recommend using the branch `tvcg-hexmesh` of sgl, which was tested for compatibility with
HexVolumeRenderer.

After that, all obligatory dependencies can be installed using the following command.

```
sudo apt-get install cmake libglm-dev libsdl2-dev libsdl2-image-dev libpng-dev libboost-filesystem-dev libtinyxml2-dev \
libarchive-dev libjsoncpp-dev libeigen3-dev python3-dev

# Ubuntu 20.04
sudo apt-get install libcurl4-openssl-dev
OR:
sudo apt-get install libcurl4-gnutls-dev

# Ubuntu 16.04
sudo apt-get install libcurl-dev
```

There are two optional dependencies that can to be set up manually. They are currently not part of the Ubuntu package
repository. The build process will also work perfectly fine without them.

- The ray tracing library [Intel Embree](https://github.com/embree/embree). If it is not installed,
  [NanoRT](https://github.com/lighttransport/nanort) is used as a (slower) fallback.
  Pass `-Dembree_DIR=<path-to-embree>` to CMake in case it might not be able to automatically locate your local Embree
  installation. If TBB and ISPC are not found and installed manually, please use the following command.
```
cmake -DISPC_EXECUTABLE=<path-to-ispc-binary> -DTBB_ROOT=<path-to-tbb> -Dembree_DIR=<path-to-embree> ..
```
- The graph library [Lemon](https://lemon.cs.elte.hu/trac/lemon). It was removed from the Ubuntu repositories starting
  Ubuntu 20.04.

After all dependencies have been set up, the following commands can be used to build the program.

```
mkdir build
cd build
cmake -Dsgl_DIR=<path-to-sgl> ..
make -j
```

If the program was built out-of-source (i.e., the folder `build` does not lie in the source directory), the user must
either create a symbolic link to the directory `Data` in the build folder (this only works on Linux and not Windows),
or the CMake variable `DATA_PATH` must be set to the path pointing to the `Data` folder.

If sgl was not installed globally on the system, the library path might need to be adapted before launching the
application.

```
export LD_LIBRARY_PATH=<path-to-sgl>/lib
./HexVolumeRenderer
```


### Arch Linux

The following command can be used to install all dependencies on Arch Linux (last tested in May 2021).

```
sudo pacman -S cmake glew boost libarchive glm tinyxml2 sdl2 sdl2_image python3 eigen curl jsoncpp
```

All other build instructions are identical to the ones for Ubuntu provided above.
