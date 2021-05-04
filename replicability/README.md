# HexVolumeRenderer

This file contains information for the TVCG Replicability Stamp.
For more details on the Replicability Stamp, please refer to http://www.replicabilitystamp.org/.

We give permission to the Replicability Stamp committee and reviewers of the Graphics Replicability Stamp Initiative
(GRSI) to review the code and advertise the review publicly after the stamp is approved.

## Submission

Interactive Focus+Context Rendering for Hexahedral Mesh Inspection. \
Christoph Neuhauser, Junpeng Wang, Rüdiger Westermann. \
to appear in IEEE Transactions on Visualization and Computer Graphics 2021 (TVCG 2021).

DOI: 10.1109/TVCG.2021.3074607

## Supported Operating Systems

The build process was tested on...
- Ubuntu 16.04 and Ubuntu 20.04
- Windows 10 using MSYS2

## Hardware

So far, the program was only tested using GPUs from NVIDIA.

## Build Process on Ubuntu

Please execute the script `setup.sh` in this folder on the command line. It will compile and install all requirements,
download the data sets, build the program and execute the program reproducing Figure 19 (right) from the paper.
This process may take some time depending on your internet connection speed. Please note that super user rights need to
be given to parts of the program, as it will install some dependencies using the Ubuntu package manager.

## Build Process on Windows 10

Please download and install MSYS2 from https://www.msys2.org/.
Then, follow the instruction on the MSYS2 webpage to make sure that MSYS2 is up-to-date.
Please make sure to execute `pacman -Syu` until all packages are up-to-date. This should be done no matter whether you
installed MSYS2 freshly for this program or it was already installed on your system. This program uses a new version of
Intel's Embree ray tracing library, and it might not compile if your MSYS2 package sources are not up-to-date.

In the next step, open the MSYS2 MinGW 64-bit shell using the Windows start menu and execute the script `setup.sh` in
this folder. It will compile and install all requirements, download the used data sets, build the program and execute
the program reproducing Figure 19 (right) from the paper. This process may take some time depending on your internet
connection speed.

## Troubleshooting

If an error occurs during the build process, please open a bug report or contact us by e-mail.
After the bug was fixed, please pull the new code and delete the directories `dependencies` and `build` before again
calling `setup.sh`.

When compiling the program on Windows, please make sure that MSYS2 is up-to-date.

When calling `setup.sh`, the program will download 1.4 GiB of hex-mesh data.
Please make sure that enough hard-drive space is available for the mesh files.
