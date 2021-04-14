# HexVolumeRenderer

This file contains information for the TVCG Replicability Stamp.
For more details on the replicability stamp, please refer to http://www.replicabilitystamp.org/.

We give permission to the reproducibility committee and reviewers of the Graphics Replicability Stamp Initiative (GRSI)
to review the code and advertise the review publicly after the stamp is approved.

## Submission

Interactive Focus+Context Rendering for Volumetric Mesh Inspection.
Christoph Neuhauser, Junpeng Wang, Rüdiger Westermann.
To appear in IEEE Transactions on Visualization and Computer Graphics 2021.

TODO: Arxiv

## Supported Operating Systems

The build process was tested on...
- Ubuntu 20.04
- Windows 10 using MSYS2

## Build Process on Ubuntu 20.04

Please execute the script `setup.sh` on the command line.
It will compile and install all requirements, download the data sets, build the program and execute the program
reproducing Figure 19 (right) from the paper. Please note that super user rights need to be given to parts of the
program, as it will install some dependencies using the Ubuntu package manager.

## Build Process on Windows 10

Please download and install MSYS2 from https://www.msys2.org/.
Then, follow the instruction on the MSYS2 webpage to make sure that MSYS2 is up-to-date.

In the next step, open the MSYS2 MinGW 64-bit shell using the Windows start menu and execute the script `setup.sh`.
It will compile and install all requirements, download the used data sets, build the program and execute the program
reproducing figure 19 (right) from the paper.

## Hardware

So far, the program was only tested using GPUs from NVIDIA.
