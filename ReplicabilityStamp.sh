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
# It is assumed that the build folder is called 'build' and lies in the project
# directory.

if [ -d "Data/Meshes/2020" ]; then
    # This tool can help download data sets from HexaLab. The program assumes the user has the rights to download the data.
    #
    # HexaLab.net: an online viewer for hexahedral meshes
    # Matteo Braccix, Marco Tarini1,2,x, Nico Pietroni1,4, Marco Livesu3, Paolo Cignoni1
    # Computer-Aided Design, Volume 110, May 2019
    # DOI:10.1016/j.cad.2018.12.003
    # (preprint available on arxiv)
    # Copyright 2018 Visual Computing Lab ISTI - CNR
    echo "Downloading HexaLab data sets..."
    build/HexaLabDatasetsDownloader
fi

build/HexVolumeRenderer --replicability