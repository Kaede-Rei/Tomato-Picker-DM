#!/bin/bash
# This script sets up the environment variables for using the compilers from the conda environment in ROS.
# Usage:
#   source ./use-mamba-gcc.sh
#   . ./use-mamba-gcc.sh

export CC=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-cc
export CXX=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
export LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH
export CMAKE_PREFIX_PATH=$CONDA_PREFIX:$CMAKE_PREFIX_PATH
export PATH=$CONDA_PREFIX/bin:$PATH

echo "----------------------------------------"
echo "Using compilers from conda environment:"
which gcc
which g++
echo "C is set to: $CC"
echo "C++ is set to: $CXX"
echo "----------------------------------------"
