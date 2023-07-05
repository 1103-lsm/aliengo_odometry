#!/bin/bash
echo -e "\033[40;32mCeres install start\033[0m"

sudo apt update
sudo apt install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev

git clone https://ceres-solver.googlesource.com/ceres-solver
echo -e "\033[40;32mDownload success\033[0m"
cd ceres-solver
mkdir build
cd build
cmake ..
make -j4
sudo make install

cd ../..
rm -rf ceres-solver
echo -e "\033[40;32mCeres install success\033[0m"