#!/bin/bash
echo -e "\033[40;32mstart\033[0m"
mkdir build
cd build
cmake ..
echo -e "\033[40;32mcmake success\033[0m"
make
echo -e "\033[40;32mmake success\033[0m"
./test
cd .. && rm -rf build
