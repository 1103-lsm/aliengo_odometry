#!/bin/bash
echo -e "\033[40;32mgram_savitzky_golay install start\033[0m"
git clone --recurse-submodules https://github.com/arntanguy/gram_savitzky_golay.git
echo -e "\033[40;32mdownload success\033[0m"
# install
cd gram_savitzky_golay
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
# clean
cd ../..
rm -rf gram_savitzky_golay
echo -e "\033[40;32mgram_savitzky_golay install success\033[0m"