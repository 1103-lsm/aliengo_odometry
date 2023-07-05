#!/bin/bash
echo -e "\033[40;32myaml-cpp install start\033[0m"

# wget 下载如果卡住就使用代理下载
# wget https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.7.0.tar.gz 
proxychains4  curl -L -O -J https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.7.0.tar.gz # 代理下载 proxychains4 
echo -e "\033[40;32myaml-cpp download success\033[0m"
tar -zxvf yaml-cpp-yaml-cpp-0.7.0.tar.gz
cd yaml-cpp-yaml-cpp-0.7.0/
mkdir build && cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON  .. # 设置CMAKE_POSITION_INDEPENDENT_CODE选项为ON，以启用位置无关代码（PIC) 
sudo make
sudo make install
sudo rm -rf ../../yaml-cpp-yaml-cpp-0.7.0/
sudo rm -rf ../../yaml-cpp-yaml-cpp-0.7.0.tar.gz
echo -e "\033[40;32myaml-cpp install success\033[0m"