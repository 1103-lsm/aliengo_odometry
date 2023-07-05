#!/bin/bash

echo -e "\033[40;32mipopt install start\033[0m"
sudo apt update
sudo apt install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev libblas-dev unzip
mkdir Ipopt_pkg  
cd Ipopt_pkg

# 如果 git clone 卡住，可以为 git 设置代理

# 安装 ASL
echo -e "\033[40;32mipopt install ASL\033[0m"
git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
cd ThirdParty-ASL
sudo ./get.ASL # 有时候会卡住，可以手动下载
sudo ./configure
sudo make
sudo make install
cd ..

# 安装 HSL
echo -e "\033[40;32mipopt install HSL\033[0m"
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd ThirdParty-HSL
# 接下来需要下载coinhsl文件，并解压到ThirdParty-HSL目录下
git clone https://github.com/CHH3213/testCPP.git
cd testCPP
unzip coinhsl.zip -d ../
cd ..
rm -rf testCPP/
sudo ./configure
sudo make
sudo make install
cd ..

# 安装 MUMPS
echo -e "\033[40;32mipopt install MUMPS\033[0m"
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
sudo ./get.Mumps
sudo ./configure
sudo make
sudo make install
cd ..

# 安装 IPOPT
echo -e "\033[40;32mipopt install IPOPT\033[0m"
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt
mkdir build
cd build
sudo ../configure
sudo make
sudo make test
sudo make install
cd ..

# 删除临时文件
cd ../..
sudo rm -rf Ipopt_pkg/

# 完善环境
echo -e "\033[40;32mimprove the env\033[0m"
cd /usr/local/include
sudo cp coin-or coin -r
sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3


echo -e "\033[40;32mipopt install success\033[0m"
