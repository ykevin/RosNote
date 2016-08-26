#!/bin/bash

if [ $(id -u) != 0 ]; then
   echo "This script requires root permissions"
   echo "$ sudo "$0""
   exit
fi

#Install cuda
cd ~/Downloads/
wget http://developer.download.nvidia.com/compute/cuda/6_5/rel/installers/cuda-repo-l4t-r21.2-6-5-prod_6.5-34_armhf.deb
sudo dpkg  -i cuda-repo-l4t-r21.2-6-5-prod_6.5-34_armhf.deb

sudo apt-get update
sudo apt-get install cuda-toolkit-6-5 
sudo usermod -a -G video $USER  

echo "# Add CUDA bin & library paths:" >> ~/.bashrc
echo "export PATH=/usr/local/cuda/bin:$PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc

