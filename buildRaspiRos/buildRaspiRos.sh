#!/bin/bash

cd ~
echo  "添加源，并更新系统"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'

sudo apt-get update
sudo apt-get upgrade

echo "下载所需要的依赖包"
sudo apt-get -y install git vim python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six  checkinstall cmake libboost-all-dev libtinyxml-dev libxml2-dev build-essential

sudo pip install rosdep rosinstall_generator wstool rosinstall

echo "配置rosdep"
sudo rosdep init
rosdep update 

echo "增加系统交换分区,16GB分2GB的交换分区"
sudo dd if=/dev/zero of=/swap bs=1M count=2048
sudo mkswap /swap
sudo swapon /swap
echo "需要手工编辑/etc/fstab文件，将'/swap           none            swap    sw                0       0'添加进去"
#sudo echo "/swap           none            swap    sw                0       0" >> /etc/fstab 

echo "建立ros工作空间"
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "获取相关包的源代码到工作目录"
rosinstall_generator desktop --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-desktop-wet.rosinstall
#rosinstall_generator desktop --rosdistro indigo --deps --wet-only --exclude roslisp collada_parser collada_urdf --tar > indigo-desktop-wet.rosinstall
wstool init -j4 src indigo-desktop-wet.rosinstall


echo "解决ROS GUI版本所需要的依赖问题"
mkdir ~/ros_catkin_ws/external_src

cd ~/ros_catkin_ws/external_src
sudo apt-get build-dep console-bridge
apt-get source -b console-bridge
sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb

cd ~/ros_catkin_ws/external_src
apt-get source -b lz4
sudo dpkg -i liblz4-*.deb

echo "编译提示按enter键，到有选项提示时，选择2， 将包名改为liburdfdom-headers-dev, 接下来的两个问题连续选择'n'， 否则会编译出错"
cd ~/ros_catkin_ws/external_src
git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
cmake .
sudo checkinstall make install

echo "编译提示按enter键，到有选项提示时，选择2， 将包名改为liburdfdom-dev, 接下来的两个问题连续选择'n'， 否则会编译出错"
cd ~/ros_catkin_ws/external_src
git clone https://github.com/ros/urdfdom.git
cd urdfdom
cmake .
sudo checkinstall make install 

echo "编译提示按enter键，到有选项提示时，选择2， 将包名改为collada-dom-dev, 接下来的两个问题连续选择'n'， 否则会编译出错"
cd ~/ros_catkin_ws/external_src
wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
tar -xzf collada-dom-2.4.0.tgz
cd collada-dom-2.4.0
cmake .
sudo checkinstall make install


echo "用rosdep解决其它依赖问题"
cd  ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie

echo "编译相关ROS 包"
cd  ~/ros_catkin_ws
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo 

