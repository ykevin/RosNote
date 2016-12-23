#!/bin/bash

echo "树莓派系统为官方最新的系统，下载连接为：http://vx2-downloads.raspberrypi.org/raspbian/images/raspbian-2016-05-31/2016-05-27-raspbian-jessie.zip， 如果用其它以前版本编译可能会存在编译错误"

DIR=$(pwd)

cd ~
echo  "添加源，并更新系统"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'

sudo apt-get update
sudo apt-get -y upgrade

echo "下载所需要的依赖包"
sudo apt-get -y install git vim python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six python-empy checkinstall cmake libboost-all-dev libtinyxml-dev libxml2-dev build-essential ttf-wqy-microhei 

echo "解决中文字体乱码问题"
sudo fc-cache

sudo pip install rosdep rosinstall_generator wstool rosinstall

echo "配置rosdep"
sudo rosdep init
rosdep update 

echo "增加系统交换分区,16GB分2GB的交换分区"
sudo dd if=/dev/zero of=/swap bs=1M count=2048
sudo mkswap /swap
sudo swapon /swap
sudo chmod 777 /etc/fstab
#echo "需要手工编辑/etc/fstab文件，将'/swap           none            swap    sw                0       0'添加进去"
sudo echo "/swap           none            swap    sw                0       0" >> /etc/fstab 

echo "建立ros工作空间"
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "获取相关包的源代码到工作目录"
rosinstall_generator desktop navigation move_base amcl rosserial gmapping teleop_twist_keyboard --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-desktop-wet.rosinstall
wstool init  src indigo-desktop-wet.rosinstall

echo "用rosdep解决其它依赖问题"
cd  ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie

echo "打上编译错误补丁"
rviz_dir="/home/pi/ros_catkin_ws/src/rviz/src/rviz/"
rviz_file="~/ros_catkin_ws/src/rviz/src/rviz/mesh_loader.cpp"
if [ ! -f "$rviz_file" ] ; then
    sudo cp $DIR/rviz_mesh_loader.patch $rviz_dir
    cd $rviz_dir
    patch -p0 < rviz_mesh_loader.patch  
fi

urdf_dir="/home/pi/ros_catkin_ws/src/robot_model/collada_urdf/src"
urdf_file="~/ros_catkin_ws/src/robot_model/collada_urdf/src/collada_urdf.cpp"
if [ ! -f "$urdf_file" ] ; then
    sudo cp $DIR/collada_urdf.patch $urdf_dir
    cd $urdf_dir
    patch -p0 < collada_urdf.patch  
fi



echo "编译相关ROS 包"
cd  ~/ros_catkin_ws
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo 

echo "写入环境变量"
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "install user ros ws and  ros_arduino_bridge"
mkdir ~/catkin_ws/src/ -p
cd ~/catkin_ws/src/ 
git clone https://github.com/hbrobotics/ros_arduino_bridge.git
cd ..
catkin_make 

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
