####build csi camera(usb camera)
> $ git clone https://github.com/ktossell/camera_umd.git  
$ rosdep install camera_umd uvc_camera jpeg_streamer  
$ sudo apt-get install libv4l-dev

####about cv_bridge demo
[cv_bridge demo](http://www.pirobot.org/blog/0016/)

####ORB_SLAM2 build raspberry
* build Pangolin 
> sudo apt-get install libglew-dev libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev  
git clone https://github.com/stevenlovegrove/Pangolin.git  
cd Pangolin  
mkdir build  
cd build  
cmake ..  
make -j1

* build orb_slam2 for ros,but make ORB_SLAM2 must "make -j1" 
> cd ~/catkin_ws/src/  
git clone https://github.com/raulmur/ORB_SLAM2.git
cd ORB_SLAM2  
chmod +x build.sh  
./build.sh  

* test camera_calibration
> roslaunch uvc_camera camera_node.launch  
roslaunch uvc_camera camera_node.launch --size 8x6 --square 0.025 image:=/image_raw

* slove lidar slam start error
> sudo apt-get update  
sudo dpkg --configure -a


