#install gazebo for indigo

##1、install gazebo
		sudo apt-get install ros-indigo-gazebo-ros-pkgs  ros-indigo-gazebo-ros-control

##2、测试 gazebo
		$roscore
		$rosrun gazebo_ros gazebo
		
##3、分开运行gazebo server 与 client
		$rosrun gazebo_ros gzserver
		$rosrun gazebo_ros gzclient
