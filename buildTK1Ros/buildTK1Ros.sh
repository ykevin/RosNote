#!/bin/sh

if [ $(id -u) != 0 ]; then
   echo "This script requires root permissions"
   echo "$ sudo "$0""
   exit
fi

# To obtain full performance on the CPU (eg: for performance measurements or benchmarking or when you don't care about power draw), you can disable CPU scaling and force the 4 main CPU cores to always run at max performance until reboot:

echo 0 > /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
echo 1 > /sys/devices/system/cpu/cpu0/online
echo 1 > /sys/devices/system/cpu/cpu1/online
echo 1 > /sys/devices/system/cpu/cpu2/online
echo 1 > /sys/devices/system/cpu/cpu3/online
echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Load prerequisites
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted

#Setup Locale
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install  bash-completion command-not-found libncurses5-dev -y
sudo apt-get install ros-indigo-ros-base -y
# Add Individual Packages here
# You can install a specific ROS package (replace underscores with dashes of the package name):
# sudo apt-get install ros-indigo-PACKAGE
# e.g.
# sudo apt-get install ros-indigo-navigation
#
# To find available packages:
# apt-cache search ros-indigo
# 
# Initialize rosdep
sudo apt-get install python-rosdep -y
sudo rosdep init
# To find available packages, use:
rosdep update
# Environment Setup
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Install rosinstall
sudo apt-get install python-rosinstall -y

# Get the kernel source for LT4 21.5
cd /usr/src/
wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v5.0/source/kernel_src.tbz2
# Decompress
tar -xvf kernel_src.tbz2
cd kernel
# Get the kernel configuration file
zcat /proc/config.gz > .config
# And begin editing
make menuconfig

sudo sed -i 's/# CONFIG_USB_SERIAL_FTDI_SIO is not set/CONFIG_USB_SERIAL_FTDI_SIO=m/' .config
# Make sure that the local kernel version is set
LOCALVERSION=$(uname -r)
# vodoo incantation; This removes everything from the beginning to the last occurrence of "-"
# of the local version string i.e. 3.10.40 is removed
release="${LOCALVERSION##*-}"
CONFIGVERSION="CONFIG_LOCALVERSION=\"-$release\""
# Replace the empty local version with the local version of this kernel
sudo sed -i 's/CONFIG_LOCALVERSION=""/'$CONFIGVERSION'/' .config

# Before running this script, you should have used
# $ menu menuconfig
# to select the wireless drivers that you wanted to compile
# For wifi, you'll need to build cfg80211, mac80211 and the
# driver for the particular device(s) you are building
cd /usr/src/kernel
make prepare
make modules_prepare
make M=drivers/usb/serial/
make modules SUBDIRS=net/wireless
make modules SUBDIRS=net/mac80211
make modules SUBDIRS=drivers/net/wireless

# copy the driver files to ~/builtDrivers
mkdir ~/builtModulesAndDrivers
mkdir ~/builtModulesAndDrivers/builtDrivers
cd /usr/src/kernel/drivers/net/wireless/
find . -name '*.ko' | cpio -pdm ~/builtModulesAndDrivers/builtDrivers
mkdir ~/builtModulesAndDrivers/builtModules
cd /usr/src/kernel/net/wireless
find . -name '*.ko' | cpio -pdm ~/builtModulesAndDrivers/builtModules
cd /usr/src/kernel/net/mac80211
find . -name '*.ko' | cpio -pdm ~/builtModulesAndDrivers/builtModules

# copy modules and drivers to the correct places
# Install wireless drivers card
cp -rv ~/builtModulesAndDrivers/builtDrivers /lib/modules/$(uname -r)/kernel/drivers/net/wireless
# Supplied mac80211.ko does not export __ieee80211_get_radio_led_name symbol; Newly built one does
sudo cp -v ~/builtModulesAndDrivers/builtModules/mac80211.ko /lib/modules/$(uname -r)/kernel/net/mac80211
# Update wireless module - cfg80211.ko
sudo cp -v ~/builtModulesAndDrivers/builtModules/cfg80211.ko /lib/modules/$(uname -r)/kernel/net/wireless
# After compilation, copy the compiled module to the system area
cd /usr/src/kernel
sudo cp -v drivers/usb/serial/ftdi_sio.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial

depmod -a
sudo apt-get install linux-firmware -y
/bin/echo -e "\e[1;32mFTDI Driver Module Installed.\e[0m"
# output completion message in green
echo "$(tput setaf 2)Drivers and Modules copied. "
echo "Please Modify /etc/rc.local as appropriate$(tput setaf 7)"

echo "install turtlebot2"
sudo apt-get install -y ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions  ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs ros-indigo-collada-urdf ros-indigo-navigation ros-indigo-slam-gmapping

sudo chown ubuntu .ros/
source ./opt/ros/indigo/setup.bash

rosrun kobuki_ftdi create_udev_rules 
