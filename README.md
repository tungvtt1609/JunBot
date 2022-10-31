# JunBot Overview

## Tung version

![JunBot](https://github.com/ScarecrowStraw/JunBot/blob/junbot-dev/resources/junbot.png)

## Hardware
- TurtleBot3 Waffle Pi Platform (OpenCR 1.0 + Dynamixel XM430-W210)
- Hokuyo UST-05LX
- Jetson Xavier AGX (32GB RAM)
- Realsense D435i x 2
- Realsense T265
- ZED 2
 
## Software
- Ubuntu 18.04 
- CUDA 10.2
- Realsense SDK (2.43.0)
- Realsense ROS (2.2.23)
- ROS Melodic
- OpenCV 4.2.0
- ZED SDK 3.6.5
- Qt5

# JunBot Setting

## OpenCR and Dynamixel testing
1. OpenCR test [Here](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)

2. Dynamixel test [Here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

3. Dynamixel and OpenCR communication [Here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

## Setup Dynamixe's for TBot
Because Xavier can not install cross compiler for OpenCR so you need use Intel CPU for this step (at least in when i write this)
[Here](https://emanual.robotis.com/docs/en/platform/turtlebot3/faq/#setup-dynamixels-for-turtlebot3)

## AGX Xavier Setup

### Step 1: Install OpenCV (Qt and Cuda enable)

- Dependencies
```
sudo apt update
sudo apt upgrade
sudo apt install build-essential cmake pkg-config unzip yasm git checkinstall
sudo apt install libjpeg-dev libpng-dev libtiff-dev
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libavresample-dev
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev
sudo apt install libfaac-dev libmp3lame-dev libvorbis-dev
sudo apt install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils
sudo apt-get install libgtk-3-dev
sudo apt-get install qt5-default
sudo apt-get install python3-dev python3-pip
sudo -H pip3 install -U pip numpy
sudo apt install python3-testresources
sudo apt-get install libtbb-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install libprotobuf-dev protobuf-compiler
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```
- Download OpenCV 4.2.0
```
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.2.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.2.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
```

- Install OpenCV (C++)
```
cd opencv-4.2.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_C_COMPILER=/usr/bin/gcc-7 \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D WITH_TBB=ON \
-D WITH_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D WITH_CUBLAS=1 \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D CUDA_ARCH_BIN=7.2 \
-D WITH_V4L=ON \
-D WITH_QT=ON \
-D WITH_OPENGL=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv.pc \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib-4.2.0/modules \
-D BUILD_EXAMPLES=ON .. 

make -j4 
sudo make install

sudo /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'

sudo ldconfig
```

### Step 2: Install ROS and Turtlebot 3 Waffle Pi core

1. ROS melodic install 
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt update

sudo apt install ros-melodic-desktop-full

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

2. Turtlebot 3 Waffle Pi core install 

- Dependencies
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
- TurtleBot3 Packages
```
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
```

- Set TurtleBot3 Model Name
```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

- OpenCR Setup
- 
Connect AGX and OpenCR via USB
```
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=waffle
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
tar -xvf opencr_update.tar.bz2 
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

### Step 4: Install RealSenseSDK and RealSense ROS
- cv_bridge for ROS melodic and opencv 4.2.0 

Edit this file /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake

Find this "/usr/include/opencv" and change to "/usr/include/opencv4"

- RealSenseSKD
[Here](https://github.com/jetsonhacks/installRealSenseSDK)

- RealSense ROS
[Here](https://github.com/jetsonhacks/installRealSenseROS)

### Step 5: Hokuyo setting 

```
# install Hokuyp driver
sudo apt-get install ros-medolic-urg-node
# Network config
sudo ip addr add 192.168.0.15/24 broadcast 192.168.0.255 dev eth0
# Run ROS Node
rosrun urg_node urg_node _ip_address:=192.168.0.10
# Test (option without ROS TF)
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map laser 10
```


### Step 6: Calibration

```
python src/JunBot/jun_camera/scripts/set_cams_transforms.py cam_1_link cam_2_link 0.7 0.6 0 -90 0 0
```
### Step 7: Qt GUI setting

```
sudo apt-get update

sudo apt-get install python3-catkin-tools

sudo apt-get install -y ros-noetic-move-base-msgs

sudo apt install libgl1-mesa-dev ninja-build libyaml-cpp-dev libqtermwidget5-0-dev libutf8proc-dev

sudo apt install qtmultimedia5-dev
```

### Step 8: Cartographer SLAM

```
sudo apt install libgmock-dev
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y -i -r

catkin_make_isolated --install --use-ninja -DPYTHON_EXECUTABLE=$(which python3)
# or
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

# JunBot Running

### TurtleBot3 Control

```
# SLAM simulation in Gazebo
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch cartographer_junbot junbot_2d.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Save map
# Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0

# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '${HOME}/Downloads/<map name>.bag.pbstream', include_unfinished_submaps: "true"}"

# Run navigation
roslaunch cartographer_junbot demo_junbot_2d_localization.launch

# User interface

rosrun junbot_gui junbot_gui

```

# TODO
- [x] Cartographer SLAM with t265
- [x] AMCL relocalization
- [x] T265 navigation
- [ ] User interface
- [ ] Odometry fusion




