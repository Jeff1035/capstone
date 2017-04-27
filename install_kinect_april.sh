#!/bin/bash
cd ~
echo "installing libfreenect2"
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
cd depends; ./download_debs_trusty.sh
sudo apt-get install build-essential cmake pkg-config
sudo dpkg -i debs/libusb*deb
sudo apt-get install libturbojpeg libjpeg-turbo8-dev
sudo dpkg -i debs/libglfw3*deb; sudo apt-get install -f; sudo apt-get install
sudo apt-add-repository ppa:floe/beignet; sudo apt-get update; sudo apt-get install beignet-dev; sudo dpkg -i debs/ocl-icd*deb
sudo dpkg -i debs/{libva,i965}*deb; sudo apt-get install -f
sudo apt-get install libopenni2-dev
cd ..
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
make
sudo make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

echo "installing apriltags-cpp"
cd ~
git clone https://github.com/personalrobotics/apriltags-cpp
cd apriltags-cpp
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make install

cd ~
echo "installing opencv: 2.4.9  this may take a while"
sudo apt-get update
sudo apt-get ugrade
sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev
cd ~
wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip
unzip opencv-2.4.9.zip
cd opencv-2.4.9
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ..
make
sudo make install

cd 	~
echo "installing iai_kinect and apriltags"
cd catkin_ws/src
git clone https://github.com/personalrobotics/apriltags
git clone https://github.com/code-iai/iai_kinect2
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"

echo "finished"
cd ~