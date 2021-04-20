#!/bin/sh

#install neccessary (optional) dependent libs
#test pass on Ubuntu 16.04
echo "Make sure your OS is Ubuntu 16.04, or you have to install some of the dependence on your own"
echo "Begin to install all the dependent libs"

mkdir dependent_libs
echo "Create a new folder called dependent_libs at current path"

sudo apt-get update
# you'd better to use the higher version of cmake for compiling TEASER (which may not be installed by apt-get install)
sudo apt-get install cmake
# google-glog
sudo apt-get install libgoogle-glog-dev
# google-gflag
sudo apt-get install libgflags2 libgflags-dev

echo "install [eigen] 3"
sudo apt-get install libeigen3-dev
pkg-config --modversion eigen3
echo "install [eigen] done"

echo "install [pcl] 1.7"
echo "eigen, boost, flann, vtk involved in pcl"
sudo apt-get install libpcl-dev pcl-tools libproj-dev
echo "install [pcl] done"

cd dependent_libs

echo "install [libLAS] 1.8"
echo "install libLAS dependent libs: geotiff"
sudo apt-get install libgeotiff-dev 
#clone LibLAS to local
git clone https://github.com/libLAS/libLAS.git 
cd libLAS
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [libLAS] done"

echo "install [OpenCV] 3"
echo "install OpenCV dependent libs"
sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libpng16-16 libjpeg9 libjpeg.dev libtiff4.dev libswscale-dev libjasper-dev 
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout "3.4.4"
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
echo "install [OpenCV] done"

cd ..

# you might then delete the dependent_libs folder
sudo rm -rf ./dependent_libs

echo "Finished"

# test pass on Ubuntu 16.04