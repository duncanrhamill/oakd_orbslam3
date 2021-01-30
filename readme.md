# OAK-D ORB_SLAM3 Experiment

This is an experiment in getting ORB_SLAM3 to work with an OAK-D.

`src/main.cpp` is well commented and describes what's needed in order to get
position information out of an OAK-D using ORB_SLAM3.

TODO: More info

[![Simple Demo](http://img.youtube.com/vi/3b_P6yItYxM/0.jpg)](http://www.youtube.com/watch?v=3b_P6yItYxM "OAK-D / ORB_SLAM3 Demo")

## OpenCV Versions

Support for OpenCV 3 or 4 is selected by the branch you choose:

- [`main`](https://github.com/duncanrhamill/oakd_orbslam3/tree/main): Uses a
  slightly fixed version of ORB_SLAM3 which targets OpenCV 3.
- [`opencv_4`](https://github.com/duncanrhamill/oakd_orbslam3/tree/opencv_4):
  Uses a PR branch of ORB_SLAM3 which targets OpenCV 4. 
## Build

1. Download ORB_SLAM3 submodule (note this is a fork that fixes some build
   issues I was having, not the actual ORB_SLAM3 repo), and depthai-core:
```
git submodule init
```
2. Follow ORB_SLAM3 install and build process in [their
   readme](ORB_SLAM3/README.md) to build `libORB_SLAM3.so`.

   On Ubuntu 20.04 you may need to install
   [libilmbase24](https://www.ubuntuupdates.org/package/core/focal/universe/base/libilmbase24)
   and
   [libopenexr24](https://www.ubuntuupdates.org/package/core/focal/universe/base/libopenexr24),
   as there seems to have been some shared library mismatch on 20.04.
3. Install depthai-core dependencies and submodules git:
```
cd depthai-core
git submodule update --init --recursive
```
4. Configure the settings to be used by ORB_SLAM3, I did this by running
   `depthai_demo.py` from the main `depthai` repo, and copying the information
   required from the EEPROM dump at the start of the script.
5. Build the project:
```
mkdir build && cd build
cmake ..
make
```
6. From the root of this repo run the experiment:
```
./build/bin/oakd_orbslam3
```
