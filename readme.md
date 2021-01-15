# OAK-D ORB_SLAM3 Experiment

This is an experiment in getting ORB_SLAM3 to work with an OAK-D.

`src/main.cpp` is well commented and describes what's needed in order to get
position information out of an OAK-D using ORB_SLAM3.

TODO: More info

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
