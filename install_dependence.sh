#! /bin/bash

# lesson3
sudo apt-get install ros-kinetic-csm

# lesson4
sudo apt-get install ros-kinetic-laser-geometry

# lesson6 install ros-kinetic-g2o
sudo apt-get install ros-kinetic-libg2o

# install ceres-solver-1.13.0
cd TrirdParty/
unzip ceres-solver-1.13.0.zip
cd ceres-solver
mkdir build
cd build
cmake .. 
make

# intsall gtsam-4.0.2
cd ../../
unzip gtsam-4.0.2.zip
cd gtsam-4.0.2
mkdir build
cd build
cmake .. 
make
