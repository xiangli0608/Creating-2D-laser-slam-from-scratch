#! /bin/bash

# lesson3
sudo apt-get install ros-kinetic-csm

# lesson4
sudo apt-get install ros-kinetic-laser-geometry

# lesson6 install ceres-solver-1.13.0
cd TrirdParty/
unzip ceres-solver-1.13.0.zip
cd ceres-solver
mkdir build
cd build
cmake .. -G Ninja -DCXX11=ON
ninja
sudo ninja install