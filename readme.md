# C++ Boilerplate
[![Build Status](https://travis-ci.org/Gautam-Balachandran/SoftwareDev-MIDTERM.svg?branch=master)](https://travis-ci.org/Gautam-Balachandran/SoftwareDev-MIDTERM)
[![Coverage Status](https://coveralls.io/repos/github/Gautam-Balachandran/SoftwareDev-MIDTERM/badge.svg?branch=master)](https://coveralls.io/github/Gautam-Balachandran/SoftwareDev-MIDTERM?branch=master)
[![License: MIT](https://img.shields.io/badge/License-CDDL-red.svg)](https://opensource.org/licenses/CDDL-1.0)
---

## Authors

Sprint 1:
- Driver : Gautam Balachandran
- Navigator : Sri Sai Kaushik
- Design Keeper : Sri Manika Makam

Sprint 2:
- Driver : Sri Sai Kaushik
- Navigator : Sri Manika Makam
- Design Keeper : Gautam Balachandran

## Overview

In this project, we have implemented the steering control for the Ackermann kinematic model. The control problem is to find the steering angle and the two driver velocities, given the expected heading and velocity of the robot or vehicle. The Ackermann steering model mechanism assumes that the steering angle of the front two wheels is calculated around a common center. As the rear wheels are assumed to be fixed, the center point is assumed to be along the line drawn from the axles of the rear wheels. The angle made by the center point to the tangent of the desired trajectory gives the desired steering angle. And, in order to calculate the drive wheel velocities, we have to consider the displacement of each of the wheels and their respective wheel radii which is assumed to be a constant value. The main goal of this model is to ensure that the vehicle always keeps moving in the desired trajectory with proper orientation. This controller plays a greater importance in autonomous driving applications and while buiding robots which can travel on uneven terrains.

## Agile Iterative Process (AIP)
This project was completed using AIP with the involvement of 3 programmers using Pair-programming in turns. The detailed Product Backlog, Iteration Backlogs and Work Log are mentioned in the link given below : 
https://docs.google.com/spreadsheets/d/1s-P0bJoN7X7E9u1Dm3FD0PoOtxiNC-nHBYtS63hynHs/edit?usp=sharing

## Algorithm

<p align="center">
  <img width="250" height="250" src="https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM/blob/Iteration-1/Images/Ackermann.-Steering-1.png">
  <img width="250" height="250" src="https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM/blob/Iteration-1/Images/Ackermann.-Steering-2.png">
</p>

<p align="center">
  <img width="500" height="250" src="https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM/blob/Iteration-1/Images/Closed_Loop_Controller.png">
</p>

## To-do tasks for pair programming (Driver navigator discussion)
- [x] Create UML Class and Activity diagram.
- [x] Run cpplint and cppcheck as part of Sprint 2.
- [x] Add defect log and release backlog.
- [x] Modify implementation pipeline using separate classes
- [x] Update UML diagrams with changes in implementation.
- [x] Write Unit Test cases for testing and perform the testing.
- [x] Run Valgrind to detect memory leaks. Fix all detected memory leaks.
- [x] Generate Doxygen documentation in the docs folder.
- [x] Make sure that the repository is updated with all delivarables as mentioned in the proposal.

## Standard install via command-line
```
git clone --recursive https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: 
```

## Building for code coverage 
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## Working with Eclipse IDE ##

## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)
```
mkdir -p ~/workspace
cd ~/workspace
git clone --recursive https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM
```

In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of lane detector

```
cd ~/workspace
mkdir -p SoftwareDev-MIDTERM
cd SoftwareDev-MIDTERM
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 ../SoftwareDev-MIDTERM/
```

## Import

Open Eclipse, go to File -> Import -> General -> Existing Projects into Workspace -> 
Select "SoftwareDev-MIDTERM" directory created previously as root directory -> Finish

# Edit

Source files may be edited under the "[Source Directory]" label in the Project Explorer.


## Build

To build the project, in Eclipse, unfold laneDetector project in Project Explorer,
unfold Build Targets, double click on "all" to build all projects.

## Run

1. In Eclipse, right click on the laneDetector in Project Explorer,
select Run As -> Local C/C++ Application

2. Choose the binaries to run (e.g. SoftwareDev-MIDTERM, cpp-test for unit testing)


## Debug


1. Set breakpoint in source file (i.e. double click in the left margin on the line you want 
the program to break).

2. In Eclipse, right click on the boilerplate-eclipse in Project Explorer, select Debug As -> 
Local C/C++ Application, choose the binaries to run (e.g. SoftwareDev-MIDTERM).

3. If prompt to "Confirm Perspective Switch", select yes.

4. Program will break at the breakpoint you set.

5. Press Step Into (F5), Step Over (F6), Step Return (F7) to step/debug your program.

6. Right click on the variable in editor to add watch expression to watch the variable in 
debugger window.

7. Press Terminate icon to terminate debugging and press C/C++ icon to switch back to C/C++ 
perspetive view (or Windows->Perspective->Open Perspective->C/C++).


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project.

## Doxygen

Doxygen is a tool used for generating software reference documentation.

To install it use 
```
sudo apt install doxygen
```
To generate doxygen documentation after installation use 
```
doxygen ./Doxygen
```

## License
```
Common Development and Distribution License 1.0
Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
