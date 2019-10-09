# C++ Boilerplate
[![Build Status](https://travis-ci.org/Gautam-Balachandran/SoftwareDev-MIDTERM.svg?branch=master)](https://travis-ci.org/Gautam-Balachandran/SoftwareDev-MIDTERM)
[![Coverage Status](https://coveralls.io/repos/github/Gautam-Balachandran/SoftwareDev-MIDTERM/badge.svg?branch=master)](https://coveralls.io/github/Gautam-Balachandran/SoftwareDev-MIDTERM?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
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

## Algorithm

<p align="center">
  <img width="250" height="250" src="https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM/blob/Iteration-1/Images/Ackermann.-Steering-1.png">
  <img width="250" height="250" src="https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM/blob/Iteration-1/Images/Ackermann.-Steering-2.png">
</p>

<p align="center">
  <img width="500" height="250" src="https://github.com/Gautam-Balachandran/SoftwareDev-MIDTERM/blob/Iteration-1/Images/Closed_Loop_Controller.png">
</p>

