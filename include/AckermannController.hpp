/*
 * @file AckermannController.hpp
 * @Author Gautam Balachandran
 * Created on 11 October 2019
 * @brief Ackermann Controller header file
 */

/*
 The MIT License
 Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef INCLUDE_ACKERMANNCONTROLLER_HPP_
#define INCLUDE_ACKERMANNCONTROLLER_HPP_

#include<iostream>
#include<vector>
#include<cmath>
#include<math.h>
#include <fstream>

class AckermannController {
 private:
  /**
   * @brief Variable that defines the heading.
   */
  std::vector<double> d;
  /**
   * @brief Variables that define the orientation, velocity and the wheel base distance respectively
   */
  double theta, v, l;

 public:
  /**
   * @brief Getter method for the heading vector
   * @param  none
   * @return The heading vector containing the x and y coordinates of the heading
   */
  std::vector<double> getD();
  /**
   * @brief Setter method for the heading vector
   * @param  New heading vector to be set
   * @return none
   */
  void setD(std::vector<double> newD);
  /**
   * @brief Getter method for the orientation
   * @param  none
   * @return The orientation of the vehicle
   */
  double getTheta();
  /**
   * @brief Setter method for the orientation
   * @param  New orientation to be set
   * @return none
   */
  void setTheta(double newTheta);
  /**
   * @brief Getter method for the velocity
   * @param  none
   * @return The current velocity of the vehicle
   */
  double getV();
  /**
   * @brief Setter method for the velocity
   * @param  New velocity to be set
   * @return none
   */
  void setV(double newV);
  /**
   * @brief Getter method for the wheel base distance
   * @param  none
   * @return The wheel base distance
   */
  double getL();
  /**
   * @brief Setter method for the wheel base distance
   * @param  New wheel base distance
   * @return none
   */
  void setL(double newL);
  /**
   * @brief Method to compute the look-ahead distance which depends on the current velocity
   * @param
   * @return Computed look-ahead distance
   */
  double computeLH();
  /**
   * @brief Method to compute the controller gains
   * @param none
   * @return Vector of computed gains, kp and kd
   */
  std::vector<double> controlConstants();
  /**
   * @brief Method to compute the steering angle
   * @param Required heading and required orientation
   * @return Computed steering angle
   */
  double computeSteering(std::vector<double> newD, double newTheta);
  /**
   * @brief Method to compute the wheel drive velocities
   * @param Required velocity and required orientation
   * @return Computed vector of the wheel velocities
   */
  std::vector<double> driveVelocities(double reqV, double reqTheta);
};

#endif /* INCLUDE_ACKERMANNCONTROLLER_HPP_ */

