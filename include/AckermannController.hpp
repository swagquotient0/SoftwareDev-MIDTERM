/*
 * AckermannController.hpp
 *
 *  Created on: Oct 11, 2019
 *      Author: Gautam
 */

/*Common Development and Distribution License 1.0
Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam

COVERED SOFTWARE IS PROVIDED UNDER THIS LICENSE ON AN AS IS BASIS, WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
WITHOUT LIMITATION, WARRANTIES THAT THE COVERED SOFTWARE IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE OR 
NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE COVERED SOFTWARE IS WITH YOU. SHOULD ANY COVERED SOFTWARE PROVE
DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE OF ANY COVERED SOFTWARE IS AUTHORIZED
HEREUNDER EXCEPT UNDER THIS DISCLAIMER.*/

#ifndef INCLUDE_ACKERMANNCONTROLLER_HPP_
#define INCLUDE_ACKERMANNCONTROLLER_HPP_

#include<iostream>
#include<vector>
#include<cmath>
#include<math.h>
#include <fstream>

/**
 * @Author Gautam Balachandran
 * @file AckermannController.hpp
 * @brief Ackermann Controller header file
 *
 */

class AckermannController {
 private:
  /**
   * @brief Variable that define the heading.
   */
  std::vector<double> d;
  /**
   * @brief Variables that define the orientation, velocity and the wheel base distance
   */
  double theta, v, l;

 public:
  /**
   * @brief Getter method for the heading vector
   * @param  none
   * @return The heading vector containing the x and y positions of the heading.
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
   * @brief Getter method for the wheel base
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
   * @brief Method to compute the look-ahead for the current velocity
   * @param Current Velocity of the vehicle
   * @return Computed Look-Ahead distance
   */
  double computeLH();
  /**
   * @brief Method to compute the controller gains
   * @param Current Velocity of the vehicle
   * @return Vector of computed gains
   */
  std::vector<double> controlConstants();
  /**
   * @brief Method to compute the steering angle
   * @param Current heading and wheel base distance
   * @return Computed Steering angle
   */
  double computeSteering(std::vector<double> newD, double newTheta);
  /**
   * @brief Method to compute the wheel drive velocities
   * @param Current heading calculated
   * @return Computed vector of the wheel velocities
   */
  std::vector<double> driveVelocities(double reqV, double reqTheta);
};

#endif /* INCLUDE_ACKERMANNCONTROLLER_HPP_ */

