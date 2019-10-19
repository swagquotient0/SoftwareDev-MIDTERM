/*
 * AckermannController.hpp
 *
 *  Created on: Oct 11, 2019
 *      Author: Gautam
 */

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
  double computeSteering(std::vector<double> newD, int newTheta);
  /**
   * @brief Method to compute the wheel drive velocities
   * @param Current heading calculated
   * @return Computed vector of the wheel velocities
   */
  std::vector<double> driveVelocities(double reqV,double reqTheta);
};

#endif /* INCLUDE_ACKERMANNCONTROLLER_HPP_ */

