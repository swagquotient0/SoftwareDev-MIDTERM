/*
 * AckermannController.hpp
 *
 *  Created on: Oct 11, 2019
 *      Author: gautam
 */

#ifndef INCLUDE_ACKERMANNCONTROLLER_HPP_
#define INCLUDE_ACKERMANNCONTROLLER_HPP_

#include<iostream>
#include<vector>

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
   * @brief Variable that define the Orientation.
   */
   double theta;
  /**
   * @brief Variables that define the velocity and the wheel base distance
   */
   double v,l;
 
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
   void setD(vector<double> newD);
 /**
   * @brief Getter method for the orientation
   * @param  none
   * @return The orientation of the vehicle
   */
   int getTheta();
 /**
   * @brief Setter method for the orientation
   * @param  New orientation to be set
   * @return none
   */
   void setTheta(int newTheta);
 /**
   * @brief Method to compute the look-ahead for the current velocity
   * @param Current Velocity of the vehicle
   * @return Computed Look-Ahead distance
   */
   double computLH(double vel);
 /**
   * @brief Method to compute the controller gains
   * @param Current Velocity of the vehicle
   * @return Vector of computed gains
   */
   std::vector<double> controlConstants(double vel);
 /**
   * @brief Method to compute the steering angle
   * @param Current heading and wheel base distance
   * @return Computed Steering angle
   */
   double computeSteering(std::vector<double> d, double l);
 /**
   * @brief Method to compute the wheel drive velocities
   * @param Current heading calculated
   * @return Computed vector of the wheel velocities
   */
   std::vector<double> driveVelocities(int calcTheta);
};
 
 
 
#endif /* INCLUDE_ACKERMANNCONTROLLER_HPP_ */

