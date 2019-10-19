/*
 * main file
 *
 *  Created on: Oct 19, 2019
 *      Author: Mushty Sri Sai Kaushik
 *  Copyright : This code is developed for ENPM808X. Do not copy without citation.
 */

/*Common Development and Distribution License 1.0
Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam

COVERED SOFTWARE IS PROVIDED UNDER THIS LICENSE ON AN AS IS BASIS, WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
WITHOUT LIMITATION, WARRANTIES THAT THE COVERED SOFTWARE IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE OR 
NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE COVERED SOFTWARE IS WITH YOU. SHOULD ANY COVERED SOFTWARE PROVE
DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE OF ANY COVERED SOFTWARE IS AUTHORIZED
HEREUNDER EXCEPT UNDER THIS DISCLAIMER.*/
  
#include <iostream>
#include <AckermannController.hpp>

int main() {

  
  double requiredVel = 30;  // Velocity to be achieved
  std::vector<double> requiredD { 50, 100 };
  int requiredTheta = 80;  // Heading angle to be achieved
  ///Assign the x,y values for currentD
  std::vector<double> currentD { 15.0, 5.0 };
  std::vector<double> driveWheelVel;
  double steeringAng;

  AckermannController ackermann;
  ackermann.setL(2.4);  /// Wheel Base
  ackermann.setV(28);  ///  current velocity
  ackermann.setTheta(30);  /// Current Orientation
  ackermann.setD(currentD);  /// Current Position
  ///Output values of Required and Current Orientation
  
  std::cout << "Required Orientation : " << requiredTheta << std::endl;
  std::cout << "Current Orientation : " << ackermann.getTheta() << std::endl;
  
  steeringAng = ackermann.computeSteering(requiredD, requiredTheta);
  ///Give parameters for driveWheelVel
  driveWheelVel = ackermann.driveVelocities(requiredVel, requiredTheta);
  ///Output values of drive velocities and required velocities
  std::cout << "Required Velocity : " << requiredVel << std::endl;
  std::cout << "Drive Wheel Velocity 1 : " << driveWheelVel[0] << std::endl;
  std::cout << "Drive Wheel Velocity 2 : " << driveWheelVel[1] << std::endl;
  return 0;
}
