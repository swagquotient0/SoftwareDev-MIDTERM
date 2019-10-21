/*
 * @file main.cpp
 * @Author: Gautam Balachandran
 * Created on 11 October 2019
 * @brief Main file for the application
 * Copyright : This code is developed for ENPM808X. Do not copy without citation.
 */

/*
 The MIT License
 Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <AckermannController.h>

int main() {
  double requiredVel, currentVel, requiredTheta, currentTheta, wheelBase;
  int input = 0;
  std::vector<double> requiredD, driveWheelVel, currentD;
  std::cout
      << "Enter the required final velocity of the vehicle in meters/second : "
      << std::endl;
  // Required velocity to be achieved is given as user input
  std::cin >> requiredVel;
  std::cout
      << "Enter the desired position (x and y coordinates) of the vehicle : "
      << std::endl;
  // Desired position of the vehicle is given as user input
  for (int i = 0; i < 2; i++) {
    std::cin >> input;
    requiredD.push_back(input);
  }
  std::cout << "Enter the final orientation of the vehicle in degrees : "
            << std::endl;
  // Final orientation to be achieved by the vehicle is given as user input
  std::cin >> requiredTheta;
  std::cout << "Enter the initial orientation of the vehicle in degrees : "
            << std::endl;
  // Initial orientation of the vehicle is given as user input
  std::cin >> currentTheta;
  std::cout << "Enter the initial position of the vehicle: " << std::endl;
  // Initial position of the vehicle is given as vector of x and y coordinates
  for (int i = 0; i < 2; i++) {
    std::cin >> input;
    currentD.push_back(input);
  }
  std::cout << "Enter the current velocity of the vehicle in meters/second : "
            << std::endl;
  // Current velocity of the vehicle is given as user input
  std::cin >> currentVel;
  std::cout << "Enter the wheel base of the vehicle in meters : " << std::endl;
  // Wheel base of the vehicle is given as user input
  std::cin >> wheelBase;

  AckermannController ackermann;
  ackermann.setL(wheelBase);
  ackermann.setV(currentVel);
  ackermann.setTheta(currentTheta);
  ackermann.setD(currentD);
  ackermann.computeSteering(requiredD, requiredTheta);
  driveWheelVel = ackermann.driveVelocities(requiredVel, requiredTheta);
  // Output the values of two drive wheel velocities
  std::cout << "Required Velocity to be achieved by the vehicle: "
            << requiredVel << std::endl;
  std::cout << "Drive Wheel Velocity of inner wheel : " << driveWheelVel[0]
            << std::endl;
  std::cout << "Drive Wheel Velocity of outer wheel : " << driveWheelVel[1]
            << std::endl;
  return 0;
}
