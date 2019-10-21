/*
 * @file AckermannControllerTest.cpp
 * @Author Mushty Sri Sai Kaushik, Sri Manika Makam
 * Created on 18 October 2019
 * @brief Test cases for Ackermann Controller class
 */

/*
 The MIT License
 Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <AckermannController.h>
#include <gtest/gtest.h>

/**
 * @brief Test case to check if the obtained steering angle is less than 45 degrees
 */
TEST(AckermannControllerTest, phiTest) {
  std::vector<double> currentD = { 10.0, 15.0 };
  std::vector<double> requiredD = { 77.5, 17.2 };
  AckermannController AckermannTest;
  AckermannTest.setL(2.4);  //  Wheel Base
  AckermannTest.setV(45);  // Current velocity
  AckermannTest.setTheta(40);  // Current Orientation
  AckermannTest.setD(currentD);  // Current Position
  double val = AckermannTest.computeSteering(requiredD, 15.0);
  EXPECT_LT(val, 45.0);
}
/**
 * @brief Test case to check if the obtained steering angle is of type double
 */
TEST(AckermannControllerTest, phiTestType) {
  std::vector<double> currentD = { 10.0, 15.0 };
  std::vector<double> requiredD = { 77.5, 17.2 };
  AckermannController AckermannTest;
  AckermannTest.setL(2.4);  // Wheel Base
  AckermannTest.setV(45);  // Current velocity
  AckermannTest.setTheta(40);  // Current Orientation
  AckermannTest.setD(currentD);  // Current Position
  double val = AckermannTest.computeSteering(requiredD, 15.0);
  EXPECT_EQ(typeid(val), typeid(double));
}
/**
 * @brief Test case to check if kp is less than 1
 */
TEST(AckermannControllerTest, kPTest) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_LT(val[0], 1.0);
}
/**
 * @brief Test case to check if kp is of type double
 */
TEST(AckermannControllerTest, kPTestType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_EQ(typeid(val[0]), typeid(double));
}
/**
 * @brief Test case to check if kd is less than 1
 */
TEST(AckermannControllerTest, kDTest) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_LT(val[1], 1.0);
}
/**
 * @brief Test case to check if kd is of type double
 */
TEST(AckermannControllerTest, kDTestType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_EQ(typeid(val[1]), typeid(double));
}
/**
 * @brief Test case to check if look-ahead distance is less than or equal to Lmax=31.25m
 */
TEST(AckermannControllerTest, lHMax) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  double val = AckermannTest.computeLH();
  EXPECT_LE(val, 31.25);
}
/**
 * @brief Test case to check if obtained look-ahead distance is of type double
 */
TEST(AckermannControllerTest, lHMaxType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  double val = AckermannTest.computeLH();
  EXPECT_EQ(typeid(val), typeid(double));
}
/**
 * @brief Test case to check if look-ahead distance is greater than or equal to Lmin=10.41m
 */
TEST(AckermannControllerTest, lHMin) {
  AckermannController AckermannTest;
  AckermannTest.setV(25);
  double val = AckermannTest.computeLH();
  EXPECT_GE(val, 10.41);
}
/**
 * @brief Test case to check if obtained look-ahead distance is of type double
 */
TEST(AckermannControllerTest, lHMinType) {
  AckermannController AckermannTest;
  AckermannTest.setV(25);
  double val = AckermannTest.computeLH();
  EXPECT_EQ(typeid(val), typeid(double));
}
/**
 * @brief Test case to check if inner drive wheel velocity is less than 100 meters/second
 */
TEST(AckermannControllerTest, driveVelocitiesTest1) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_LT(val[0], 100);
}
/**
 * @brief Test case to check if inner drive wheel velocity is of type double
 */
TEST(AckermannControllerTest, driveVelocitiesTestType1) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_EQ(typeid(val[0]), typeid(double));
}
/**
 * @brief Test case to check if outer drive wheel velocity is less than 100 meters/second
 */
TEST(AckermannControllerTest, driveVelocitiesTest2) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_LT(val[1], 100);
}
/**
 * @brief Test case to check if outer drive wheel velocity is of type double
 */
TEST(AckermannControllerTest, driveVelocitiesTestType2) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_EQ(typeid(val[1]), typeid(double));
}
