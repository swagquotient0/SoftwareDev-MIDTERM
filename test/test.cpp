#include <gtest/gtest.h>
#include "../include/AckermannController.hpp"

TEST(AckermannControllerTest, phiTest) {
  AckermannController AckermannTest;
  AckermannTest.setL(2.4);  /// Wheel Base
  AckermannTest.setV(45);  //  current velocity
  AckermannTest.setTheta(40);  // Current Orientation
  AckermannTest.setD( { 10.0, 15.0 });  // Current Position
  double val = AckermannTest.computeSteering( { 77.5, 17.2 }, 15.0);
  EXPECT_LT(val, 45.0);
}

TEST(AckermannControllerTest, phiTestType) {
  AckermannController AckermannTest;
  AckermannTest.setL(2.4);  /// Wheel Base
  AckermannTest.setV(45);  //  current velocity
  AckermannTest.setTheta(40);  // Current Orientation
  AckermannTest.setD( { 10.0, 15.0 });  // Current Position
  double val = AckermannTest.computeSteering( { 77.5, 17.2 }, 15.0);
  EXPECT_EQ(typeid(val), typeid(double));
}
TEST(AckermannControllerTest, kPTest) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_LT(val[0], 1.0);
}
TEST(AckermannControllerTest, kPTestType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_EQ(typeid(val[0]), typeid(double));
}
TEST(AckermannControllerTest, kDTest) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_LT(val[1], 1.0);
}
TEST(AckermannControllerTest, kDTestType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.controlConstants();
  EXPECT_EQ(typeid(val[1]), typeid(double));
}
TEST(AckermannControllerTest, lHMax) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  double val = AckermannTest.computeLH();
  EXPECT_LE(val, 31.25);
}
TEST(AckermannControllerTest, lHMaxType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  double val = AckermannTest.computeLH();
  EXPECT_EQ(typeid(val), typeid(double));
}
TEST(AckermannControllerTest, lHMin) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  double val = AckermannTest.computeLH();
  EXPECT_GT(val, 10.41);
}
TEST(AckermannControllerTest, lHMinType) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  double val = AckermannTest.computeLH();
  EXPECT_EQ(typeid(val), typeid(double));
}
TEST(AckermannControllerTest, driveVelocitiesTest1) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_LT(val[0], 250);
}
TEST(AckermannControllerTest, driveVelocitiesTestType1) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_EQ(typeid(val[0]), typeid(double));
}
TEST(AckermannControllerTest, driveVelocitiesTest2) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_LT(val[1], 250);
}
TEST(AckermannControllerTest, driveVelocitiesTestType2) {
  AckermannController AckermannTest;
  AckermannTest.setV(45);
  std::vector<double> val = AckermannTest.driveVelocities(30, 50);
  EXPECT_EQ(typeid(val[1]), typeid(double));
}
