/*
 * @file AckermannControllerGmock.hpp
 * @Author Mushty Sri Sai Kaushik
 * Created on 25 November 2019
 * @brief Ackermann Controller Google Mock test file
 */

/*
 The MIT License
 Copyright 2019 Gautam Balachandran, Sri Sai Kaushik, Sri Manika Makam
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <gmock/gmock.h>
#include <AckermannController.h>
#include <vector>

using ::testing::_;

class AckermannControllerGmock : public AckermannController{
 public:
  /*
   * @brief To mock the getTheta function
   * @param none
   * @return void
   */
  MOCK_METHOD1(getTheta, double);
  /*
   * @brief To mock method of Control Constants
   * @param none
   * @return void
   */
  MOCK_METHOD3(controlConstants, std::vector<double>);
  /*
   * @brief To mock computeLH function
   * @param none
   * @return void
   */
  MOCK_METHOD2(computeLH, double);
};
  /*
   * @brief Test for verifying steering angle and drive velocities
   * @param none
   * @return none
   */
TEST(ackermannControllerTest, steeringTest) {
  AckermannControllerGmock controllerTesting;
  ::testing::Expectation check = EXPECT_CALL(controllerTesting,
                                            controlConstants()).TimesAtLeast(1);
  ::testing::Expectation check = EXPECT_CALL(controllerTesting,
                                            computeLH()).TimesAtLeast(1);
  EXPECT_CALL(controllerTesting, getTheta()).After(check);
  double testTheta = 15.5;
  std::vector<double> testD{30.4, 37.3};
  mock.computeSteering(testD, testTheta);
  mock.driveVeocities(testD, testTheta);
}
