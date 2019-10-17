#include "Ackermann Steering Control.h"
#include<vector>
#include<math>



int main()
{
	
	std::vector<double> AckermannController::getD();
	{
		return getD{ double x, double y };
	}
	
	double AckermannController::computeLH(double vel);
	{
		if (vel < 25) {
			return double Lh = 10.41;
		}
		if (25 <= vel <= 75) {
			return double Lh = vel * 1.5;
		}
		else {
			return double Lh = 31.25;
		}
	}
	std::vector<double> AckermannController::controlConstants(double vel);
	{
		double kP = (0.3383 / vel) ^ 2;
		double kD = (0.4 / vel);

		return std::vector k{ kP, kD };
	}

	double AckermannController::computeSteering(std::vector<double> d, double l);
	{
		double k1 = (1 / l);
		double a1 = (1 - exp ^ (-k1 * (sin(newTheta) * (kD * tan(newtheta) + kP * newD))) / (sin(newTheta) + Lh * ((cos (newTheta))^4) * (kD * tan(newtheta) + kP * newD)));
		double a2 = (1 + exp ^ (-k1 * (sin(newTheta) * (kD * tan(newtheta) + kP * newD))) / (sin(newTheta) + Lh * ((cos (newTheta))^4) * (kD * tan(newtheta) + kP * newD)));
		double phi = atan(-((cos(newTheta)) ^ 3)(a1 / a2));
		return phi;
	}

	std::vector<double> AckermannController::driveVelocities(int calcTheta);
	{

	}
	
}
