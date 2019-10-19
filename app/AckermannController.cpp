// Ackermann Steering Control.cpp : Defines the entry point for the application.
/*
 * AckermannController.cpp
 *
 * Created on : Oct 16, 2019
 * Author : Mushty Sri Sai Kaushik
 */

#include "AckermannController.hpp"
#include<fstream>


std::vector<double> AckermannController::getD() {
  return AckermannController::d;
}

void AckermannController::setD(std::vector<double> newD) {
  AckermannController::d.swap(newD);
}

double AckermannController::getTheta() {
  return AckermannController::theta;
}

void AckermannController::setTheta(double newTheta) {
  AckermannController::theta = newTheta;
}

double AckermannController::getV() {
  return AckermannController::v;
}

void AckermannController::setV(double newV) {
  AckermannController::v = newV;
}

double AckermannController::getL() {
  return AckermannController::l;
}

void AckermannController::setL(double newL) {
  AckermannController::l = newL;
}

double AckermannController::computeLH() {
  double vel, lH;
  int vmin = 6.94, vmax = 20.83;

  vel = AckermannController::getV();
  if (vel < vmin) {
    lH = 10.41;
  } else if (vel >= vmin and vel <= vmax) {
    lH = vel * 1.5;
  } else {
    lH = 31.25;
  }
  return lH;
}

std::vector<double> AckermannController::controlConstants() {
  double vel, kP, kD;
  vel = AckermannController::getV();
  kP = pow((0.3383 / vel), 2);
  kD = (0.4 / vel);
  std::vector<double> k { kP, kD };
  return k;
}

double AckermannController::computeSteering(std::vector<double> newD,
                                            double newTheta) {
  std::ofstream myfile;
  myfile.open ("Output.txt");
  std::vector<double> k, d;
  double lH, th, dE, xD, yD,thetaD,thetaE, k1, a1, a2, inner, phi, thetaIncr;
  k = AckermannController::controlConstants();
  lH = computeLH();
  d = AckermannController::getD();
  th = AckermannController::getTheta();

  thetaE = th-newTheta;
  thetaIncr = (double)(fabs(thetaE/10));
  thetaD = th+thetaIncr;

  xD = d[0]+lH*cos(thetaD);
  yD = d[1]+lH*sin(thetaD);
  dE = (yD - newD[1]) * cos(newTheta)-(xD- newD[0]) * sin(newTheta);

  std::cout<<"Position Error : "<<dE<<std::endl;
  std::cout<<"Orientation Error : "<<thetaE<<std::endl;

  while(-10>dE or dE>10){
    myfile<<"Current Orientation : "<<th<<"\n";
    myfile<<"Position Error : "<<dE<<"\n";
    myfile<<"Orientation Error : "<<thetaE<<"\n";


    k1 = (tan(45.0 * M_PI / 180.0) / l);
    a1 = 1-exp((-k1*(sin(thetaE)*(k[1]*tan(thetaE)+k[0]*dE)))/(sin(thetaE)+lH*(pow(cos(thetaE),4))*(k[1]*tan(thetaE) + k[0] * dE)));
    a2 = 1+exp((-k1*(sin(thetaE)*(k[1]*tan(thetaE)+k[0]*dE)))/(sin(thetaE)+lH*(pow(cos(thetaE),4))*(k[1]*tan(thetaE)+k[0]*dE)));
    inner = (double) (-k1 * l * pow(cos(thetaE), 3) * (a1 / a2));
    phi = atan(inner) * 180 / M_PI;

    myfile<<"PHI : "<<phi<<"\n";
    std::cout<<"PHI : "<<phi<<std::endl;

    // Update Sequence
    th = thetaD;
    d[0] = xD;
    d[1] = yD;
    xD += lH*cos(th);
    yD += lH*sin(th);

    if(th<newTheta){
      thetaD =th+thetaIncr;
    }
    else{
      thetaD =th-thetaIncr;
    }

    dE = -(xD- newD[0]) * sin(newTheta) + (yD - newD[1]) * cos(newTheta);

    if(((newTheta-3)>=th or th>=(newTheta+3))){
      thetaE = thetaD-newTheta;
    }
    std::cout<<"Position Error : "<<dE<<std::endl;
    std::cout<<"Orientation Error : "<<thetaE<<std::endl;
    std::cout<<"======================================================================="<<std::endl;
    myfile<<"============================================================================= \n";
  }

  myfile<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% \n";
  myfile<<"DESIRED POSITION : "<<newD[0]<<","<<newD[1]<<"\n";
  myfile<<"FINAL POSITION ERROR: "<<dE<<"\n";
  myfile<<"FINAL POSITION : "<<xD<<","<<yD<<"\n\n";
  myfile<<"DESIRED ORIENTATION : "<<newTheta<<"\n";
  myfile<<"FINAL ORIENTATION ERROR: "<<thetaE<<"\n";
  myfile<<"FINAL ORIENTATION : "<<thetaD<<"\n\n";
  myfile<<"FINAL STEERING ANGLE : "<<phi<<"\n";

  myfile.close();
  return phi;

}

std::vector<double> AckermannController::driveVelocities(double reqV, double reqTheta) {
  double l, wheelRadius, trackWidth, angularVelocity, turningRadius, ICRI,
      displacement1, rps1, driveVelocity1, ICRO, displacement2, rps2,
      driveVelocity2, dt;
  l = AckermannController::getL();
  wheelRadius = 0.3;
  trackWidth = 1;
  dt = 1.0;
  angularVelocity = (reqTheta*M_PI)/(180*dt);
  turningRadius = reqV/angularVelocity;
  ICRI = sqrt(pow(l, 2) + turningRadius - pow((trackWidth / 2), 2));
  displacement1 = ICRI * angularVelocity;
  rps1 = displacement1 / wheelRadius;
  driveVelocity1 = rps1 * M_PI * 2 * wheelRadius ;
  ICRO = sqrt(pow(l, 2) + turningRadius + pow((trackWidth / 2), 2));
  displacement2 = ICRO * angularVelocity;
  rps2 = displacement2 / wheelRadius;
  driveVelocity2 = rps2 * M_PI * 2 * wheelRadius;

  std::cout<<"RPS1 : "<<rps1<<std::endl;
  std::cout<<"RPS2 : "<<rps2<<std::endl;

  std::vector<double> driveVel { driveVelocity1, driveVelocity2 };
  return driveVel;
}
