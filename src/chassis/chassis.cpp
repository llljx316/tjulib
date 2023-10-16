#define _USE_MATH_DEFINES
#include "vex.h"
#include <cmath>
#include <iostream>
#include <random>
#include "tjulib/chassis/chassis.hpp"

using namespace tjulib;

typedef float T;

Chassis::Chassis(Drivetrain_t drivetrain) {
    this->drivetrain = drivetrain;
}


// DRIVE HELPER FUNCTIONS
void Chassis::setSpinRpm(T LF, T LB, T RF, T RB){
    this->drivetrain->leftFrontMotors.spin(fwd, LF, rpm);
    this->drivetrain->leftBackMotors.spin(fwd, LB, rpm);
    this->drivetrain->rightFrontMotors.spin(fwd, RF, rpm);
    this->drivetrain->rightBackMotors.spin(fwd, RB, rpm);

}

void Chassis::setSpinPct(T LF, T LB, T RF, T RB){
    this->drivetrain->leftFrontMotors.spin(fwd, LF, pct);
    this->drivetrain->leftBackMotors.spin(fwd, LB, pct);
    this->drivetrain->rightFrontMotors.spin(fwd, RF, pct);
    this->drivetrain->rightBackMotors.spin(fwd, RB, pct);
}

void Chassis::setSpinVolt(T LF, T LB, T RF, T RB){
    this->drivetrain->leftFrontMotors.spin(fwd, LF, volt);
    this->drivetrain->leftBackMotors.spin(fwd, LB, volt);
    this->drivetrain->rightFrontMotors.spin(fwd, RF, volt);
    this->drivetrain->rightBackMotors.spin(fwd, RB, volt);
}

void Chassis::VRUN(T LF, T LB, T RF, T RB){
    this->drivetrain->leftFrontMotors.setVelocity(LF,volt);
    this->drivetrain->leftBackMotors.setVelocity(LB, volt);
    this->drivetrain->rightFrontMotors.setVelocity(RF, volt);
    this->drivetrain->rightBackMotors.setVelocity(RB, volt);
}

  
void Chassis::setStop(brakeType type)
{
    this->drivetrain->leftFrontMotors.stop(type);
    this->drivetrain->leftBackMotors.stop(type);
    this->drivetrain->rightFrontMotors.stop(type);
    this->drivetrain->rightBackMotors.stop(type);
}

void Chassis::setStop(brakeType type)
{
    this->drivetrain->leftFrontMotors.stop(type);
    this->drivetrain->leftBackMotors.stop(type);
    this->drivetrain->rightFrontMotors.stop(type);
    this->drivetrain->rightBackMotors.stop(type);
}

void Chassis::setStopType(brakeType type)
{
    this->drivetrain->leftFrontMotors.setStopping(type);
    this->drivetrain->leftBackMotors.setStopping(type);
    this->drivetrain->rightFrontMotors.setStopping(type);
    this->drivetrain->rightBackMotors.setStopping(type);
}