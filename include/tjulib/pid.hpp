#pragma once

#include "vex.h"
#include "velPID.hpp"
#include "Math-Functions.h"

using namespace vex;



namespace tjulib{
    struct args{
        motor *pmotor;
        velPID* pMPID;
        float* ptargetRPM;

        args(motor *pmotor, velPID* pMPID, float* ptargetRPM):pmotor(pmotor), pMPID(pMPID), ptargetRPM(ptargetRPM){};
    };
    int processVelPID(void * nargs);

    class PIDMotor: public motor{

    protected :
        task spinPIDMotor;
        args arg;
        velPID MPID;
        float targetRPM;
        

    public:
        PIDMotor(int index, float Kp, float Ki, float Kd);

        void setPID( float Kp, float Ki, float Kd);

        //暂时只有pct的
        void spinPID( directionType dir, float velocity); 

        void distancePID();

        ~PIDMotor();
    };

};



