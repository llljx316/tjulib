#pragma once

#include "vex.h"
#include "velPID.hpp"
#include "Math-Functions.h"

using namespace vex;


int test(){
    return 0;
}



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
        PIDMotor(int index, float Kp, float Ki, float Kd):motor(index), arg(this,&this->MPID,&targetRPM), spinPIDMotor(&processVelPID, &arg), MPID(Kp, Ki, Kd){
            spinPIDMotor.suspend();
        }

        void setPID( float Kp, float Ki, float Kd){
            MPID.setGains(Kp, Ki, Kd);
        }

        //暂时只有pct的
        void spinPID( directionType dir, float velocity){
            //改参数
            targetRPM = velocity * (dir==directionType::fwd?1:-1);
            if(abs(velocity)<1e-6)
                spinPIDMotor.suspend();
            else
                spinPIDMotor.resume();
        }
    };


    

    //传地址，连带修改
    
    
    float currentRPM, motorPower, lastPower;

    int processVelPID(void * nargs){
        args *arg = (args *)nargs;
        while(true) {
            currentRPM = arg->pmotor->velocity(rpm);
            motorPower = arg->pMPID->calculate(*arg->ptargetRPM, currentRPM);
            
            if(motorPower <= 0) motorPower = 0; //Prevent motor from spinning backward
            
            //Give the motor a bit of a starting boost
            if(motorPower > lastPower && lastPower < 10 && motorPower > 10) lastPower = 10;
            
            //This slews the motor by limiting the rate of change of the motor speed
            float increment = motorPower - lastPower;
            lastPower = motorPower;
            
        
            vexMotorVoltageSet(arg->pmotor->index(),motorPower* Math::velocityToVoltage);//controled by voltage
            wait(10, msec);
        }
        return 0;
    }



    // class PIDMotorGroup: public motor_group{
    //     public:
    //         spinPID( directionType dir, float velocity){
                
    //             for(int i=0;i<size();i++){
    //                 ((PIDMotor*)at(i))->spinPID(dir, velocity);
    //             }
    //         }
    // };

};



