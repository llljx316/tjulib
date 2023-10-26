#include"tjulib/pid.hpp"
using namespace vex;

namespace tjulib{
    PIDMotor::PIDMotor(int index, float Kp, float Ki, float Kd):motor(index), arg(this,&this->MPID,&targetRPM), MPID(Kp, Ki, Kd), spinPIDMotor(&processVelPID, &arg){
        spinPIDMotor.suspend();
    }

    void PIDMotor::setPID( float Kp, float Ki, float Kd){
        MPID.setGains(Kp, Ki, Kd);
    }

    //暂时只有pct的
    void PIDMotor::spinPID( directionType dir, float velocity){
        //改参数
        targetRPM = velocity * (dir==directionType::fwd?1:-1);
        if(abs(velocity)<1e-6)
            spinPIDMotor.suspend();
        else
            spinPIDMotor.resume();
    }

    void PIDMotor::distancePID(){}

    PIDMotor::~PIDMotor(){
        spinPIDMotor.stop();
    }

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

};