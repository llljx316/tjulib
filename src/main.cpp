/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Miracle                                                   */
/*    Created:      2023/10/4 21:42:20                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "tjulib.h"
#include <string>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
tjulib::PIDMotor test(PORT1, 1, 1, 1);
tjulib::motor test1(PORT2);
vex::controller c1 = controller(primary);

// define your global instances of motors and other devices here


int main() {

    Brain.Screen.printAt( 10, 50, "Hello V5" );

    while(1){
        test.spinPID(directionType::fwd, 100);
        if(c1.ButtonA.pressing()){
            test.spinPID(directionType::fwd, 100);
            printf("hello!");
        }
        else{
            test.spinPID(directionType::fwd, 0);
        }
        //printf("%lf", test.velocity(pct));
        
        Brain.Screen.printAt( 10, 50, "%lf", test.velocity(pct) );
        vex::task::sleep(100);

    }
}
