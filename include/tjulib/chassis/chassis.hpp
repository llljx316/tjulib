#pragma once
#include "vex.h"

namespace tjulib {

/**
 * @brief Struct containing constants for a drivetrain
 *
 * The constants are stored in a struct so that they can be easily passed to the chassis class
 * Set a constant to 0 and it will be ignored
 *
 * @param leftFrontMotors pointer to the left Front motors
 * @param leftBackMotors pointer to the left Back motors
 * @param rightFrontMotors pointer to the right Front motors
 * @param rightBackMotors pointer to the right Back motors
 * @param chasePower higher values make the robot move faster but causes more overshoot on turns
 */
typedef struct {
        vex::motor_group* leftFrontMotors;
        vex::motor_group* leftBackMotors;
        vex::motor_group* rightFrontMotors;
        vex::motor_group* rightBackMotors;
        float chasePower;//防止转弯和移动时的过冲现象
} Drivetrain_t;

/**
 * @brief Function pointer type for drive curve functions.
 * @param input The control input in the range [-127, 127].
 * @param scale The scaling factor, which can be optionally ignored.
 * @return The new value to be used.
 */
typedef std::function<float(float, float)> DriveCurveFunction_t;

/**
 * @brief  Default drive curve. Modifies  the input with an exponential curve. If the input is 127, the function
 * will always output 127, no matter the value of scale, likewise for -127. This curve was inspired by team 5225, the
 * Pilons. A Desmos graph of this curve can be found here: https://www.desmos.com/calculator/rcfjjg83zx
 * @param input value from -127 to 127
 * @param scale how steep the curve should be.
 * @return The new value to be used.
 */
float defaultDriveCurve(float input, float scale);

/**
 * @brief Chassis class
 *
 */
class Chassis {
    public:
        /**
         * @brief Construct a new Chassis
         *
         * @param drivetrain drivetrain to be used for the chassis
         * @param driveCurve drive curve to be used. defaults to `defaultDriveCurve`
         */
        Chassis(Drivetrain_t drivetrain, DriveCurveFunction_t driveCurve = &defaultDriveCurve);
        // 驱动函数
        setSpinRpm(float LF, float LB, float RF, float RB);
        setSpinPct(float LF, float LB, float RF, float RB);
        setSpinVolt(float LF, float LB, float RF, float RB);
        VRUN(float LF, float LB, float RF, float RB);
        //刹车函数
        setStop(brakeType type);
        setStopType(brakeType type);
        


    {
        private:
            Drivetrain_t drivetrain;
            DriveCurveFunction_t driveCurve;
    };
} // namespace tjulib