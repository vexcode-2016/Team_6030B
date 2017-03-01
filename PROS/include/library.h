#pragma once

////////////////
//// Macros ////
////////////////

//Motors
#define MOTOR_WHEEL_LB                  2
#define MOTORS_CLAPPER                  3   //Y-Cabled; right motor is wiring-reversed
#define MOTOR_WHEEL_RB                  4
#define MOTORS_ARM_L_HIGH               5   //To Power Expander A & B via Y-Cable
#define MOTORS_ARM_R_HIGH               6   //To Power Expander C & D via Y-Cable
#define MOTORS_ARM_LR_LOW               7   //Y-Cabled; right-side motor is wiring-reversed
#define MOTOR_WHEEL_LF                  8
#define MOTOR_WHEEL_RF                  9

//Analog Sensors
#define SENSOR_POT_ARM                  1
#define SENSOR_POT_CLAPPER              2
#define SENSOR_GYRO_YAW                 3
#define SENSOR_ACCEL_LX                 5
#define SENSOR_ACCEL_LY                 6
#define SENSOR_ACCEL_RX                 7
#define SENSOR_ACCEL_RY                 8

//Current States of Systems
#define CURRENT_ARM                     \
    (analogRead(SENSOR_POT_ARM) / 10)
#define CURRENT_CLAPPER                 \
    (analogRead(SENSOR_POT_CLAPPER) / 10)
#define CURRENT_PITCH_DELTA             \
    (analogReadCalibrated(SENSOR_GYRO_PITCH))



//////////////////
//// Typedefs ////
//////////////////
typedef struct {
    unsigned char(*fn)(float);
    float arg;
    unsigned char group;
} AutonWrappable;



///////////////////
//// Variables ////
///////////////////

//Motor Groups
extern const signed char motorgroupWheelsL[];
extern const signed char motorgroupWheelsR[];
extern const signed char motorgroupArm[];
extern const signed char motorgroupClapper[];

//Autonomous
extern AutonWrappable autonDoNothing;

//Arm
extern const int armFloorGrab;
extern const int armHoldCube;
extern const int armNoMoreDown;
extern const int armScore;
extern const int armNoMoreUp;
extern int armTarget;
extern double armKpUp;
extern double armKpDown;

//Clapper
extern const int clapperHold;
extern const int clapperReady;
extern const int clapperOpenWide;
extern int clapperTarget;
extern double clapperKp;



///////////////////
//// Functions ////
///////////////////

/**
 * Sets the speed of the specified motor port(s) with slew rate
 * @param ports array containing motor ports for which to set the speed; negative
 * port numbers signify that the speed should be in reverse direction for those ports
 * @param speed the desired signed speed; -127 is fully in the negative direction and
 * 127 is fully in the positive direction for positive ports, with 0 being off
 */
void motorsSlew(const signed char *ports, int speed);

/**
 * Background task for slew rate control
 * DO NOT RUN DIRECTLY AS A FUNCTION!
 * Use ONLY when creating the background task in initialize()
 */
void slewControlTask (void * parameter);

/**
 * Function for easy PID loop implementation
 * Recommended to create a macro for each different PID loop that will be used
 * @param current value representing the current state of the system (generally a sensor reading)
 * @param target value representing the desired state of the system; must
 * use the same units as current
 * @param kp constant representing influence of proportional calculation on motor speed
 * @param ki constant representing influence of integral calculation on motor speed
 * @param kd constant representing influence of derivative calculation on motor speed
 * @param motors array containing motor ports for which to set the speed; negative
 * port numbers signify that the speed should be in reverse direction for those ports
 * @param tolerance the margin of error around the target for which 1 will be returned
 * @return 1 if (current) is within (tolerance) of (target) or 0 otherwise
 */
unsigned char pid(float current, float target, float kp, float ki, float kd, const signed char *motors, float tolerance);

/**
 * TODO: Add documentation
 */
unsigned char armToAngle(float target);

/**
 * TODO: Add documentation
 */
unsigned char clapperToOpenness(float target);

/**
 * Wrapper for using functions constantly needing rerunning in autonomous mode where there is typically no infinite loop
 * Supports simultaneous use of up to 5 child functions at a time
 * Child functions must return 1 on success and accept a float as the only argument
 * Grouping system: child functions are assigned to 'groups' of functions sharing the same identifier;
 * [WIP] each group's functions will stop executing once at least 1 function in the group returns 1;
 * 0 is not a valid group identifier; it will cause the child function to execute until all groups' executions stop
 * @param uno AutonWrappable struct representing the first child function
 * @param dos AutonWrappable struct representing the second child function
 * @param tres AutonWrappable struct representing the third child function
 * @param cuatro AutonWrappable struct representing the fourth child function
 * @param cinco AutonWrappable struct representing the fifth child function
 */
void autonWrapper(AutonWrappable uno, AutonWrappable dos, AutonWrappable tres, AutonWrappable cuatro, AutonWrappable cinco);