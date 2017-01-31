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
#define SENSOR_GYRO                     3
#define SENSOR_ACCEL_LX                 5
#define SENSOR_ACCEL_LY                 6
#define SENSOR_ACCEL_RX                 7
#define SENSOR_ACCEL_RY                 8

//Digital Sensors
#define JUMPER_SKILLS                   11
#define JUMPER_AUTON                    12

//Current States of Systems
#define CURRENT_ARM                     \
    (analogRead(SENSOR_POT_ARM) / 10)
#define CURRENT_CLAPPER                 \
    (analogRead(SENSOR_POT_CLAPPER) / 10)

//PID Loops
#define armToAngle(target)              \
    if (target != -1) { pid(CURRENT_ARM, target, (target > CURRENT_ARM) ? armKpUp : armKpDown, 0, 0, motorgroupArm); }
#define clapperToOpenness(target)       \
    if (target != -1) { pid(CURRENT_CLAPPER, target, clapperKp, 0, 0, motorgroupClapper); }

//QwikScore Modes
#define QWIKSCORE_INACTIVE              0
#define QWIKSCORE_GRAB                  1
#define QWIKSCORE_ROTATE                2
#define QWIKSCORE_DRIVE                 3
#define QWIKSCORE_THROW                 4
#define QWIKSCORE_DONE                  5



///////////////////
//// Variables ////
///////////////////

//Motor Groups
extern const signed char motorgroupWheelsL[];
extern const signed char motorgroupWheelsR[];
extern const signed char motorgroupArm[];
extern const signed char motorgroupClapper[];

//Autonomous
extern int autonMode;

//Drivetrain
extern Gyro driveGyro;

//Arm
extern const int armFloorGrab;
extern const int armNoMoreDown;
extern const int armScore;
extern const int armNoMoreUp;
extern int armTarget;
extern double armKpUp;
extern double armKpDown;

//Clapper
extern const int clapperHold;
extern const int clapperReady;
extern const int clapperFence;
extern const int clapperBack;
extern int clapperTarget;
extern double clapperKp;

//QwikScore
extern int qwikScoreMode;
extern int qwikScoreXtraIter;



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
 */
void pid(double current, double target, double kp, double ki, double kd, const signed char *motors);

/**
 * Closes the clapper, raises the arm, rotates, and drives as necessary to score in 1 graceful motion
 * ENCLOSE IN WHILE LOOP AND RESET 'qwikScoreMode' TO 'QWIK_SCORE_INACTIVE' AFTER USE!!!
 * @param autoDrive whether or not the robot should autonomously rotate and drive to the fence (1 = yes, 0 = no)
 */
void qwikScore (int autoDrive);