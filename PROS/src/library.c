#include "main.h"

///////////////////
//// Variables ////
///////////////////

int autonMode;

//Arm
const int armFront = 380;
const int armFence = 340;
const int armBalanced = 300;
const int armBack = 215;
int armTarget = -1;
int armPot = -1;
int armP = 0;

//Clapper
const int clapperClosed = 10;
const int clapperStraight = 25;
const int clapperOpen = 45;
const int clapperBack = 335;
int clapperTarget = -1;
int clapperPot = -1;
int clapperP = 0;



///////////////////
//// Functions ////
///////////////////

//Equivalency to RobotC slaving and reversing of motors
void motorGroupSet (unsigned char motorGroup, int speed) {
    if (motorGroup == MOTORGROUP_WHEELS_L) {
        motorSet(MOTOR_WHEEL_LF, speed);
        motorSet(MOTOR_WHEEL_LB, speed);
    }
    if (motorGroup == MOTORGROUP_WHEELS_R) {
        motorSet(MOTOR_WHEEL_RF, -speed);
        motorSet(MOTOR_WHEEL_RB, -speed);
    }
    if (motorGroup == MOTORGROUP_ARM) {
        motorSet(MOTORS_ARM_L, speed);
        motorSet(MOTORS_ARM_R, -speed);
    }
    if (motorGroup == MOTORGROUP_CLAPPER) {
        motorSet(MOTOR_CLAPPER_L, speed);
        motorSet(MOTOR_CLAPPER_R, -speed);
    }
    if (motorGroup == MOTORGROUP_HANGER) {
        motorSet(MOTOR_HANGER_L, speed);
        motorSet(MOTOR_HANGER_R, -speed);
    }
}

//Arm PID control
void armToAngle (int target) {

    //Read current sensor value
    armPot = analogRead(SENSOR_POT_ARM) / 10;
    printf("Arm: %d, ", armPot);

    //PID control code
    const float pUp = 2.75;
    const float pDown = 0.5;

    if (target != -1) {
        armP = target - armPot;

        if (target > armPot) { //Needing to go up
            motorGroupSet(MOTORGROUP_ARM, (pUp * armP));
        }
        else if (target < armPot) { //Needing to go down
            motorGroupSet(MOTORGROUP_ARM, (pDown * armP));
        }
        printf("Arm_MTR: %d, ", motorGet(MOTORS_ARM_L));
    }
}

//Clapper PID control
void clapperToOpenness (int target) {

    //Read current sensor value
    clapperPot = analogRead(SENSOR_POT_CLAPPER) / 10;
    printf("Clapper: %d, ", clapperPot);

    //PID control code
    const float p = 0.5;

    if (target != -1) {
        clapperP = clapperPot - target;

        motorGroupSet(MOTORGROUP_CLAPPER, (p * clapperP));

        printf("Clapper_MTR: %d, ", motorGet(MOTOR_CLAPPER_L));

    }
}