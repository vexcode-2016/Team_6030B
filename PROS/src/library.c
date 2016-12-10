#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Autonomous
int autonMode;

//Arm
const int armFloorGrab = 380;
const int armHighest = 340;
const int armDrop = 320;
const int armFenceGrab = 300;
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

//Drive-straight
int driveImeL = 0;
int driveImeR = 0;
int driveP = 0;
int driveComp = 0;

//Drive-rotate
Gyro rotateGyroSensor;
int rotateGyro = -1;
int rotateP = 0;

//QwikScore
int qwikScoreMode = QWIKSCORE_INACTIVE;



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
        //motorSet(MOTOR_HANGER_L, speed);
        //motorSet(MOTOR_HANGER_R, -speed);
    }
}

//Arm PID control
void armToAngle (int target) {
    //Read current sensor value
    armPot = analogRead(SENSOR_POT_ARM) / 10;
    printf("Arm: %d, ", armPot);

    //PID control code
    const float pUp = 0.5;
    const float pDown = 0.5;

    if (target != -1) {
        armP = target - armPot;

        if (((armPot < armHighest) && (armPot < target)) || ((armPot > armHighest) && (armPot > target))) { //Needing to go up
            motorGroupSet(MOTORGROUP_ARM, (pUp * armP));
        }
        else { //Needing to go down
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
    print

    //PID control code
    const float p = 1;

    if (target != -1) {
        clapperP = target - clapperPot;

        //motorGroupSet(MOTORGROUP_CLAPPER, p * clapperP);

        printf("Clapper_MTR: %d, ", p * clapperP);
    }
}

//Drive-straightish PID control
void driveStraightish (int target) {
    //Read current sensor value
    imeGet (SENSOR_IME_WHEEL_LF, &driveImeL);

    //PID control code
    const float p = 0.5;

    driveP = target - driveImeL;

    motorGroupSet (MOTORGROUP_WHEELS_L, p * driveP);
    motorGroupSet (MOTORGROUP_WHEELS_R, p * driveP);

    printf ("Drive_MTR: %d, ", motorGet (MOTOR_WHEEL_LF));
}

//Drive-rotate PID control
void rotateToHeading (int target) {
    //Read current sensor value
    rotateGyro = gyroGet (rotateGyroSensor);
    rotateGyro = ((rotateGyro > 0) - (rotateGyro < 0)) * (abs (rotateGyro) % 360);
    printf ("Rotate: %d, ", rotateGyro);

    //PID control code
    const float p = 0.5;

    if (target != -1) {
        rotateP = target - rotateGyro;

        motorGroupSet (MOTORGROUP_WHEELS_L, p * rotateP);
        motorGroupSet (MOTORGROUP_WHEELS_R, -p * rotateP);

        printf ("Rotate_MTR: %d, ", motorGet (MOTOR_WHEEL_LF));
    }
}

//QwikScore
void qwikScore() {
    if (qwikScoreMode == QWIKSCORE_INACTIVE)
        qwikScoreMode += 1;
    if (qwikScoreMode == QWIKSCORE_GRAB) {
        if (abs (clapperPot - clapperClosed) > 15)
            clapperToOpenness (clapperClosed);
        else
            qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_RAISE) {
        if (abs (armPot - armDrop) > 15)
            armToAngle (armDrop);
        else
            qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_ROTATE) {
        if (((abs (gyroGet (rotateGyroSensor))) % 360) - 0 > 10)
            rotateToHeading (0);
        else
            qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_DRIVE) {
        if (!digitalRead (SENSOR_BUMPER_FENCE_LH) && !digitalRead (SENSOR_BUMPER_FENCE_LL) && !digitalRead(SENSOR_BUMPER_FENCE_RH) && !digitalRead(SENSOR_BUMPER_FENCE_RL)) {
            motorGroupSet (MOTORGROUP_WHEELS_L, 127);
            motorGroupSet (MOTORGROUP_WHEELS_R, 127);
        }
        else
            qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_RELEASE) {
        if (abs (clapperPot - clapperOpen) > 15)
            clapperToOpenness (clapperOpen);
        else
            qwikScoreMode += 1;
    }
}