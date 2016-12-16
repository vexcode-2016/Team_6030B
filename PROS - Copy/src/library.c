#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Autonomous
int autonMode;

//Arm
const int armFloorGrab = 370;
const int armHighest = 165;
const int armDrop = 142;
const int armFenceGrab = 87;
int armTarget = -1;
int armPot = -1;
int armP = 0;

//Clapper
const int clapperClosed = 8;
const int clapperStraight = 25;
const int clapperOpen = 70;
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
int qwikScoreXtraIter = 0;



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
    //printf("Arm: %3d, ", armPot);

    if (target != -1) {
        armP = abs(target - armPot);

        if ((armPot < armHighest) && (armPot < target)) { //Up - back side
            motorGroupSet(MOTORGROUP_ARM, -(0.75 * armP));
        }
        else if ((armPot > armHighest) && (armPot > target)) { //Up - front side
            motorGroupSet(MOTORGROUP_ARM, (0.9 * armP));
        }
        else if ((armPot < armHighest) && (armPot > target)) { //Down - back side
            motorGroupSet(MOTORGROUP_ARM, (0.25 * armP));
        }
        else if ((armPot > armHighest) && (armPot < target)) { //Down - front side
            motorGroupSet(MOTORGROUP_ARM, -(0.25 * armP));
        }

        //printf("Arm_MTR: %3d, ", motorGet(MOTORS_ARM_L));
    }
}

//Clapper PID control
void clapperToOpenness (int target) {
    //Read current sensor value
    clapperPot = analogRead(SENSOR_POT_CLAPPER) / 10;
    //printf("Clapper: %3d, ", clapperPot);

    if (target != -1) {
        clapperP = clapperPot - target;

        motorGroupSet(MOTORGROUP_CLAPPER, 1.25 * clapperP);

        //printf("Clapper_MTR: %3d, ", motorGet(MOTOR_CLAPPER_L));
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

    //printf ("Drive_MTR: %3d, ", motorGet (MOTOR_WHEEL_LF));
}

//Drive-rotate PID control
void rotateToHeading (int target) {
    //Read current sensor value
    rotateGyro = gyroGet (rotateGyroSensor);
    rotateGyro = ((rotateGyro > 0) - (rotateGyro < 0)) * (abs (rotateGyro) % 360);
    //printf ("Rotate: %3d, ", rotateGyro);

    if (target != -1) {
        rotateP = target - rotateGyro;

        motorGroupSet (MOTORGROUP_WHEELS_L, 1 * rotateP);
        motorGroupSet (MOTORGROUP_WHEELS_R, -1 * rotateP);

        //printf ("Rotate_MTR: %3d, ", motorGet (MOTOR_WHEEL_LF));
    }
}

//QwikScore
void qwikScore() {
    if (qwikScoreMode == QWIKSCORE_INACTIVE)
        qwikScoreMode += 1;
    if (qwikScoreMode == QWIKSCORE_GRAB) {
        if (abs (clapperPot - clapperClosed) > 30) {
            clapperToOpenness (clapperClosed);
        }
        else if (qwikScoreXtraIter < 5) {
            clapperToOpenness (clapperClosed);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_RAISE) {
        //print ("Shame, ");
        if (abs (armPot - armHighest) > 15) {
            armToAngle (armHighest);
        }
        else if (qwikScoreXtraIter < 5) {
            armToAngle (armHighest);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_ROTATE) {
        //print ("Hola, ");
        if (((abs (gyroGet (rotateGyroSensor))) % 360) - 180 > 10) {
            rotateToHeading (180);
        }
        else if (qwikScoreXtraIter < 5) {
            rotateToHeading (180);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_DRIVE) {
        if (digitalRead (SENSOR_BUMPER_FENCE_LH) && digitalRead (SENSOR_BUMPER_FENCE_LL) && digitalRead(SENSOR_BUMPER_FENCE_RH) && digitalRead(SENSOR_BUMPER_FENCE_RL)) {
            //print ("uno, ");
            motorGroupSet (MOTORGROUP_WHEELS_L, -127);
            motorGroupSet (MOTORGROUP_WHEELS_R, -127);
        }
        else {
            motorGroupSet (MOTORGROUP_WHEELS_L, 0);
            motorGroupSet (MOTORGROUP_WHEELS_R, 0);
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_ANGLE) {
        if (abs (armPot - armDrop) > 15) {
            armToAngle (armDrop);
        }
        else if (qwikScoreXtraIter < 5) {
            armToAngle (armDrop);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_RELEASE) {
        if (abs (clapperPot - clapperOpen) > 15) {
            clapperToOpenness (clapperOpen);
        }
        else {
            qwikScoreMode += 1;
        }
    }
    //print ("\n");
}