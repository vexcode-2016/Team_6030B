#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Slew rate control
int slewTarget[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int slewTmp;

//Arm
const int armFloorGrab = 47;
const int armNoMoreDown = 82;
const int armScore = 235;
const int armNoMoreUp = 180;
int armTarget = -1;
int armPot = -1;
double armKpUp = 2;
double armKpDown = 0;
int armP = 0;

//Clapper
const int clapperHold = 20;
const int clapperReady = 90;
const int clapperFence = 165;
const int clapperBack = 345;
int clapperTarget = -1;
int clapperPot = -1;
double clapperKp = 0;
int clapperP = 0;

//QwikScore
int qwikScoreMode = QWIKSCORE_INACTIVE;
int qwikScoreXtraIter = 0;
Gyro gyro;
int heading = 0;
int rotateP = 0;



///////////////////
//// Functions ////
///////////////////

//Slew rate commanding - individual motors
void motorSlew (unsigned char channel, int speed) {
    slewTarget[channel - 1] = speed;
}

//Slew rate commanding - motor groups
void motorGroupSet (unsigned char motorGroup, int speed) {
    if (motorGroup == MOTORGROUP_WHEELS_L) {
        motorSlew (MOTOR_WHEEL_LF, speed);
        motorSlew (MOTOR_WHEEL_LB, speed);
    }
    if (motorGroup == MOTORGROUP_WHEELS_R) {
        motorSlew (MOTOR_WHEEL_RF, -speed);
        motorSlew (MOTOR_WHEEL_RB, -speed);
    }
    if (motorGroup == MOTORGROUP_ARM) {
        motorSlew (MOTORS_ARM_L_HIGH, speed);
        motorSlew (MOTORS_ARM_R_HIGH, -speed);
        motorSlew (MOTORS_ARM_LR_LOW, speed);
    }
    if (motorGroup == MOTORGROUP_CLAPPER) {
        motorSlew (MOTORS_CLAPPER, speed);
    }
}

//Slew rate control (run as task)
void slewControlTask (void * parameter) {
    while (1) {
        if (isEnabled ()) {
            for (int i = 0; i < 10; i++) { //Cycle through each motor port
                slewTmp = motorGet (i + 1);
                if (slewTmp != slewTarget[i]) {
                    if (slewTmp < slewTarget[i]) {
                        slewTmp += 15;
                        if (slewTmp > slewTarget[i])
                            slewTmp = slewTarget[i];
                    }
                    if (slewTmp > slewTarget[i]) {
                        slewTmp -= 15;
                        if (slewTmp < slewTarget[i])
                            slewTmp = slewTarget[i];
                    }
                }
                motorSet (i + 1, slewTmp);
            }
        }
        wait (20);
    }
}

//Arm PID control
void armToAngle (int target) {
    armPot = analogRead (SENSOR_POT_ARM) / 10;

    if (target != -1) {
        if (armPot != 25)
            armP = target - armPot;

        if (armP >= 0) //Up
            motorGroupSet (MOTORGROUP_ARM, armKpUp * armP);
        else if (armP < 0) //Down
            motorGroupSet (MOTORGROUP_ARM, armKpDown * armP);
    }
    else {
        motorGroupSet (MOTORGROUP_ARM, 0);
    }
}

//Clapper PID control
void clapperToOpenness (int target) {
    clapperPot = analogRead(SENSOR_POT_CLAPPER) / 10;

    if (target != -1) {
        clapperP = target - clapperPot;

        motorGroupSet (MOTORGROUP_CLAPPER, clapperKp * clapperP);
    }
    else {
        motorGroupSet (MOTORGROUP_CLAPPER, 0);
    }
}

//QwikScore
/*void qwikScore(int autoDrive) {
    if (qwikScoreMode == QWIKSCORE_INACTIVE) {
        motorGroupSet (MOTORGROUP_WHEELS_L, 0);
        motorGroupSet (MOTORGROUP_WHEELS_R, 0);
        qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_GRAB) {
        if (qwikScoreXtraIter < 30) {
            clapperToOpenness (clapperHold);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            if (autoDrive)
                qwikScoreMode += 1;
            else
                qwikScoreMode += 3;
        }
    }
    if (qwikScoreMode == QWIKSCORE_ROTATE) {
        clapperToOpenness (clapperHold);
        heading = gyroGet (gyro);
        heading = ((heading > 0) - (heading < 0)) * (abs (heading) % 360);
        if (heading < 0) heading += 360;
        rotateP = 180 - heading;
        if (abs (rotateP) > 15) {
            motorGroupSet (MOTORGROUP_WHEELS_L, -1.8 * rotateP);
            motorGroupSet (MOTORGROUP_WHEELS_R, 1.8 * rotateP);
        }
        else {
            motorGroupSet (MOTORGROUP_WHEELS_L, 0);
            motorGroupSet (MOTORGROUP_WHEELS_R, 0);
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_DRIVE) {
        
    }
    if (qwikScoreMode == QWIKSCORE_THROW) {
        if ((analogRead(SENSOR_POT_ARM) / 10) > (armThrow + 30)) {
            clapperToOpenness (clapperHold);
            motorGroupSet (MOTORGROUP_ARM, 127);
        }
        else if ((analogRead (SENSOR_POT_ARM) / 10) > armThrow) {
            motorGroupSet (MOTORGROUP_CLAPPER, 127);
        }
        else {
            clapperTarget = clapperReady;
            armTarget = armFloorGrab;
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_DONE) {
        armToAngle (armTarget);
        clapperToOpenness (clapperTarget);
    }
}*/