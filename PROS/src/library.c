#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Slew rate control
int slewTarget[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int slewTmp;

//More sensible version of imeGet
int imeValue;

//Arm
const int armFloorGrab = 330;
const int armThrow = 155;
const int armFence = 165;
int armTarget = -1;
int armPot = -1;
int armP = 0;

//Clapper
const int clapperHold = 20;
const int clapperReady = 90;
const int clapperFence = 165;
int clapperTarget = -1;
int clapperPot = -1;
int clapperP = 0;

//IME nav
int driveLeftIME = 0;
int driveRightIME = 0;
int driveLeftP = 0;
int driveRightP = 0;

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
void motorGroupSlew (unsigned char motorGroup, int speed) {
    if (motorGroup == MOTORGROUP_WHEELS_L) {
        motorSlew(MOTOR_WHEEL_LF, speed);
        motorSlew(MOTOR_WHEEL_LB, speed);
    }
    if (motorGroup == MOTORGROUP_WHEELS_R) {
        motorSlew(MOTOR_WHEEL_RF, -speed);
        motorSlew(MOTOR_WHEEL_RB, -speed);
    }
    if (motorGroup == MOTORGROUP_ARM) {
        motorSlew(MOTORS_ARM_L, speed);
        motorSlew(MOTORS_ARM_R, -speed);
    }
    if (motorGroup == MOTORGROUP_CLAPPER) {
        motorSlew(MOTOR_CLAPPER_L, speed);
        motorSlew(MOTOR_CLAPPER_R, -speed);
    }
    if (motorGroup == MOTORGROUP_HANGER) {
        motorSlew(MOTOR_HANGER_L, -speed);
        motorSlew(MOTOR_HANGER_R, speed);
    }
}

//Slew rate control (run as task)
void slewControlTask (void * parameter) {
    while (1) {
        if( isEnabled() ){
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

//More sensible version of imeGet
int imeGetValue (unsigned char address) {
    if (!imeGet (address, &imeValue))
        imeValue = 0;
    return imeValue;
}

//Arm PID control
void armToAngle (int target) {
    armPot = analogRead(SENSOR_POT_ARM) / 10;

    if (target != -1) {
        armP = abs(target - armPot);

        if (armPot > target) //Up
            motorGroupSlew(MOTORGROUP_ARM, (0.9 * armP));
        else if (armPot < target) //Down
            motorGroupSlew(MOTORGROUP_ARM, -(0.25 * armP));
    }
}

//Clapper PID control
void clapperToOpenness (int target) {
    clapperPot = analogRead(SENSOR_POT_CLAPPER) / 10;

    if (target != -1) {
        clapperP = target - clapperPot;

        motorGroupSlew(MOTORGROUP_CLAPPER, 1 * clapperP);
    }
}

//Drivetrain PID control
void robotToPosition (int targetLeft, int targetRight) {
    driveLeftIME = imeGetValue (SENSOR_IME_WHEEL_LF);
    driveRightIME = -imeGetValue (SENSOR_IME_WHEEL_RF);

    driveLeftP = targetLeft - driveLeftIME;
    driveRightP = targetRight - driveRightIME;

    motorGroupSlew (MOTORGROUP_WHEELS_L, 0.2 * driveLeftP);
    motorGroupSlew (MOTORGROUP_WHEELS_R, 0.3 * driveRightP);
}

//QwikScore
void qwikScore(int autoDrive) {
    if (qwikScoreMode == QWIKSCORE_INACTIVE) {
        motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
        motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
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
            motorGroupSlew (MOTORGROUP_WHEELS_L, -1.8 * rotateP);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 1.8 * rotateP);
        }
        else {
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_DRIVE) {
        if (((digitalRead (SENSOR_BUMPER_LOW1) == HIGH) || (digitalRead (SENSOR_BUMPER_LOW2) == HIGH)) && ((digitalRead (SENSOR_BUMPER_HIGH1) == HIGH) || (digitalRead (SENSOR_BUMPER_HIGH2) == HIGH))) {
            motorGroupSlew (MOTORGROUP_WHEELS_L, -127);
            motorGroupSlew (MOTORGROUP_WHEELS_R, -127);
        }
        else {
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_THROW) {
        if ((analogRead(SENSOR_POT_ARM) / 10) > (armThrow + 30)) {
            clapperToOpenness (clapperHold);
            motorGroupSlew (MOTORGROUP_ARM, 127);
        }
        else if ((analogRead (SENSOR_POT_ARM) / 10) > armThrow) {
            motorGroupSlew (MOTORGROUP_CLAPPER, 127);
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
}

//End-of-match hanging (run as task)
void maintainHangTask (int motorSpeed) {
    while (1) {
        motorGroupSlew (MOTORGROUP_HANGER, motorSpeed);
        //wait (5);
    }
}