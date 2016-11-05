#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Four-Bar
const int fourBarMax = 20;
const int fourBarFenceHigh = 50;
const int fourBarFenceLow = 75;
const int fourBarMin = 160;
int fourBarTarget = -1;
int fourBarPot = -1;
int fourBarP = 0;
int fourBarI = 0;

//Prongs
const int prongVertical = 100;
const int prongStorage = 75;
const int prongFlat = 50;
const int prongDrop = 25;
int prongTarget = -1;
int prongPot = -1;
int prongP = 0;
int prongI = 0;

///////////////////
//// Functions ////
///////////////////

//Equivalency to RobotC slaving and reversing of motors
void motorGroupSet(unsigned char motorGroup, int speed) {
    if (motorGroup == 1) {
        motorSet(MOTOR_WHEEL_LF, speed);
        motorSet(MOTOR_WHEEL_LB, speed);
    }
    if (motorGroup == 2) {
        motorSet(MOTOR_WHEEL_RF, -speed);
        motorSet(MOTOR_WHEEL_RB, -speed);
    }
    if (motorGroup == 3) {
        motorSet(MOTOR_FOURBAR_L, -speed);
        motorSet(MOTOR_FOURBAR_R, speed);
    }
}

//Four-bar PID control
void fourBarToHeight(int target) {
    
    //Read current sensor value
    fourBarPot = analogRead(SENSOR_FOURBAR_POT)/10;
    printf("4B: %d, ", fourBarPot);

    //PID control code
    const float pUp = 5;
    const float pDown = 0.25;
    const float i = 1;

    if (target != -1) {
        fourBarP = fourBarPot - target;
        if (abs(fourBarPot - target) > 3) {
            fourBarI += i;
        }
        else {
            fourBarI = 0;
        }

        if (target < fourBarPot) { //Needing to go up
            motorGroupSet(MOTORGROUP_FOURBAR, (pUp * fourBarP) + (fourBarI));
        }
        else if (target > fourBarPot) { //Needing to go down
            motorGroupSet(MOTORGROUP_FOURBAR, (pDown * fourBarP) - (fourBarI));
        }
    }
}


//Prong PID control
void prongToAngle(int target) {

    //Adjust target based on four-bar position
    target += (0 * fourBarPot);

    //Read current sensor value
    prongPot = analogRead(SENSOR_PRONG_POT)/10;
    print("PR: %d, ", prongPot - (0 * fourBarPot));
    
    //PID control code
    const float p = 1;
    const float i = 0;

    if (target != -1) {
        prongP = prongPot - target;
        if (abs(prongPot - target) > 3) {
            prongI += i;
        }
        else {
            prongI = 0;
        }

        if (target != prongPot) {
            motorSet(MOTOR_PRONGS, (p * prongP) + (prongI));
        }
    }
    printf("PR_MTR: %d", motorGet(MOTOR_PRONGS));
}