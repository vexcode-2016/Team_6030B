#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Four-Bar
const int fourBarMax = 380;
const int fourBarFenceHigh = 340;
const int fourBarFenceLow = 300;
const int fourBarMin = 215;
int fourBarTarget = -1;
int formerFourBarTarget = -1;
int fourBarPot = -1;
int fourBarP = 0;
int fourBarI = 0;

//Prongs
const int prongVertical = 90;
const int prongStorage = 180;
const int prongFlat = 230;
const int prongDrop = 300;
int prongTarget = -1;
int formerProngTarget = -1;
int prongPot = -1;
int prongP = 0;
int prongI = 0;

///////////////////
//// Functions ////
///////////////////

//Equivalency to RobotC slaving and reversing of motors
void motorGroupSet(unsigned char motorGroup, int speed) {
    if (motorGroup == MOTORGROUP_WHEELS_L) {
        motorSet(MOTOR_WHEEL_LF, speed);
        motorSet(MOTOR_WHEEL_LB, speed);
    }
    if (motorGroup == MOTORGROUP_WHEELS_R) {
        motorSet(MOTOR_WHEEL_RF, -speed);
        motorSet(MOTOR_WHEEL_RB, -speed);
    }
    if (motorGroup == MOTORGROUP_FOURBAR) {
        motorSet(MOTOR_FOURBAR_LT, -speed);
        motorSet(MOTOR_FOURBAR_LB, -speed);
        motorSet(MOTOR_FOURBAR_RT, speed);
        motorSet(MOTOR_FOURBAR_RB, speed);
    }
}

//Four-bar PID control
void fourBarToHeight(int target) {
    
    //Read current sensor value
    fourBarPot = analogRead(SENSOR_FOURBAR_POT)/10;
    printf("4B: %d, ", fourBarPot);

    //PID control code
    const float pUp = 2.75;
    const float pDown = 0.5;
    const float i = 0;

    if (target != -1) {
        fourBarP = target - fourBarPot;
        if (abs(fourBarP) > 3) {
            fourBarI += i;
        }
        else {
            fourBarI = 0;
        }

        if (target > fourBarPot) { //Needing to go up
            motorGroupSet(MOTORGROUP_FOURBAR, (pUp * fourBarP) + (fourBarI));
        }
        else if (target < fourBarPot) { //Needing to go down
            motorGroupSet(MOTORGROUP_FOURBAR, (pDown * fourBarP) - (fourBarI));
        }
        printf("4B_MTR: %d, ", motorGet(MOTOR_FOURBAR_RB));
    }
}


//Prong PID control
void prongToAngle(int target) {

    //Read current sensor value
    prongPot = analogRead(SENSOR_PRONG_POT)/10;
    printf("PR: %d, ", prongPot);
    
    //PID control code
    const float p = 1;
    const float i = 0.25;

    if (target != -1) {
        target -= 0.5*(fourBarPot - 215);
        printf("PR_TRG: %d, ", target);

        prongP = target - prongPot;
        if (abs(prongP) > 3) {
            prongI += i;
        }
        else if(target != formerProngTarget){
            prongI = 0;
            formerProngTarget = target;
        }

        if (target < prongPot) { //Needing to go up
            motorSet(MOTOR_PRONGS, (p * prongP) - (prongI));
        }
        else if (target > prongPot) { //Needing to go down
            motorSet(MOTOR_PRONGS, (p * prongP) + (prongI));
        }
        printf("PR_MTR: %d, ", motorGet(MOTOR_PRONGS));
    }
}