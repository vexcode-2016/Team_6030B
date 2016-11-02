#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Four-Bar
const int fourBarMax = 20;
const int fourBarFenceHigh = 50;
const int fourBarFenceLow = 75;
const int fourBarMin = 160;
int fourBarPreset = -1;
int fourBarPot = -1;
int fourBarP = 0;
int fourBarI = 0;
int fourBarD = 0;

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
    printf(",%d", fourBarPot);

    //PID control code
    const float pUp = 5;
    const float pDown = 0.25;
    const float i = 1;

    fourBarP = fourBarPot - target;
    if (abs(fourBarPot - target) > 3) {
        fourBarI += 1;
    }
    else {
        fourBarI = 0;
    }

    printf(",%d", target);
    if (target < fourBarPot) { //Needing to go up
        motorGroupSet(3, (pUp * fourBarP) + (i * fourBarI));
    }
    else if (target > fourBarPot) { //Needing to go down
        motorGroupSet(3, (pDown * fourBarP) - (i * fourBarI));
    }
    printf(",%d", motorGet(MOTOR_FOURBAR_R));
}