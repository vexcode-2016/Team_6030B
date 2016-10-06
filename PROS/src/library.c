#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Four-Bar
const int fourBarMax = 4000;
const int fourBarFenceHigh = 3000;
const int fourBarFenceLow = 2000;
const int fourBarMin = 0;
int fourBarPreset = -1;
int fourBarPotHist[6] = { -1, -1, -1, -1, -1, -1 };
int fourBarP = 0;
int fourBarI = 0;
int fourBarD = 0;

///////////////////
//// Functions ////
///////////////////

//Equivalency to RobotC slaving and reversing of motors
void motorGroupSet(char motorGroup[], int speed) {
    print( strcat("Setting motors in ", motorGroup) );
    if (motorGroup == "wheelsL") {
        motorSet(motorWheelLF, speed);
        motorSet(motorWheelLB, speed);
    }
    if (motorGroup == "wheelsR") {
        motorSet(motorWheelRF, -speed);
        motorSet(motorWheelRB, -speed);
    }
    if (motorGroup == "fourBar") {
        motorSet(motorFourBarL, speed);
        motorSet(motorFourBarR, -speed);
    }
    if (motorGroup == "grabber") {
        motorSet(motorGrabberL, speed);
        motorSet(motorGrabberR, -speed);
    }
}

//Four-bar PID control
void fourBarToHeight(int height) {
    
    //Store brief history of potentiometer readings for derivative calculation
    for (int i = 5; i > 0; i -= 1) {
        fourBarPotHist[i] = fourBarPotHist[i - 1];
    }
    fourBarPotHist[0] = analogRead(sensorFourBarPot);
    print( strcat("Pot reading:", fourBarPotHist[0]) );

    //Actual PID control code
    if (height != -1) {
        const int p = 0.5;
        const int i = 0;
        const int d = 0;

        fourBarP = height - fourBarPotHist[0];
        if (abs(fourBarPotHist[0] - height) > 10) {
            fourBarI += 1;
        }
        else {
            fourBarI = 0;
        }
        if (fourBarPotHist[5] != -1) {
            fourBarD = ((fourBarPotHist[0] + fourBarPotHist[1] + fourBarPotHist[2]) / 3) - ((fourBarPotHist[3] + fourBarPotHist[4] + fourBarPotHist[5]) / 3);
        }
        else {
            fourBarD = 0;
        }

        motorGroupSet("fourBar", (p * fourBarP) + (i * fourBarI) - (d * fourBarD));
    }
}