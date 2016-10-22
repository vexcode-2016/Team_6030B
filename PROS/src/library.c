#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Four-Bar
const int fourBarMax = 10;
const int fourBarFenceHigh = 50;
const int fourBarFenceLow = 75;
const int fourBarMin = 144;
int fourBarPreset = -1;
int fourBarPotHist[6] = { -1, -1, -1, -1, -1, -1 };
int fourBarP = 0;
int fourBarI = 0;
int fourBarD = 0;

///////////////////
//// Functions ////
///////////////////

//Equivalency to RobotC slaving and reversing of motors
void motorGroupSet(int motorGroup, int speed) {
    if (motorGroup == 1) {
        motorSet(motorWheelLF, speed);
        motorSet(motorWheelLB, speed);
    }
    if (motorGroup == 2) {
        motorSet(motorWheelRF, -speed);
        motorSet(motorWheelRB, -speed);
    }
    if (motorGroup == 3) {
        motorSet(motorFourBarL, -speed);
        motorSet(motorFourBarR, speed);
    }
}

//Four-bar PID control
void fourBarToHeight(int target) {
    
    //Store brief history of potentiometer readings for derivative calculation
    for (int i = 5; i > 0; i -= 1) {
        fourBarPotHist[i] = fourBarPotHist[i - 1];
    }
    fourBarPotHist[0] = analogRead(sensorFourBarPot)/10;
    printf(",%d", fourBarPotHist[0]);

    //Actual PID control code
    if (target != -1) {
        const float pUp = 5;
        const float pDown = 0.25;
        const float i = 1;
        const float d = 0;

        fourBarP = fourBarPotHist[0] - target;
        if (abs(fourBarPotHist[0] - target) > 3) {
            fourBarI += 1;
        }
        else {
            fourBarI = 0;
        }
        if (fourBarPotHist[5] != -1) {
            fourBarD = (((fourBarPotHist[0] + fourBarPotHist[1] + fourBarPotHist[2]) / 3) - ((fourBarPotHist[3] + fourBarPotHist[4] + fourBarPotHist[5]) / 3));
        }
        else {
            fourBarD = 0;
        }

        printf(",%d", target);
        if (target < fourBarPotHist[0]) { //Needing to go up
            motorGroupSet(3, (pUp * fourBarP) + (i * fourBarI) - (d * fourBarD));
        }
        else if (target > fourBarPotHist[0]) { //Needing to go down
            motorGroupSet(3, (pDown * fourBarP) - (i * fourBarI) - (d * fourBarD));
        }
        printf(",%d", motorGet(motorFourBarR));
    }
}