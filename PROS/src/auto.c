/**
* File for autonomous code.
*
* This file should contain the user autonomous() function and any functions related to it.
*
* Copyright (c) 2011-2014, Purdue University ACM SIG BOTS. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of Purdue University ACM SIG BOTS nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be obtained from http://sourceforge.net/projects/freertos/files/ or on request.
********************************************************************************/

#include "main.h"

int autonStartTime;
unsigned long autonMillis() {
    return millis() - autonStartTime;
}
unsigned char driveShamefullyL(double directionAndStopTime) {
    int motorSpeed = ((directionAndStopTime > 0) - (directionAndStopTime < 0)) * (127);
    int stopTime = abs(directionAndStopTime);
    if (autonMillis() < stopTime) {
        motorsSlew(motorgroupWheelsL, motorSpeed);
    } else {
        motorsSlew(motorgroupWheelsL, 0);
        if (motorGet(MOTOR_WHEEL_LF) == 0)
            return 1;
    }
    return 0;
}
unsigned char driveShamefullyR(double directionAndStopTime) {
    int motorSpeed = ((directionAndStopTime > 0) - (directionAndStopTime < 0)) * (127);
    int stopTime = abs(directionAndStopTime);
    if (autonMillis() < stopTime) {
        motorsSlew(motorgroupWheelsR, motorSpeed);
    } else {
        motorsSlew(motorgroupWheelsR, 0);
        if (motorGet(MOTOR_WHEEL_RF) == 0)
            return 1;
    }
    return 0;
}
void autonTimerBased() {
    //Arm
    AutonWrappable autonArmReady = {.fn = armToAngle, .arg = armFloorGrab, .group = 1};
    AutonWrappable autonArmHoldCube = {.fn = armToAngle, .arg = armHoldCube, .group = 1};
    AutonWrappable autonArmScore = {.fn = armToAngle, .arg = armScore, .group = 1};

    //Clapper
    AutonWrappable autonClapperOpen = {.fn = clapperToOpenness, .arg = clapperOpenWide, .group = 2};
    AutonWrappable autonClapperHold = {.fn = clapperToOpenness, .arg = clapperHold, .group = 2};

    //Drivetrain (shamefully controlled)
    AutonWrappable autonDriveLToCube = {.fn = driveShamefullyL, .arg = 500, .group = 3};
    AutonWrappable autonDriveRToCube = {.fn = driveShamefullyR, .arg = 500, .group = 4};
    AutonWrappable autonDriveLTurnToScore = {.fn = driveShamefullyL, .arg = -1000, .group = 3};
    AutonWrappable autonDriveLBackUpToFence = {.fn = driveShamefullyL, .arg = -250, .group = 3};
    AutonWrappable autonDriveRBackUpToFence = {.fn = driveShamefullyR, .arg = -250, .group = 4};

    autonStartTime = millis();

    //           ARM                    CLAPPER                 LEFT WHEELS                     RIGHT WHEELS                    UNASSIGNED
    autonWrapper(&autonDoNothing,       &autonClapperOpen,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Deploy clapper
    autonWrapper(&autonDoNothing,       &autonClapperOpen,      &autonDriveLToCube,             &autonDriveRToCube,             &autonDoNothing); //Drive to cube
    autonWrapper(&autonDoNothing,       &autonClapperHold,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Grab cube
    autonWrapper(&autonArmHoldCube,     &autonClapperHold,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Raise arm so cube doesn't drag
    autonWrapper(&autonArmHoldCube,     &autonClapperHold,      &autonDriveLTurnToScore,        &autonDoNothing,                &autonDoNothing); //Turn to score
    autonWrapper(&autonArmHoldCube,     &autonClapperHold,      &autonDriveLBackUpToFence,      &autonDriveRBackUpToFence,      &autonDoNothing); //Back up to fence
    autonWrapper(&autonArmScore,        &autonClapperHold,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Raise arm
    autonWrapper(&autonArmScore,        &autonClapperOpen,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Drop cube
    autonWrapper(&autonArmReady,        &autonClapperOpen,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Lower arm
}

/**
* Runs the user autonomous code.
*
* This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart the task, not re-start it from where it left off.
*
* Code running in the autonomous task cannot access information from the VEX Joystick. However, the autonomous function can be invoked from another task if a VEX Competition Switch is not available, and it can access joystick information if called in this way.
*
* The autonomous task may exit, unlike operatorControl() which should never exit. If it does so, the robot will await a switch to another mode or disable/enable cycle.
*/
void autonomous() {
    if (digitalRead(JUMPER_SKILLS) == HIGH) { //No jumper in 11
        if (digitalRead(JUMPER_AUTON) == HIGH) { //No jumper in 12
            int startTime = millis();
            while (millis() - startTime < 2000) {
                clapperToOpenness(clapperOpenWide);
                wait(10);
            }
        } else if (digitalRead(JUMPER_AUTON) == LOW) { //Jumper in 12
            int startTime = millis();
            while (millis() - startTime < 2000) {
                clapperToOpenness(clapperOpenWide);
                wait(10);
            }
            while (millis() - startTime < 5000) {
                motorsSlew(motorgroupWheelsL, 127);
                motorsSlew(motorgroupWheelsR, 127);
                wait(10);
            }
            motorsSlew(motorgroupWheelsL, 0);
            motorsSlew(motorgroupWheelsR, 0);
        }
    } else if (digitalRead(JUMPER_SKILLS) == LOW) { //Jumper in 11

        if (digitalRead(JUMPER_AUTON) == HIGH) { //No jumper in 12

        } else if (digitalRead(JUMPER_AUTON) == LOW) { //Jumper in 12
            autonTimerBased();
        }

    }
}