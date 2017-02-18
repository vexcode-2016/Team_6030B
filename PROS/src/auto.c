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

int autonTimerStart;
unsigned long autonMillis() {
    return millis() - autonTimerStart;
}
unsigned long autonShame(short nada) {
    //printf("TIME: %d\n", millis() - autonTimerStart);
    return 1;
}
unsigned char driveShamefullyL(short directionAndStopTime) {
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
unsigned char driveShamefullyR(short directionAndStopTime) {
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
    AutonWrappable autonTimer = {.fn = autonShame,.arg = 1,.group = 5};

    //Arm
    AutonWrappable autonArmReady = {.fn = armToAngle, .arg = armFloorGrab, .group = 1};
    AutonWrappable autonArmHoldCube = {.fn = armToAngle, .arg = armHoldCube, .group = 1};
    AutonWrappable autonArmScore = {.fn = armToAngle, .arg = armScore, .group = 1};

    //Clapper
    AutonWrappable autonClapperOpen = {.fn = clapperToOpenness, .arg = clapperOpenWide, .group = 2};
    AutonWrappable autonClapperHold = {.fn = clapperToOpenness, .arg = clapperHold, .group = 2};

    //Drivetrain (shamefully controlled)
    AutonWrappable autonDriveLToCube = {.fn = driveShamefullyL, .arg = 1100, .group = 3};
    AutonWrappable autonDriveRToCube = {.fn = driveShamefullyR, .arg = 1100, .group = 4};
    AutonWrappable autonDriveLTurnToScore = {.fn = driveShamefullyL, .arg = -1000, .group = 3};
    AutonWrappable autonDriveLBackUpToFence = {.fn = driveShamefullyL, .arg = -2500, .group = 3};
    AutonWrappable autonDriveRBackUpToFence = {.fn = driveShamefullyR, .arg = -2500, .group = 4};

    //           ARM                    CLAPPER                 LEFT WHEELS                     RIGHT WHEELS                    UNASSIGNED
    autonTimerStart = millis();
    autonWrapper(&autonDoNothing,       &autonClapperOpen,      &autonDriveLToCube,             &autonDriveRToCube,             &autonTimer); //Drive to cube
    autonWrapper(&autonDoNothing,       &autonClapperHold,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Grab cube
    print("UNO");
    autonTimerStart = millis();
    autonWrapper(&autonDoNothing,       &autonClapperHold,      &autonDriveLTurnToScore,        &autonDoNothing,                &autonTimer); //Turn to score
    print("DOS");
    autonTimerStart = millis();
    autonWrapper(&autonDoNothing,       &autonClapperHold,      &autonDriveLBackUpToFence,      &autonDriveRBackUpToFence,      &autonTimer); //Back up to fence
    print("TRES");
    autonWrapper(&autonArmScore,        &autonClapperHold,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Raise arm
    print("CUATRO");
    autonWrapper(&autonArmScore,        &autonClapperOpen,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Drop cube
    print("CINCO");
    autonWrapper(&autonArmReady,        &autonClapperOpen,      &autonDoNothing,                &autonDoNothing,                &autonDoNothing); //Lower arm
    print("SEIS");
}
void autonZackTimer() {
    AutonWrappable autonArmScore = {.fn = armToAngle,.arg = armScore,.group = 1};

    clapperToOpenness(clapperOpenWide);//deploy claw
    wait(500);
    clapperToOpenness(clapperOpenWide);

    motorsSlew(motorgroupWheelsL, 127);//drive to cube
    motorsSlew(motorgroupWheelsR, 127);
    wait(1100);
    motorsSlew(motorgroupWheelsL, 0);
    motorsSlew(motorgroupWheelsR, 0);

    motorsSlew(motorgroupClapper, 60);//grab cube
    wait(250);

    //armToAngle(armHoldCube);//raise arme so cube doesnt drag
    //wait(250);
    
    motorsSlew(motorgroupWheelsL, 100);//turn to score
    motorsSlew(motorgroupWheelsR, -100);
    wait(800);

    motorsSlew(motorgroupWheelsL, -100);
    motorsSlew(motorgroupWheelsR, -100);//back up to fence
    wait(1200);

    motorsSlew(motorgroupWheelsL, 0);
    motorsSlew(motorgroupWheelsR, 0);
    wait(250);

    int autonShame = millis();
    while (millis() - autonShame < 1500) {
        armToAngle(armScore);
        wait(10);
    }
    while (millis() - autonShame < 3000) {
        armToAngle(armScore);
        clapperToOpenness(clapperOpenWide);
        wait(10);
    }
    motorsSlew(motorgroupClapper, 0);
    while (millis() - autonShame < 4500) {
        armToAngle(armFloorGrab);
        wait(10);
    }
    motorsSlew(motorgroupArm, 0);
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
            autonZackTimer();
        } else if (digitalRead(JUMPER_AUTON) == LOW) { //Jumper in 12
            autonTimerBased();
        }

    }
}