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

unsigned long autonTimerStart;
void autonTimerReset() {
    autonTimerStart = millis();
}
unsigned long autonTimerGet() {
    return millis() - autonTimerStart;
}
unsigned char driveShamefullyL(float directionAndStopTime) {
    int motorSpeed = ((directionAndStopTime > 0) - (directionAndStopTime < 0)) * (127);
    int stopTime = abs(directionAndStopTime);
    if (autonTimerGet() < stopTime) {
        motorsSlew(motorgroupWheelsL, motorSpeed);
    } else {
        motorsSlew(motorgroupWheelsL, 0);
        if (motorGet(MOTOR_WHEEL_LF) == 0)
            return 1;
    }
    return 0;
}
unsigned char driveShamefullyR(float directionAndStopTime) {
    int motorSpeed = ((directionAndStopTime > 0) - (directionAndStopTime < 0)) * (127);
    int stopTime = abs(directionAndStopTime);
    if (autonTimerGet() < stopTime) {
        motorsSlew(motorgroupWheelsR, motorSpeed);
    } else {
        motorsSlew(motorgroupWheelsR, 0);
        if (motorGet(MOTOR_WHEEL_RF) == 0)
            return 1;
    }
    return 0;
}
void autonTimerBased(int leftPosRightNeg) {
    //Arm
    AutonWrappable autonArmReady = {.fn = armToAngle, .arg = armFloorGrab, .group = 1};
    AutonWrappable autonArmHoldCube = {.fn = armHoldCube, .arg = 0, .group = 1};
    AutonWrappable autonArmScore = {.fn = armToAngle, .arg = armScore, .group = 1};

    //Clapper
    AutonWrappable autonClapperOpen = {.fn = clapperToOpenness, .arg = clapperOpenWide, .group = 2};
    AutonWrappable autonClapperHold = {.fn = clapperToOpenness, .arg = clapperHold, .group = 2};

    //Drivetrain (shamefully controlled)
    AutonWrappable autonDriveLToCube = {.fn = driveShamefullyL, .arg = 1100, .group = 3};
    AutonWrappable autonDriveRToCube = {.fn = driveShamefullyR, .arg = 1100, .group = 4};
    AutonWrappable autonDriveLTurnToScore = {.fn = driveShamefullyL, .arg = leftPosRightNeg * 800, .group = 3};
    AutonWrappable autonDriveRTurnToScore = {.fn = driveShamefullyR, .arg = -leftPosRightNeg * 800,.group = 3};
    AutonWrappable autonDriveLBackUpToFence = {.fn = driveShamefullyL, .arg = -1200, .group = 3};
    AutonWrappable autonDriveRBackUpToFence = {.fn = driveShamefullyR, .arg = -1200, .group = 4};

    //           ARM                   CLAPPER                LEFT WHEELS                    RIGHT WHEELS                   UNASSIGNED
    autonWrapper(autonArmScore,        autonDoNothing,        autonDoNothing,                autonDoNothing,                autonDoNothing); //Lift arm to deploy clapper
    autonWrapper(autonArmScore,        autonClapperOpen,      autonDoNothing,                autonDoNothing,                autonDoNothing); //Deploy clapper
    autonTimerReset();
    autonWrapper(autonArmReady,        autonClapperOpen,      autonDriveLToCube,             autonDriveRToCube,             autonDoNothing); //Drive to cube
    for (int i = 0; i < 10; i++) {
        print("SHAME0");
        wait(5);
    }
    autonWrapper(autonArmReady,        autonClapperHold,      autonDoNothing,                autonDoNothing,                autonDoNothing); //Grab cube
    for (int i = 0; i < 10; i++) {
        print("SHAME1");
        wait(5);
    }
    autonTimerReset();
    autonWrapper(autonArmHoldCube,     autonClapperHold,      autonDriveLTurnToScore,        autonDriveRTurnToScore,        autonDoNothing); //Turn to score
    for (int i = 0; i < 10; i++) {
        print("SHAME2");
        wait(5);
    }
    autonTimerReset();
    autonWrapper(autonArmHoldCube,     autonClapperHold,      autonDriveLBackUpToFence,      autonDriveRBackUpToFence,      autonDoNothing); //Back up to fence
    autonWrapper(autonArmScore,        autonClapperHold,      autonDoNothing,                autonDoNothing,                autonDoNothing); //Raise arm
    autonWrapper(autonArmScore,        autonClapperOpen,      autonDoNothing,                autonDoNothing,                autonDoNothing); //Drop cube
    autonWrapper(autonArmReady,        autonClapperOpen,      autonDoNothing,                autonDoNothing,                autonDoNothing); //Lower arm
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
    motorsSlew(motorgroupWheelsL, 0);
    motorsSlew(motorgroupWheelsR, 0);
    motorsSlew(motorgroupArm, 0);
    motorsSlew(motorgroupClapper, 0);
    motorStopAll();

    if ((digitalRead(11) == HIGH) && (digitalRead(11) == HIGH)) { //No jumper in 11 or 12

    }
    if ((digitalRead(11) == LOW) && (digitalRead(12) == HIGH)) { //Jumper in 11 only
        autonTimerBased(1);
    }
    if ((digitalRead(11) == HIGH) && (digitalRead(12) == LOW)) { //Jumper in 12 only
        autonTimerBased(-1);
    }
    if ((digitalRead(11) == LOW) && (digitalRead(12) == LOW)) { //Jumpers in 11 and 12
        AutonWrappable autonArmScore = {.fn = armToAngle,.arg = armScore,.group = 1};
        AutonWrappable autonClapperOpen = {.fn = clapperToOpenness,.arg = clapperOpenWide,.group = 2};

        motorsSlew(motorgroupWheelsL, -100);
        motorsSlew(motorgroupWheelsR, -100);
        wait(2000);
        motorsSlew(motorgroupWheelsL, 0);
        motorsSlew(motorgroupWheelsR, 0);

        autonWrapper(autonArmScore, autonDoNothing, autonDoNothing, autonDoNothing, autonDoNothing);
        autonWrapper(autonArmScore, autonClapperOpen, autonDoNothing, autonDoNothing, autonDoNothing);

    }
}