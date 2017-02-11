/**
 * File for operator control code.
 *
 * This file should contain the user operatorControl() function and any functions related to it.
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

int armManual = 0;
int clapperManual = 0;
int clapperClamping = -1;
int pressed7D = 0;

/**
 * Runs the user operator control code.
 *
 * This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the operator control mode. If the robot is disabled or communications is lost, the operator control task will be stopped by the kernel. Re-enabling the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will run the operator control task. Be warned that this will also occur if the VEX Cortex is tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
    //Reset QwikScore
    qwikScoreMode = QWIKSCORE_INACTIVE;
    qwikScoreXtraIter = 0;

    while (1) {
        //Drivetrain
        motorsSlew(motorgroupWheelsL, abs(joystickGetAnalog(1, 3)) > 15 ? joystickGetAnalog(1, 3) : 0);
        motorsSlew(motorgroupWheelsR, abs(joystickGetAnalog(1, 2)) > 15 ? joystickGetAnalog(1, 2) : 0);

        //Arm
        if (joystickGetDigital(1, 6, JOY_UP)) {
            if (CURRENT_ARM < armNoMoreUp) {
                motorsSlew(motorgroupArm, 100);
                armTarget = CURRENT_ARM;
            } else {
                motorsSlew(motorgroupArm, 0);
                armTarget = armScore;
            }
            armManual = 1;
        } else if (joystickGetDigital(1, 6, JOY_DOWN)) {
            if (CURRENT_ARM > armNoMoreDown) {
                motorsSlew(motorgroupArm, -50);
                armTarget = CURRENT_ARM;
            } else {
                armTarget = armFloorGrab;
                armToAngle(armTarget);
            }
            armManual = 1;
        } else {
            armToAngle(armTarget);
            if (armManual && motorGet(MOTORS_ARM_LR_LOW) >= -15 && motorGet(MOTORS_ARM_LR_LOW) <= 15) {
                armTarget = CURRENT_ARM;
                armManual = 0;
            }
        }

        //Clapper
        if (joystickGetDigital(1, 7, JOY_DOWN) && !pressed7D) {
            clapperClamping = -clapperClamping;
            pressed7D = 1;
        }
        if (clapperClamping == 1) { //Serious clamping mode
            clapperToOpenness(clapperHold);
        } else if (joystickGetDigital(1, 5, JOY_DOWN)) { //Open
            motorsSlew(motorgroupClapper, -50);
            clapperTarget = CURRENT_CLAPPER;
            clapperManual = 1;
        } else if (joystickGetDigital(1, 5, JOY_UP)) { //Close
            motorsSlew(motorgroupClapper, 40);
            clapperTarget = CURRENT_CLAPPER;
            clapperManual = 1;
        } else {
            clapperToOpenness(clapperTarget);
            if (clapperManual && motorGet(MOTORS_CLAPPER) >= -15 && motorGet(MOTORS_CLAPPER) <= 15) {
                clapperTarget = CURRENT_CLAPPER;
                clapperManual = 0;
            }
        }
        if (!joystickGetDigital(1, 7, JOY_DOWN) && pressed7D) {
            pressed7D = 0;
        }

        //QwikScore
/*        while (joystickGetDigital (1, 7, JOY_DOWN)) {
            armTarget = armFloorGrab;
            clapperTarget = clapperReady;
            qwikScore (0);
            wait (10);
        }
        qwikScoreMode = QWIKSCORE_INACTIVE;
        qwikScoreXtraIter = 0;
        while (joystickGetDigital (1, 8, JOY_DOWN)) {
            qwikScore (1);
            wait (10);
        }
        qwikScoreMode = QWIKSCORE_INACTIVE;
        qwikScoreXtraIter = 0;*/

        if (millis() % 1000 < 20) {
            writeJINXDataNumeric("armCurrent", CURRENT_ARM);
            writeJINXDataNumeric("armTarget", armTarget);
            writeJINXDataNumeric("clapperCurrent", CURRENT_CLAPPER);
            writeJINXDataNumeric("clapperTarget", clapperTarget);
        }
        wait(20);
    }
}