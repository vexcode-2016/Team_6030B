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

///////////////////
//// Variables ////
///////////////////

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
    while (1) {
        
        //Drivetrain
        if (abs(joystickGetAnalog(2, 3)) > 15) {
            motorGroupSet(MOTORGROUP_WHEELS_L, joystickGetAnalog(2, 3)*0.25);
        }
        else if (abs(joystickGetAnalog(1, 3)) > 15) {
            motorGroupSet(MOTORGROUP_WHEELS_L, joystickGetAnalog(1, 3));
        }
        else {
            motorGroupSet(MOTORGROUP_WHEELS_L, 0);
        }
        if (abs(joystickGetAnalog(2, 2)) > 15) {
            motorGroupSet(MOTORGROUP_WHEELS_R, joystickGetAnalog(2, 2)*0.25);
        }
        else if (abs(joystickGetAnalog(1, 2)) > 15) {
            motorGroupSet(MOTORGROUP_WHEELS_R, joystickGetAnalog(1, 2));
        }
        else {
            motorGroupSet(MOTORGROUP_WHEELS_R, 0);
        }
        if (joystickGetDigital(1, 5, JOY_UP) || joystickGetDigital(1, 5, JOY_DOWN) || joystickGetDigital(2, 7, JOY_LEFT)) {
            motorSet(MOTOR_WHEEL_M, 127);
        }
        else if (joystickGetDigital(1, 6, JOY_UP) || joystickGetDigital(1, 6, JOY_DOWN) || joystickGetDigital(2, 7, JOY_RIGHT)) {
            motorSet(MOTOR_WHEEL_M, -127);
        }
        else {
            motorSet(MOTOR_WHEEL_M, 0);
        }

        //Four-bar height presets
        if (joystickGetDigital(2, 8, JOY_UP)) {
            fourBarTarget = fourBarMax;
        }
        else if (joystickGetDigital(2, 8, JOY_RIGHT)) {
            fourBarTarget = fourBarFenceHigh;
        }
        else if (joystickGetDigital(2, 8, JOY_LEFT)) {
            fourBarTarget = fourBarFenceLow;
        }
        else if (joystickGetDigital(2, 8, JOY_DOWN)) {
            fourBarTarget = fourBarMin;
        }

        //Four-bar control
        if (joystickGetDigital(2, 6, JOY_UP)) {
            motorGroupSet(MOTORGROUP_FOURBAR, 64);
            fourBarTarget = analogRead(SENSOR_FOURBAR_POT)/10;
        }
        else if (joystickGetDigital(2, 6, JOY_DOWN)) {
            motorGroupSet(MOTORGROUP_FOURBAR, -64);
            fourBarTarget = analogRead(SENSOR_FOURBAR_POT)/10;
        }
        else if (joystickGetDigital(1, 8, JOY_UP)) {
            motorGroupSet(MOTORGROUP_FOURBAR, 0);
        }
        else {
            fourBarToHeight(fourBarTarget);
        }

        //Prongs
        if (joystickGetDigital(2, 5, JOY_DOWN)){
            prongTarget = prongDrop;
		}
        else if (joystickGetDigital(2, 5, JOY_UP)) {
            prongTarget = prongFlat;
        }
        else if (joystickGetDigital(2, 7, JOY_DOWN)) {
            prongTarget = prongStorage;
        }
        else if (joystickGetDigital(2, 7, JOY_UP)) {
            prongTarget = prongVertical;
        }
        prongToAngle(prongTarget);

        print("\n");
        delay(25);
    }
}