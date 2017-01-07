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
int i = 0;

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
    imeReset (SENSOR_IME_WHEEL_LF);
    imeReset (SENSOR_IME_WHEEL_RF);
    motorGroupSlew (MOTORGROUP_CLAPPER, 0);

    while (abs((analogRead (SENSOR_POT_ARM) / 10) - (armThrow - 70)) > 30) { //Free the arm
        print ("1\n");
        armToAngle (armThrow - 70);
        wait (10);
    }

    while (abs ((analogRead (SENSOR_POT_CLAPPER) / 10) - clapperReady) > 30) { //Ready the clapper
        print ("2\n");
        armToAngle (armThrow - 50);
        clapperToOpenness (clapperReady);
        wait (10);
    }

    if (digitalRead (JUMPER_SKILLS) == HIGH) { //No jumper in 11 = match autonomous (not programming skills)

        if (digitalRead (JUMPER_AUTON) == HIGH) { //No jumper in 12 = left starting tile

            while ((abs (imeGetValue (SENSOR_IME_WHEEL_LF) - 750) > 50) || (abs (-imeGetValue (SENSOR_IME_WHEEL_RF) - (0)) > 30)) { //Turn 45 deg right to pick up cube
                print ("3\n");
                armToAngle (armFence);
                clapperToOpenness (clapperReady);
                robotToPosition (750, 0);
                wait (10);
            }
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);

            while (abs ((analogRead (SENSOR_POT_ARM) / 10) - armFloorGrab) > 30) { //Get into ready position
                print ("4\n");
                armToAngle (armFloorGrab);
                clapperToOpenness (clapperReady);
                wait (10);
            }

            while ((abs (imeGetValue (SENSOR_IME_WHEEL_LF) - 2750) > 300) || (abs (-imeGetValue (SENSOR_IME_WHEEL_RF) - 1000) > 30)) { //Drive forward to grab cube
                print ("5\n");
                armToAngle (armFloorGrab);
                clapperToOpenness (clapperReady);
                robotToPosition (2750, 1000);
                wait (10);
            }
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);

            while (i < 30) { //Hold cube
                print ("6\n");
                armToAngle (armFloorGrab);
                clapperToOpenness (clapperHold);
                i++;
                wait (10);
            }
            i = 0;

            while (((abs (imeGetValue (SENSOR_IME_WHEEL_LF) - (2750)) > 100) || (abs (-imeGetValue (SENSOR_IME_WHEEL_RF) - (-300)) > 50)) && (i < 100)) { //Rotate in preparation for backing up
                print ("7\n");
                armToAngle (armFloorGrab - 50);
                clapperToOpenness (clapperHold);
                robotToPosition (2750, -300);
                i++;
                wait (10);
            }
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
            i = 0;

            while (i < 500) {
                print ("8\n");
                armToAngle (armFloorGrab - 50);
                clapperToOpenness (clapperHold);
                motorGroupSlew (MOTORGROUP_WHEELS_L, -127);
                motorGroupSlew (MOTORGROUP_WHEELS_R, -127);
                i++;
                wait (10);
            }
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
            i = 0;

            while (isAutonomous ()) {
                qwikScore (0);
                wait (10);
            }

        }
        else if (digitalRead (JUMPER_AUTON) == LOW) { //Jumper in 12 = right starting tile



        }

    }
    else if (digitalRead (JUMPER_SKILLS) == LOW) { //Jumper in 11 = programming skills (not match autonomous)

        if (digitalRead (JUMPER_AUTON) == HIGH) { //No jumper in 12 = left starting tile

            

        }
        else if (digitalRead (JUMPER_AUTON) == LOW) { //Jumper in 12 = right starting tile



        }

    }
}