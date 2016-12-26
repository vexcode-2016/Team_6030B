#include "main.h"

///////////////////
//// Variables ////
///////////////////

//Slew rate control
int slewTarget[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int slewTmp;

//Arm
const int armFloorGrab = 330;
const int armThrow = 155;
const int armFence = 165;
int armTarget = -1;
int armPot = -1;
int armP = 0;

//Clapper
const int clapperHold = 20;
const int clapperReady = 90;
const int clapperFence = 165;
int clapperTarget = -1;
int clapperPot = -1;
int clapperP = 0;

//Inertial nav
int aLeft[3] = { 0, 0, 0 };
int aForward[3] = { 0, 0, 0 };
float theta;
int ax[2] = { 0, 0 };
int vx[2] = { 0, 0 };
int x = 0;
int ay[2] = { 0, 0 };
int vy[2] = { 0, 0 };
int y = 0;
Gyro gyro;
int gyroCalibration = 90;
int heading = 0;
int posCalibrating = 0;
int navTarget[3] = { -1, -1, -1 }; // { x, y, heading }
int driveP = 0;
int rotateP = 0;

//QwikScore
int qwikScoreMode = QWIKSCORE_INACTIVE;
int qwikScoreXtraIter = 0;



///////////////////
//// Functions ////
///////////////////

//Slew rate commanding - individual motors
void motorSlew (unsigned char channel, int speed) {
    slewTarget[channel - 1] = speed;
}

//Slew rate commanding - motor groups
void motorGroupSlew (unsigned char motorGroup, int speed) {
    if (motorGroup == MOTORGROUP_WHEELS_L) {
        motorSlew(MOTOR_WHEEL_LF, speed);
        motorSlew(MOTOR_WHEEL_LB, speed);
    }
    if (motorGroup == MOTORGROUP_WHEELS_R) {
        motorSlew(MOTOR_WHEEL_RF, -speed);
        motorSlew(MOTOR_WHEEL_RB, -speed);
    }
    if (motorGroup == MOTORGROUP_ARM) {
        motorSlew(MOTORS_ARM_L, speed);
        motorSlew(MOTORS_ARM_R, -speed);
    }
    if (motorGroup == MOTORGROUP_CLAPPER) {
        motorSlew(MOTOR_CLAPPER_L, speed);
        motorSlew(MOTOR_CLAPPER_R, -speed);
    }
    if (motorGroup == MOTORGROUP_HANGER) {
        motorSlew(MOTOR_HANGER_L, -speed);
        motorSlew(MOTOR_HANGER_R, speed);
    }
}

//Slew rate control (run as task)
void slewControlTask (void * parameter) {
    while (isEnabled ()) {
        for (int i = 0; i < 10; i++) {
            slewTmp = motorGet (i + 1);
            if (slewTmp != slewTarget[i]) {
                if (slewTmp < slewTarget[i]) {
                    slewTmp += 15;
                    if (slewTmp > slewTarget[i])
                        slewTmp = slewTarget[i];
                }
                if (slewTmp > slewTarget[i]) {
                    slewTmp -= 15;
                    if (slewTmp < slewTarget[i])
                        slewTmp = slewTarget[i];
                }
            }
            motorSet (i + 1, slewTmp);
        }
        wait (20);
    }
}

//Arm PID control
void armToAngle (int target) {
    //Read current sensor value
    armPot = analogRead(SENSOR_POT_ARM) / 10;
    //printf("Arm: %3d, ", armPot);

    if (target != -1) {
        armP = abs(target - armPot);

        if (armPot > target) { //Up
            motorGroupSlew(MOTORGROUP_ARM, (0.9 * armP));
        }
        else if (armPot < target) { //Down
            motorGroupSlew(MOTORGROUP_ARM, -(0.25 * armP));
        }

        //printf("Arm_MTR: %3d, ", motorGet(MOTORS_ARM_L));
    }
}

//Clapper PID control
void clapperToOpenness (int target) {
    //Read current sensor value
    clapperPot = analogRead(SENSOR_POT_CLAPPER) / 10;
    //printf("Clapper: %3d, ", clapperPot);

    if (target != -1) {
        clapperP = target - clapperPot;

        motorGroupSlew(MOTORGROUP_CLAPPER, 1 * clapperP);

        //printf("Clapper_MTR: %3d, ", motorGet(MOTOR_CLAPPER_L));
    }
}

//Inertial nav (run as task)
void inertialNavTask (void * parameter) {
    while (1) {
        wait (5);

        //Read and de-noise accelerations in robot axes
        for (int j = 1; j <= 99; j += 2) {
            for (int i = 1; i <= 99; i += 2) {
                aLeft[1] += analogRead (SENSOR_ACCEL_LX) + analogRead (SENSOR_ACCEL_RX);
                aForward[1] += analogRead (SENSOR_ACCEL_LY) + analogRead (SENSOR_ACCEL_RY);
            }
            aLeft[0] += (aLeft[1] / 100) - aLeft[2];
            aForward[0] += (aForward[1] / 100) - aForward[2];
            aLeft[1] = 0;
            aForward[1] = 0;
        }
        aLeft[0] = aLeft[0] / 300;
        aForward[0] = aForward[0] / 300;

        //Read heading
        heading = gyroGet (gyro) + gyroCalibration;
        heading = ((heading > 0) - (heading < 0)) * (abs (heading) % 360);
        if (heading < 0) heading += 360;

        //Heading & position recalibration
        if ((digitalRead (SENSOR_BUMPER_LF) == LOW) && (digitalRead (SENSOR_BUMPER_LB) == LOW)) {
            if (abs (heading - 90) < 15) {
                gyroCalibration += 90 - heading;
                posCalibrating = 1;
                x = 0;
            }
            else if (abs (heading - 180) < 15) {
                gyroCalibration += 180 - heading;
                posCalibrating = 1;
                y = 0;
            }
            else if (abs (heading - 270) < 15) {
                gyroCalibration += 270 - heading;
                //posCalibrating = 1;
                //x = MAX;
            }
        }
        if ((digitalRead (SENSOR_LIMIT_BL) == LOW) && (digitalRead (SENSOR_LIMIT_BR) == LOW)) {
            if ((heading < 15) || (heading > 345)) {
                gyroCalibration += 360 - heading;
                posCalibrating = 1;
                x = 0;
            }
            else if (abs (heading - 90) < 15) {
                gyroCalibration += 90 - heading;
                posCalibrating = 1;
                y = 0;
            }
            else if (abs (heading - 180) < 15) {
                gyroCalibration += 180 - heading;
                //posCalibrating = 1;
                //x = MAX;
            }
        }
        if ((digitalRead (SENSOR_BUMPER_RF) == LOW) && (digitalRead (SENSOR_BUMPER_RB) == LOW)) {
            if ((heading < 15) || (heading > 345)) {
                gyroCalibration += 360 - heading;
                posCalibrating = 1;
                y = 0;
            }
            else if (abs (heading - 90) < 15) {
                gyroCalibration += 90 - heading;
                //posCalibrating = 1;
                //x = MAX;
            }
            else if (abs (heading - 270) < 15) {
                gyroCalibration += 270 - heading;
                posCalibrating = 1;
                x = 0;
            }
        }

        //Re-read heading
        heading = gyroGet (gyro) + gyroCalibration;
        heading = ((heading > 0) - (heading < 0)) * (abs (heading) % 360);
        if (heading < 0) heading += 360;

        //Transform accelerations to field axes
        theta = 90 - heading;
        if (theta < 0) theta += 360;
        theta = theta * (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482133936072602491412737245870066063155881748815209209628292540917153643678925903600113305305488204665213841469519415116094330572703657595919530921861173819326117931051185480744623799627495673518857527248912279381830119491298336733624406566430860213949463952247371907021798609437027705392171762931767523846748184676694051320005681271452635608277857713427577896091736371787214684409012249534301465495853710507922796892589235420199561121290219608640344181598136297747713099605187072113499999983729780499510597317328160963185950244594553469083026425223082533446850352619311881710100031378387528865875332083814206171776691473035982534904287554687311595628638823537875937519577818577805321712268066130019278766111959092164201989 / 180);
        ax[1] = (sin (theta) * aForward[0]) + (cos (theta) * (-aLeft[0]));
        ay[1] = (cos (theta) * aForward[0]) + (sin (theta) * aLeft[0]);

        //Velocity recalibration & calculation
        if ((motorGet (MOTOR_WHEEL_LF) == 0) && (motorGet (MOTOR_WHEEL_RF) == 0) && (abs (motorGet (MOTORS_ARM_L)) <= 10) && (abs (motorGet (MOTOR_CLAPPER_L)) <= 10)) {
            vx[1] = 0;
            vy[1] = 0;
        }
        else {
            vx[1] = vx[0] + ax[0];
            vy[1] = vy[0] + ay[0];
        }

        //Position calculation
        if (!posCalibrating) {
            x += vx[0];
            y += vy[0];
        }
        if (x < 0) x = 0;
        //if (x > MAX) x = MAX;
        if (y < 0) y = 0;
        //if (y > MAX) y = MAX;
        
        //Reset for next iteration
        ax[0] = ax[1];
        ay[0] = ay[1];
        vx[0] = vx[1];
        vy[0] = vy[1];
        aLeft[0] = 0;
        aForward[0] = 0;
        posCalibrating = 0;
    }
}

//QwikScore
void qwikScore() {
    if (qwikScoreMode == QWIKSCORE_INACTIVE) {
        motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
        motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
        qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_GRAB) {
        if (qwikScoreXtraIter < 30) {
            clapperToOpenness (clapperHold);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_POSITION) {
        clapperToOpenness (clapperHold);
        navTarget[0] = -1;
        navTarget[1] = -1; //TODO: Update with actual y
        navTarget[2] = 270;
        rotateP = navTarget[2] - heading;
        if (heading < 90) rotateP -= 360;
        if (abs (rotateP) > 15) {
            motorGroupSlew (MOTORGROUP_WHEELS_L, -1.8 * rotateP);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 1.8 * rotateP);
        }
        else if ((navTarget[1] != -1) && (abs(navTarget[1] - x) >= 100)) {
            driveP = navTarget[1] - x;
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0.01 * driveP);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0.01 * driveP);
        }
        else {
            motorGroupSlew (MOTORGROUP_WHEELS_L, 0);
            motorGroupSlew (MOTORGROUP_WHEELS_R, 0);
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_THROW) {
        if ((analogRead(SENSOR_POT_ARM) / 10) > (armThrow + 30)) {
            clapperToOpenness (clapperHold);
            motorGroupSlew (MOTORGROUP_ARM, 127);
        }
        else if ((analogRead (SENSOR_POT_ARM) / 10) > armThrow) {
            motorGroupSlew (MOTORGROUP_CLAPPER, 127);
        }
        else {
            clapperTarget = clapperReady;
            armTarget = armFloorGrab;
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_DONE) {
        armToAngle (armTarget);
        clapperToOpenness (clapperTarget);
    }
}