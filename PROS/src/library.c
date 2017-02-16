#include "main.h"

//////////////////////
//// Motor Groups ////
//////////////////////
const signed char motorgroupWheelsL[] = {MOTOR_WHEEL_LF, MOTOR_WHEEL_LB, 20};
const signed char motorgroupWheelsR[] = {-MOTOR_WHEEL_RF, -MOTOR_WHEEL_RB, 20};
const signed char motorgroupArm[] = {MOTORS_ARM_L_HIGH, -MOTORS_ARM_R_HIGH, MOTORS_ARM_LR_LOW, 20};
const signed char motorgroupClapper[] = {-MOTORS_CLAPPER, 20};



////////////////////////////////
//// [Meaningful] Variables ////
////////////////////////////////

//Slew rate control
int slewTarget[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int slewTmp;

//Arm
const int armFloorGrab = 47;
const int armHoldCube = 75;
const int armNoMoreDown = 60;
const int armScore = 235;
const int armNoMoreUp = 200;
int armTarget = -1;
double armKpUp = 3;
double armKpDown = 0.5;

//Clapper
const int clapperHold = 275;
const int clapperReady = 153;
const int clapperOpenWide = 136;
int clapperTarget = -1;
double clapperKp = 0.8;

//QwikScore
int qwikScoreMode = QWIKSCORE_INACTIVE;
int qwikScoreXtraIter = 0;
int heading = 0;
int rotateP = 0;



///////////////////
//// Functions ////
///////////////////

//Slew rate commanding
void motorsSlew(const signed char *ports, int speed) {
    signed char port;
    int i = 0;
    while(ports[i] != 20){
        port = ports[i];
        if ((port >= 1) && (port <= 10)) { //1 <= port <= 10
            slewTarget[port - 1] = speed;
        } else if ((port >= -10) && (port <= -1)) { //-10 <= port <= -1
            slewTarget[-port - 1] = -speed;
        }
        i++;
    }
}

//Slew rate control (run as task)
void slewControlTask(void * parameter) {
    while (1) {
        if (isEnabled()) {
            for (int i = 0; i < 10; i++) { //Cycle through each motor port
                slewTmp = motorGet(i + 1);
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
                motorSet(i + 1, slewTmp);
            }
        }
        wait(20);
    }
}

//Generic PID control
void pid(double current, double target, double kp, double ki, double kd, const signed char *motors) {
    //Proportional
    double p = target - current;

    //Integral (eventually)


    //Derivative (eventually)


    motorsSlew(motors, kp * p);
}

//QwikScore
/*void qwikScore(int autoDrive) {
    if (qwikScoreMode == QWIKSCORE_INACTIVE) {
        motorsSlew (motorgroupWheelsL, 0);
        motorsSlew (motorgroupWheelsR, 0);
        qwikScoreMode += 1;
    }
    if (qwikScoreMode == QWIKSCORE_GRAB) {
        if (qwikScoreXtraIter < 30) {
            clapperToOpenness (clapperHold);
            qwikScoreXtraIter += 1;
        }
        else {
            qwikScoreXtraIter = 0;
            if (autoDrive)
                qwikScoreMode += 1;
            else
                qwikScoreMode += 3;
        }
    }
    if (qwikScoreMode == QWIKSCORE_ROTATE) {
        clapperToOpenness (clapperHold);
        heading = gyroGet (gyro);
        heading = ((heading > 0) - (heading < 0)) * (abs (heading) % 360);
        if (heading < 0) heading += 360;
        rotateP = 180 - heading;
        if (abs (rotateP) > 15) {
            motorsSlew (motorgroupWheelsL, -1.8 * rotateP);
            motorsSlew (motorgroupWheelsR, 1.8 * rotateP);
        }
        else {
            motorsSlew (motorgroupWheelsL, 0);
            motorsSlew (motorgroupWheelsR, 0);
            qwikScoreXtraIter = 0;
            qwikScoreMode += 1;
        }
    }
    if (qwikScoreMode == QWIKSCORE_DRIVE) {

    }
    if (qwikScoreMode == QWIKSCORE_THROW) {
        if ((analogRead(SENSOR_POT_ARM) / 10) > (armThrow + 30)) {
            clapperToOpenness (clapperHold);
            motorsSlew (motorgroupArm, 127);
        }
        else if ((analogRead (SENSOR_POT_ARM) / 10) > armThrow) {
            motorsSlew (motorgroupClapper, 127);
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
}*/
