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
const int armFloorGrab = 50;
const int armNoMoreDown = 60;
const int armHoldCube = 80;
const int armScore = 235;
const int armNoMoreUp = 230;
int armTarget = -1;
double armKpUp = 3;
double armKpDown = 0.5;

//Clapper
const int clapperHold = 275;
const int clapperReady = 153;
const int clapperOpenWide = 136;
int clapperTarget = -1;
double clapperKp = 0.8;



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
unsigned char pid(double current, double target, double kp, double ki, double kd, const signed char *motors, double tolerance) {
    //Proportional
    double p = target - current;

    //Integral (eventually)


    //Derivative (eventually)


    motorsSlew(motors, kp * p);
    if (abs(p) <= tolerance) { return 1; } else { return 0; }
}

//Mechanism-specific PID loops
unsigned char armToAngle(short target) {
    if (target != -1) {
        return pid(CURRENT_ARM, target, (target > CURRENT_ARM) ? armKpUp : armKpDown, 0, 0, motorgroupArm, 20);
    } else {
        return 0;
    }
}
unsigned char clapperToOpenness(short target) {
    if (target == clapperHold) {
        motorsSlew(motorgroupClapper, 40);
        if (motorGet(MOTORS_CLAPPER) == 40) {
            return 1;
        }
        else {
            return 0;
        }
    } else if (target != -1) {
        return pid(CURRENT_CLAPPER, target, clapperKp, 0, 0, motorgroupClapper, 20);
    } else {
        return 0;
    }
}

//Autonomous optimization for functions
void autonWrapper(AutonWrappable *uno, AutonWrappable *dos, AutonWrappable *tres, AutonWrappable *cuatro, AutonWrappable *cinco) {
    unsigned char done = 0;
    while (done != 5) {
        done = 0;
        done += ((uno->fn)(uno->arg) == 1);
        done += ((dos->fn)(dos->arg) == 1);
        done += ((tres->fn)(tres->arg) == 1);
        done += ((cuatro->fn)(cuatro->arg) == 1);
        done += ((cinco->fn)(cinco->arg)== 1);
        wait(20);
    }
    ///TODO: Implement grouping system
}
unsigned char doNothing(short arbitraryValue) {
    return 1;
}
AutonWrappable autonDoNothing = {.fn = doNothing, .arg = 0, .group = 0};