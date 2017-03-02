#include "main.h"
#include <math.h>

#define UPDATE_RATE 5 //milliseconds

//Explicit complementary filter
Quaternion attitude = {.w = 1, .x = 0, .y = 0, .z = 0};
Vector ecfErrorAccumulated = {.x = 0, .y = 0, .z = 0};
Quaternion ecfDerivativeAccumulated = {.w = 0, .x = 0, .y = 0, .z = 0};
void ECF() {
    double kp = 1;
    double ki = 1;

    //Read from sensor
    Vector gyro = {.x = 0, .y = 0, .z = analogReadCalibrated(SENSOR_GYRO_YAW)};

    //Assumed actual gravity direction
    Vector gravDir = {.x = 0, .y = 0, .z = -1};

    //Estimate gravity direction using current quaternion
    Vector gravDirEst = {.x = 2 * ((attitude.x * attitude.z) + (attitude.w * attitude.y)), .y = 2 * ((attitude.y * attitude.z) + (attitude.w * attitude.x)), .z = pow(attitude.w, 2) - pow(attitude.x, 2) - pow(attitude.y, 2) + pow(attitude.z, 2)};

    //Error between gyro-estimated gravity direction and assumed actual
    Vector error = vectorCrossProduct(gravDirEst, gravDir);

    //Data fusion
    Vector fused = vectorSum(vectorSum(gyro, vectorCoeff(kp, error)), vectorCoeff(ki, vectorIntegrate(error, UPDATE_RATE / 1000, &ecfErrorAccumulated)));

    //Rate of change of quaternion
    Quaternion derivative = quatDifferentiate(fused, attitude);

    //Estimate attitude
    attitude = quatNormalize(quatIntegrate(derivative, UPDATE_RATE / 1000, &ecfDerivativeAccumulated));
}

//Zero velocity update
Vector velocity = {.x = 0, .y = 0, .z = 0};
Vector position = {.x = 0, .y = 0, .z = 0};
Vector zuptAccelAccumulated = {.x = 0, .y = 0, .z = 0};
Vector zuptVelocityAccumulated = {.x = 0, .y = 0, .z = 0};
void ZUPT() {
    //Read from sensor
    Vector accel = accelRead();

    //Integrate acceleration
    vectorIntegrate(accel, UPDATE_RATE / 1000, &zuptAccelAccumulated);

    //Check for actual motion
    if (abs(motorGet(MOTOR_WHEEL_LF)) < 10 && abs(motorGet(MOTOR_WHEEL_RF)) < 10) {
        zuptAccelAccumulated.x = 0;
        zuptAccelAccumulated.y = 0;
        zuptAccelAccumulated.z = 0;
    }

    velocity = zuptAccelAccumulated;

    //Integrate velocity to position
    position = vectorIntegrate(velocity, UPDATE_RATE / 1000, &zuptVelocityAccumulated);
}

//Background task
void inertialNavTask(void * parameter) {
    while (1) {
        ECF();
        ZUPT();
        if (millis() % 1000 < 5 && joystickGetDigital(1, 8, JOY_DOWN)) {
            printf("[NAV] Att :: ( %5f , %5f , %5f , %5f )\n", attitude.w, attitude.x, attitude.y, attitude.z);
            printf("[NAV] Vel :: ( %5f , %5f , %5f )\n", velocity.x, velocity.y, velocity.z);
            printf("[NAV] Pos :: ( %5f , %5f , %5f )\n", position.x, position.y, position.z);
        }
        wait(UPDATE_RATE);
    }
}