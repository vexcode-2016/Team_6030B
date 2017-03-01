#include "main.h"
#include <math.h>

//Explicit complementary filter
Quaternion attitude = {.w = 1, .x = 0, .y = 0, .z = 0};
Vector errorAccumulated = {.x = 0, .y = 0, .z = 0};
Quaternion derivativeAccumulated = {.w = 0, .x = 0, .y = 0, .z = 0};
Quaternion ECF() {
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
    Vector fused = vectorSum(vectorSum(gyro, vectorCoeff(kp, error)), vectorCoeff(ki, vectorIntegrate(error, &errorAccumulated)));

    //Rate of change of quaternion
    Quaternion derivative = quatDifferentiate(fused, attitude);

    //Estimate attitude
    attitude = quatNormalize(quatIntegrate(derivative, &derivativeAccumulated));

    return attitude;
}