#pragma once

//Vectors
typedef struct {
    float x;
    float y;
    float z;
} Vector;
Vector vectorNormalize(Vector original);
Vector vectorCrossProduct(Vector first, Vector second);
Vector vectorSum(Vector first, Vector second);
Vector vectorCoeff(float coeff, Vector vector);
Vector vectorIntegrate(Vector vector, float time, Vector * storageVar);

//Quaternions
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;
Quaternion quatNormalize(Quaternion original);
Quaternion quatDifferentiate(Vector w, Quaternion q);
Quaternion quatIntegrate(Quaternion quat, float time, Quaternion * storageVar);

//Accelerometers
void accelInit();
Vector accelRead();

//Background task
void inertialNavTask(void * parameter);