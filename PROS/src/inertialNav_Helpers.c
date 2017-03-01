#include "main.h"
#include <math.h>

//Vectors
Vector vectorNormalize(Vector original) {
    float magnitude = sqrt(powf(original.x, 2) + powf(original.y, 2) + powf(original.z, 2));
    Vector unit;
    unit.x = original.x / magnitude;
    unit.y = original.y / magnitude;
    unit.z = original.z / magnitude;
    return unit;
}
Vector vectorCrossProduct(Vector first, Vector second) {
    Vector perpendicular;
    perpendicular.x = (first.y * second.z) - (first.z * second.y);
    perpendicular.y = (first.z * second.x) - (first.x * second.z);
    perpendicular.z = (first.x * second.y) - (first.y * second.x);
    return perpendicular;
}
Vector vectorSum(Vector first, Vector second) {
    Vector sum;
    sum.x = first.x + second.x;
    sum.y = first.y + second.y;
    sum.z = first.z + second.z;
    return sum;
}
Vector vectorCoeff(float coeff, Vector vector) {
    Vector product;
    product.x = coeff * vector.x;
    product.y = coeff * vector.y;
    product.z = coeff * vector.z;
    return product;
}
Vector vectorIntegrate(Vector vector, Vector * storageVar) {
    Vector integral;
    integral.x = (storageVar->x += vector.x);
    integral.y = (storageVar->y += vector.y);
    integral.z = (storageVar->z += vector.z);
    return integral;
}

//Quaternions
Quaternion quatNormalize(Quaternion original) {
    float magnitude = sqrt(powf(original.w, 2) + powf(original.x, 2) + powf(original.y, 2) + powf(original.z, 2));
    Quaternion unit = {.w = original.w / magnitude,.x = original.x / magnitude,.y = original.y / magnitude,.z = original.z / magnitude};
    return unit;
}
Quaternion quatDifferentiate(Vector w, Quaternion q) {
    Quaternion dqdt;
    dqdt.w = -0.5 * ((w.x * q.x) + (w.y * q.y) + (w.z * q.z));
    dqdt.x = 0.5 * ((w.x * q.w) + (w.y * q.z) - (w.z * q.y));
    dqdt.y = 0.5 * ((w.y * q.w) + (w.z * q.x) - (w.x * q.z));
    dqdt.z = 0.5 * ((w.z * q.w) + (w.x * q.y) - (w.y * q.x));
    return dqdt;
}
Quaternion quatIntegrate(Quaternion quat, Quaternion * storageVar) {

}