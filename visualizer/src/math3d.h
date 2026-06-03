#ifndef UMON_MATH3D_H
#define UMON_MATH3D_H

#include "raylib.h"

Quaternion UmonQuaternionFromWxyz(const float source[4]);
Quaternion UmonRelativeQuaternion(Quaternion current, Quaternion reference);
Vector3 UmonRotateVector(Vector3 vector, Quaternion orientation);
float UmonQuaternionPitchDeg(Quaternion orientation);

#endif
