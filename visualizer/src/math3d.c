#include "math3d.h"

#include <math.h>

#include "raymath.h"

static float ClampFloat(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

Quaternion UmonQuaternionFromWxyz(const float source[4]) {
    Quaternion quaternion = {
        .x = source[1],
        .y = source[2],
        .z = source[3],
        .w = source[0],
    };
    return QuaternionNormalize(quaternion);
}

Quaternion UmonRelativeQuaternion(Quaternion current, Quaternion reference) {
    Quaternion inverse_reference = QuaternionInvert(reference);
    Quaternion relative = QuaternionMultiply(current, inverse_reference);
    return QuaternionNormalize(relative);
}

Vector3 UmonRotateVector(Vector3 vector, Quaternion orientation) {
    return Vector3RotateByQuaternion(vector, QuaternionNormalize(orientation));
}

float UmonQuaternionPitchDeg(Quaternion orientation) {
    float sinp = 2.0f * (orientation.w * orientation.y - orientation.z * orientation.x);
    float pitch = asinf(ClampFloat(sinp, -1.0f, 1.0f));
    return pitch * RAD2DEG;
}
