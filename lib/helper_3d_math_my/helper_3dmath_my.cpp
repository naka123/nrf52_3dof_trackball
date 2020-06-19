
#include "helper_3dmath_my.h"

Quaternion Quaternion::fromAngleAxis(float angle, VectorFloat &axis) {
    const float s = sin(angle/2);
    const float w = cos(angle/2);
    const float x = axis.x * s;
    const float y = axis.y * s;
    const float z = axis.z * s;
    return {w, x, y, z};
}

float Quaternion::toAngleAxis(VectorFloat &axis) const {
    const float s = sqrt(1 - w*w);
    const float angle = 2 * acos(w);
    axis.x = x / s;
    axis.y = y / s;
    axis.z = z / s;
    return angle;
}
