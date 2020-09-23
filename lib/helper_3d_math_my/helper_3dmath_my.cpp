
#include "helper_3dmath_my.h"

Quaternion Quaternion::fromAngleAxis(float angle, const VectorFloat &axis) {
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

void Quaternion::eulerAngles(ANGLES *e) {
    normalize();

    float_t test, sqx, sqy, sqz;

    test = x*y + z*w;
    if (test > 0.499) { // singularity at north pole
        e->roll = 2 * atan2(x,w);
        e->pitch = M_PI/2;
        e->yaw = 0;
        return;
    }
    if (test < -0.499) { // singularity at south pole
        e->roll = -2 * atan2(x,w);
        e->pitch = - M_PI/2;
        e->yaw = 0;
        return;
    }

    sqx = x*x;
    sqy = y*y;
    sqz = z*z;

    e->roll = atan2(2*y*w-2*x*z , 1 - 2*sqy - 2*sqz);
    e->pitch = asin(2*test);
    e->yaw = atan2(2*x*w-2*y*z , 1 - 2*sqx - 2*sqz);
}


Quaternion Quaternion::integrate(const VectorFloat& rate, const float timestep) {
    auto rotation_vector = rate * timestep;
    auto rotation_norm = rotation_vector.getMagnitude();
    Quaternion dq;
    if (rotation_norm > 0) {
        dq = Quaternion::fromAngleAxis(rotation_norm, rotation_vector / rotation_norm);
        dq.normalize();
    } else {
        dq = Quaternion();
    }
    return dq;
}

